/*
 * Questo file fa parte del progetto PiSMS.
 *
 * Copyright (C) 2026 Ivan Vettori
 *
 * Rilasciato sotto licenza GNU General Public License v3.0.
 *
 * SMS Emulator per Raspberry Pi Pico — PiSMS
 *
 * Hardware:
 *   RP2040 @ 276 MHz overclocked
 *   Display ILI9341/ST7789 320×240 SPI + TFT_eSPI (DMA)
 *   Audio: MAX98357 I2S  (BCLK GP26, LRC GP27, DIN GP28)
 *   2× FunduinoJoystick 5 vie + 2 pulsanti
 *   SD card: ROM in /games
 *
 * ─────────────────────────────────────────────────────────────────
 *  Architettura (v3)
 *
 *   Core0  Z80 + stato VDP + joystick. NON ASPETTA MAI Core1:
 *            - a inizio frame, se Core1 è libera, copia la VRAM e poi
 *              registra lo stato di ogni linea (pochi byte);
 *            - le scritture PSG vengono solo accodate con timestamp.
 *   Core1  - disegna il frame dallo snapshot (a blocchi di linee) e lo
 *            invia al TFT in DMA;
 *          - sintetizza l'audio PSG e alimenta l'I2S (pump() chiamata
 *            di continuo, anche durante le attese DMA).
 *
 *   Se Core1 è ancora occupata col frame precedente, il frame logico
 *   successivo non viene catturato: si sacrifica il framerate, mai la
 *   velocità di emulazione.
 * ─────────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/dma.h"
#include "Z80.h"
#include "VDP.h"
#include "SMSLoader.h"
#include "PSG.h"
#include "FunduinoJoystick.h"
#include "SMSInput.h"
#include "PiSMSLogo.h"
#include "SMSMenu.h"
#include "SMSgamebrowser.h"

void core1_main();

#define RAMFUNC __attribute__((section(".time_critical.")))
#define PROFILING_ENABLED 0

// ============================================================================
// PIN
// ============================================================================

#define PICO_CLOCK_KHZ 300000

// I2S MAX98357: LRC = BCLK+1 (GP27) è imposto dal programma PIO
#define PIN_I2S_BCLK    26
#define PIN_I2S_DIN     28

#define ENABLE_JOYSTICK_1     1
#define ENABLE_JOYSTICK_2     1

// Joy 1
#define J1_UP           2
#define J1_DOWN         3
#define J1_LEFT         4
#define J1_RIGHT        5
#define J1_BTN1         6
#define J1_BTN2         7    

// Joy 2
#define J2_UP           11
#define J2_DOWN         10
#define J2_LEFT         9
#define J2_RIGHT        8
#define J2_BTN1         22
#define J2_BTN2         0   

// ── Frame rate del display ────────────────────────────────────────
// Numero minimo di frame logici tra due frame catturati per il display.
// 1 = cattura ogni volta che Core1 è libera (frameskip adattivo).
// 2 = al massimo 30 fps, 3 = al massimo 20 fps ...
#define MIN_FRAMES_PER_CAPTURE  1

// Righe per blocco di rendering/DMA (192 deve essere multiplo)
#define CHUNK_LINES  8

// ============================================================================
// OGGETTI GLOBALI
// ============================================================================

TFT_eSPI        tft;
VDP             vdp;
SMSLoader       loader;
PSG             psg;
SMSInput        smsInput;
Z80             cpu;
FunduinoJoystick* joystick1 = nullptr;
FunduinoJoystick* joystick2 = nullptr;

enum AppState { STATE_MENU, STATE_GAME, STATE_OPT };

AppState currentState = STATE_MENU;
SMSMenu* menu = nullptr;
SMSGameBrowser* gameBrowser = nullptr;

// Buffer di uscita Core1 (ping-pong)
static uint16_t chunkBuf[2][SMS_SCREEN_W * CHUNK_LINES] __attribute__((aligned(4)));

// ============================================================================
// TIMING
// ============================================================================

// SMS NTSC: 228 cicli × 262 linee = 59736 cicli/frame.
// Per avere esattamente 3.579... MHz il frame dura 1e6/59.92 ca 16689 µs.
#define SMS_FRAME_HZ        60
#define SMS_LINES_TOTAL     262
#define SMS_LINES_ACTIVE    192
#define SMS_CYCLES_PER_LINE 228
#define SMS_TARGET_US       16689UL

// ============================================================================
// CALLBACKS MEMORIA (solo percorso lento: le letture e le scritture RAM
// vanno direttamente dalle tabelle di pagina del Z80)
// ============================================================================

RAMFUNC uint8_t smsMemRead(uint16_t addr) {
    return loader.read(addr);
}

RAMFUNC void smsMemWrite(uint16_t addr, uint8_t value) {
    loader.writeSlow(addr, value);
}

// ============================================================================
// CALLBACKS I/O Z80
//
// Decodifica reale SMS: contano solo i bit A7, A6 e A0.
//   01 ........  A0=0: V-counter (R) / PSG (W)   A0=1: H-counter (R) / PSG (W)
//   10 ........  A0=0: VDP data                  A0=1: VDP control/status
//   11 ........  A0=0: joystick porta A (0xDC)   A0=1: porta B (0xDD)
// ============================================================================

RAMFUNC uint8_t smsIORead(uint8_t port) {
    switch (port & 0xC1) {
        case 0x40: return (uint8_t)vdp.currentLine();
        case 0x41: return 0x00;                      // H-counter non implementato
        case 0x80: return vdp.readData();
        case 0x81: return vdp.readStatus();
        case 0xC0: return smsInput.readPort(0xDC);
        case 0xC1: return smsInput.readPort(0xDD);
        default:   return 0xFF;
    }
}

RAMFUNC void smsIOWrite(uint8_t port, uint8_t value) {
    switch (port & 0xC1) {
        case 0x40:
        case 0x41: psg.write(value);        break;   // accodata, non bloccante
        case 0x80: vdp.writeData(value);    break;
        case 0x81: vdp.writeControl(value); break;
        default: break;                              // 0x3E/0x3F: non implementati
    }
}

void rebootPicoISR() {
    rp2040.restart();
}

inline void initJoysticks(FunduinoJoystick** joy1, FunduinoJoystick** joy2) {
    #if ENABLE_JOYSTICK_1
        *joy1 = new FunduinoJoystick(JOYSTICK_PORT_1);
        (*joy1)->begin(J1_UP , J1_DOWN, J1_LEFT, J1_RIGHT, J1_BTN1);
        Serial.println("Joystick Port 1 initialized");
    #else
        *joy1 = nullptr;
        Serial.println("Joystick Port 1 disabled");
    #endif

    #if ENABLE_JOYSTICK_2
        *joy2 = new FunduinoJoystick(JOYSTICK_PORT_2);
        (*joy2)->begin(J2_UP , J2_DOWN, J2_LEFT, J2_RIGHT, J2_BTN1);
        Serial.println("Joystick Port 2 initialized");
    #else
        *joy2 = nullptr;
        Serial.println("Joystick Port 2 disabled");
    #endif
}

void cleanupTFT() {
    Serial.println("Cleanup TFT...");

    tft.endWrite();

    uint32_t timeout = millis();
    while (tft.dmaBusy() && (millis() - timeout < 200)) {
        delay(1);
    }

    if (tft.dmaBusy()) {
        Serial.println("TFT DMA force stop!");
        dma_channel_abort(0);
        dma_channel_abort(1);
    }

    tft.fillScreen(TFT_BLACK);
    delay(50);

    tft.setViewport(0, 0, 320, 240);
    tft.setTextDatum(TL_DATUM);

    Serial.println("TFT clean");
}

void resetAllDMA() {
    Serial.println("Reset DMA channels...");
    tft.dmaWait();
    tft.endWrite();
    delay(20);
    Serial.println("DMA reset complete");
}

// ============================================================================
// SETUP EMULATORE
// ============================================================================

void SMSsetup() {
    multicore_reset_core1();
    delay(10);

    Serial.println("[SMS] Init display...");
    tft.init();
    tft.initDMA();
    tft.setRotation(1);
    // Core1 produce già i pixel nel byte-order del TFT (VDP_PRESWAP_565)
    tft.setSwapBytes(!VDP_PRESWAP_565);
    tft.fillScreen(TFT_BLACK);

    Serial.println("[SMS] Init VDP...");
    vdp.reset();

    Serial.println("[SMS] Init input...");
    smsInput.begin(
        J1_UP, J1_DOWN, J1_LEFT, J1_RIGHT, J1_BTN1, J1_BTN2,
        J2_UP, J2_DOWN, J2_LEFT, J2_RIGHT, J2_BTN1, J2_BTN2
    );

    Serial.println("[SMS] Init Z80...");
    cpu.begin(smsMemRead, smsMemWrite, smsIORead, smsIOWrite);
    cpu.setPages(loader.readPages(), loader.writePages());   // accesso memoria diretto
    cpu.reset();                                             // totalCycles = 0

    Serial.println("[SMS] Init PSG (I2S)...");
    psg.reset();
    // Il PSG legge il tempo di Core0 da cpu.totalCycles per timbrare gli eventi
    psg.configure(PIN_I2S_BCLK, PIN_I2S_DIN, &cpu.totalCycles);

    Serial.println("[SMS] Avvio Core1...");
    multicore_launch_core1(core1_main);
    delay(100);

    Serial.printf("[MEM] heap libero=%u / totale=%u byte\n",
                  (unsigned)rp2040.getFreeHeap(), (unsigned)rp2040.getTotalHeap());
    Serial.println("[SMS] Pronto\n");
}

// ============================================================================
// CORE 1 — rendering + audio
// ============================================================================

RAMFUNC void core1_main() {
    // I2S parte su questo core: il suo IRQ DMA non tocca mai Core0
    psg.beginAudio();

    constexpr int X_OFF = (320 - SMS_SCREEN_W) / 2;
    constexpr int Y_OFF = (240 - SMS_SCREEN_H) / 2;

    while (true) {
        psg.pump();

        if (vdp.acquireSnapshot()) {
            tft.startWrite();

            int b = 0;
            for (int y = 0; y < SMS_SCREEN_H; y += CHUNK_LINES) {
                // Disegna il blocco mentre il DMA sta ancora inviando il precedente
                vdp.renderChunk(y, CHUNK_LINES, chunkBuf[b]);
                psg.pump();

                // Il DMA precedente deve aver finito prima di partire col nuovo;
                // nell'attesa si continua a servire l'audio
                while (tft.dmaBusy()) psg.pump();

                tft.pushImageDMA(X_OFF, Y_OFF + y, SMS_SCREEN_W, CHUNK_LINES, chunkBuf[b]);
                b ^= 1;
            }

            // Lo snapshot non serve più: Core0 può già catturare il frame dopo
            vdp.releaseSnapshot();

            while (tft.dmaBusy()) psg.pump();
            tft.endWrite();
        } else {
            tight_loop_contents();
        }
    }
}

// ============================================================================
// LOOP PRINCIPALE (Core0)
// ============================================================================

// Esegue una linea di Z80. 'carry' riporta i cicli in eccesso alla linea
// successiva: nessun ciclo viene perso 
inline __attribute__((always_inline)) void runLine(int& carry, bool active) {
    int cyc = carry;
    if (active) {
        while (cyc < SMS_CYCLES_PER_LINE) {
            cyc += cpu.step();
            if (vdp.pendingLineIRQ() && cpu.IFF1) {
                cpu.triggerINT(0xFF);
                vdp.clearLineIRQ();
            }
        }
    } else {
        while (cyc < SMS_CYCLES_PER_LINE) {
            cyc += cpu.step();
            if (vdp.pendingVBlankIRQ() && cpu.IFF1) {
                cpu.triggerINT(0xFF);
                vdp.clearVBlankIRQ();
            }
        }
    }
    carry = cyc - SMS_CYCLES_PER_LINE;
}

RAMFUNC void SMSloop() {
    uint32_t lastFrameUs  = micros();
    int      carry        = 0;
    uint8_t  sinceCapture = MIN_FRAMES_PER_CAPTURE;

#if PROFILING_ENABLED
    uint32_t lastDebugMs  = millis();
    uint32_t profBusyUs   = 0;
    uint32_t profFrames   = 0;
    uint32_t profCaptured = 0;
    Serial.println("[PROF] Profiling abilitato. Primo report fra 2s...");
#endif

    while (true) {
#if PROFILING_ENABLED
        uint32_t tFrame0 = micros();
        profFrames++;
#endif

        // Input: una volta per frame (i giochi lo leggono una volta per frame)
        smsInput.update();

        // Cattura il frame solo se Core1 è libera: altrimenti salta, senza attendere
        bool capture = false;
        if (sinceCapture >= MIN_FRAMES_PER_CAPTURE) capture = vdp.beginCapture();
        if (capture)                    sinceCapture = 1;
        else if (sinceCapture < 255)    sinceCapture++;

#if PROFILING_ENABLED
        if (capture) profCaptured++;
#endif

        // ── Scanline attive 0-191 ─────────────────────────────────
        for (int line = 0; line < SMS_LINES_ACTIVE; line++) {
            vdp.beginLine(line);
            runLine(carry, true);
            if (capture) vdp.captureLine(line);
            vdp.tickLineCounter(line);
        }

        // ── VBlank ───────────────────────────────────────────────
        vdp.signalVBlank();
        if (capture) vdp.endCapture();       // Core1 può partire subito

        if (vdp.pendingVBlankIRQ() && cpu.IFF1) {
            cpu.triggerINT(0xFF);
            vdp.clearVBlankIRQ();
        }

        // ── Blanking linee 192-261 ────────────────────────────────
        for (int line = 0; line < (SMS_LINES_TOTAL - SMS_LINES_ACTIVE); line++) {
            runLine(carry, false);
        }

        // ── Frame throttle ────────────────────────────────────────
        uint32_t frameEnd = lastFrameUs + SMS_TARGET_US;

#if PROFILING_ENABLED
        profBusyUs += (micros() - tFrame0);
#endif

        if (psg.audioRunning()) {
            // ── Sync guidato dall'audio ───────────────────────────
            // Il clock I2S (44100 Hz) è il master: si attende finché i
            // cicli emulati-ma-non-ancora-suonati scendono sotto il target.
            // La velocità di emulazione segue così l'audio, senza deriva.
            // Il timeout evita blocchi se Core1/I2S si fermano.
            uint32_t tWait = micros();
            while (psg.leadCycles() > (int32_t)AUDIO_TARGET_LEAD_CYCLES) {
                if ((uint32_t)(micros() - tWait) > 3 * SMS_TARGET_US) break;
                tight_loop_contents();
            }
            lastFrameUs = micros();
        } else {
            // Fallback a tempo (I2S non attivo)
            if ((int32_t)(micros() - frameEnd) > 100000) frameEnd = micros();
            while ((int32_t)(micros() - frameEnd) < 0) {
                tight_loop_contents();
            }
            lastFrameUs = frameEnd;
        }

#if PROFILING_ENABLED
        uint32_t nowMs = millis();
        if (nowMs - lastDebugMs >= 2000) {
            float secs = (nowMs - lastDebugMs) / 1000.0f;
            lastDebugMs = nowMs;
            Serial.printf(
                "[PROF] emu=%.1ffps  display=%.1ffps  Core0 occupata=%.1f%%  "
                "| PSG: starved=%lu i2sUnderrun=%lu gapMax=%luus evDrop=%lu evQ=%d vol=%d,%d,%d,%d\n",
                profFrames / secs, profCaptured / secs,
                100.0f * profBusyUs / (secs * 1000000.0f),
                (unsigned long)psg.starved(), (unsigned long)psg.underflows(),
                (unsigned long)psg.takeMaxGapUs(), (unsigned long)psg.eventsDropped(),
                psg.eventsPending(),
                psg.volumeReg(0), psg.volumeReg(1),
                psg.volumeReg(2), psg.volumeReg(3));
            psg.traceDump();      // stampa la traccia PSG una volta sola
            profBusyUs = profFrames = profCaptured = 0;
        }
#endif
    }
}

// ============================================================================
// BROWSER / MENU
// ============================================================================
void handleGameBrowser() {
    if (!gameBrowser) {
        if (!loader.initFilesystem(FSType::FS_SD, SD_CS)) {
            Serial.println("No filesystem available!");
            tft.fillScreen(TFT_RED);
            tft.setTextColor(TFT_WHITE);
            tft.setTextSize(2);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("NO FILESYSTEM!", 160, 120);
            delay(3000);
            currentState = STATE_MENU;
            menu->draw();
            return;
        }

        gameBrowser = new SMSGameBrowser(&tft, &loader, SD_CS);
        gameBrowser->setLogo(logoPiSMS_small, 60, 30);
        gameBrowser->setPath("/games");
        gameBrowser->setFileExt(".sms");

        if (!gameBrowser->begin(loader.getCurrentFilesystem())) {
            Serial.println("Browser init failed!");
            delete gameBrowser;
            gameBrowser = nullptr;
            currentState = STATE_MENU;
            menu->draw();
            return;
        }

        delete menu;
        menu = nullptr;
    }

    FunduinoJoystick* activeJoy = (joystick1 != nullptr) ? joystick1 : joystick2;
    uint8_t joyState = activeJoy->read();

    bool gameSelected = gameBrowser->update(joyState);

    if (gameSelected) {
        Serial.println("Game selected!");

        gameBrowser->getPrgSelected();

        delete gameBrowser;
        gameBrowser = nullptr;
        Serial.println("Browser destroyed");

        resetAllDMA();
        cleanupTFT();
        delay(100);

        if (!loader.loadROM(loader.getSelectedROM())) {
            Serial.println("Loading error!");
            tft.fillScreen(TFT_WHITE);
            tft.setTextColor(TFT_RED);
            tft.setTextSize(2);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("LOAD ERROR!", 160, 120);
            delay(3000);
            return;
        }

        activeJoy = nullptr;
        joystick1 = nullptr;
        joystick2 = nullptr;

        loader.debugStatus();

        SMSsetup();
        SMSloop();
    }

    delay(10);
    yield();
}

void handleMenu() {
    MenuAction action = menu->update();

    switch (action) {
        case ACTION_START_GAME:
            Serial.println("GAME menu start...");
            currentState = STATE_GAME;
            break;

        case ACTION_START_OPT:
            Serial.println("OPTIONS menu start...");
            currentState = STATE_OPT;
            break;

        case ACTION_NONE:
        default:
            break;
    }
}

void setup() {
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    set_sys_clock_khz(PICO_CLOCK_KHZ, true);
    clock_configure(
      clk_peri,
      0,
      CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
      PICO_CLOCK_KHZ * 1000,
      PICO_CLOCK_KHZ * 1000
    );

    delay(100);

    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== SMS Pico Emulator ===");

    tft.init();
    tft.initDMA();
    tft.setRotation(1);

    tft.fillScreen(TFT_BLACK);

    initJoysticks(&joystick1, &joystick2);
    FunduinoJoystick* activeJoy = (joystick1 != nullptr) ? joystick1 : joystick2;

    if (activeJoy == nullptr) {
        Serial.println("Error: No joystick active!");
        tft.setTextColor(TFT_RED);
        tft.drawString("JOYSTICK ERROR!", 160, 120);
        while(1);
    }

    menu = new SMSMenu(&tft, activeJoy);
    menu->setLogo(logoPiSMS, 143, 70);

    menu->begin();

    Serial.println("Setup completed!");
}

void loop() {
    switch (currentState) {
        case STATE_MENU:
            handleMenu();
            break;

        case STATE_GAME:
            handleGameBrowser();
            break;

        case STATE_OPT:
            break;
    }
}
