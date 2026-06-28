/*
 *
 * Questo file fa parte del progetto PiSMS.
 *
 * Copyright (C) 2026 Ivan Vettori
 *
 * Questo file è opera originale dell'autore e viene rilasciato
 * sotto licenza GNU General Public License v3.0.
 * Puoi ridistribuirlo e/o modificarlo secondo i termini della GPLv3.
 *
 * Questo programma è distribuito nella speranza che sia utile,
 * ma SENZA ALCUNA GARANZIA; senza neppure la garanzia implicita
 * di COMMERCIABILITÀ o IDONEITÀ PER UN PARTICOLARE SCOPO.
 * Vedi la licenza GPLv3 per maggiori dettagli.
 *
 * SMS Emulator per Raspberry Pi Pico — PiSMS
 *
 * Hardware:
 *   RP2040 @ 276 MHz overclocked
 *   Display ILI9341/ST7789 320×240 SPI + TFT_eSPI (DMA)
 *   Audio PWM pin 15 → filtro RC 2 stadi → PAM8403 (mono)
 *   2× FunduinoJoystick 5 vie + 2 pulsanti
 *   SD card: ROM in /PiSMS/games
 *
 * ─────────────────────────────────────────────────────────────────
 *  Architettura audio (DMA-driven, vedi PSG.h/PSG.cpp):
 *
 *   Core0: cpu.step() → psg.addCycles() → produce campioni nel ring
 *   DMA hardware (timer configurabile): consuma dal buffer attivo,
 *     scrive nel registro CC del PWM. Zero intervento CPU in steady
 *     state.
 *   IRQ refill (gira su Core1, ogni ~5.8ms, costo ~µs): rifornisce
 *     il buffer appena consumato leggendo dal ring.
 *
 *   Core0 non viene MAI interrotto per l'audio: lo Z80 e il VDP
 *   girano col loro timing originale, indisturbati.
 * ─────────────────────────────────────────────────────────────────
 *
 *  Architettura video:
 *   Core0: renderScanline() sul framebuffer (solo nei frame non-skip),
 *          segnala VBlank a Core1.
 *   Core1: pushImageDMA() → display. L'IRQ audio può scattare durante
 *          questo busy-wait: il NVIC la serve automaticamente, senza
 *          bisogno di "pomparla" manualmente come nelle versioni
 *          basate su busy-wait + micros().
 *
 * Memory map Z80 SMS:
 *   0x0000-0x3FFF  ROM slot 0  (bank da 0xFFFD)
 *   0x4000-0x7FFF  ROM slot 1  (bank da 0xFFFE)
 *   0x8000-0xBFFF  ROM slot 2  (bank da 0xFFFF) / RAM cart
 *   0xC000-0xFFFF  RAM sistema (8 KB + mirror)
 *
 * I/O map Z80 SMS:
 *   0x7E  R   V-counter
 *   0x7E  W   PSG (SN76489)
 *   0x7F  W   PSG mirror
 *   0xBE  R/W VDP data
 *   0xBF  R   VDP status
 *   0xBF  W   VDP control
 *   0xDC  R   Joystick port 1 + Joy2 Up/Down
 *   0xDD  R   Joystick port 2 Left/Right/Btn
 */

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "Z80.h"
#include "VDP.h"
#include "SMSLoader.h"
#include "PSG.h"
#include "FunduinoJoystick.h"
#include "SMSInput.h"
#include "PiSMSLogo.h"
#include "SMSMenu.h"
#include "SMSgamebrowser.h"

// ============================================================================
//  Forward declaration
//  Necessaria perché multicore_launch_core1(core1_main) viene chiamata
//  PRIMA della definizione della funzione: senza questo prototipo
//  esplicito, l'Arduino IDE può auto-generarne uno nel punto sbagliato
//  e rompere il parser.
// ============================================================================
void core1_main();

// ============================================================================
// PIN ASSIGNMENT
// ============================================================================

#define PIN_AUDIO       15    // PWM → filtro RC → PAM8403

#define ENABLE_JOYSTICK_1     1
#define ENABLE_JOYSTICK_2     1

// Joy 1
#define J1_UP           2
#define J1_DOWN         3
#define J1_LEFT         4
#define J1_RIGHT        5
#define J1_BTN1         6
#define J1_BTN2         26

// Joy 2
#define J2_UP           7
#define J2_DOWN         8
#define J2_LEFT         9
#define J2_RIGHT        1
#define J2_BTN1         0
#define J2_BTN2         28

// ── Frameskip ─────────────────────────────────────────────────────
// FRAMESKIP_N 1 = ogni frame (DMA video bottleneck, sconsigliato)
// FRAMESKIP_N 2 = 30fps display, 60fps logica
// FRAMESKIP_N 3 = 20fps display, 60fps logica
#define FRAMESKIP_N  3

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

enum AppState {
    STATE_MENU,
    STATE_GAME,
    STATE_OPT,
};

AppState currentState = STATE_MENU;
SMSMenu* menu = nullptr;
SMSGameBrowser* gameBrowser = nullptr;

// ============================================================================
// TIMING
// ============================================================================

// SMS NTSC: Z80 @ 3.579545 MHz, 60 Hz, 262 scanline/frame
// Cicli per scanline: ~228  
#define SMS_CPU_HZ          3579545UL
#define SMS_FRAME_HZ        60
#define SMS_LINES_TOTAL     262
#define SMS_LINES_ACTIVE    192
#define SMS_CYCLES_PER_LINE (SMS_CPU_HZ / SMS_FRAME_HZ / SMS_LINES_TOTAL)  // 228
#define SMS_TARGET_US       (1000000UL / SMS_FRAME_HZ)                     // 16667 µs

#define INPUT_POLL_CYCLES   7168

// ============================================================================
// CALLBACKS MEMORIA Z80
// ============================================================================

uint8_t smsMemRead(uint16_t addr) {
    return loader.read(addr);
}

void smsMemWrite(uint16_t addr, uint8_t value) {
    loader.write(addr, value);
}

// ============================================================================
// CALLBACKS I/O Z80
// ============================================================================

uint8_t smsIORead(uint8_t port) {
    // Il SMS decodifica le porte sui bit A6-A7
    // Molti giochi usano indirizzi "sporchi" (es. 0xDE invece di 0xDC)
    // La maschera 0xC1 copre i mirror principali

    switch (port & 0xFF) {

        // ── VDP ───────────────────────────────────
        case 0xBE:
            return vdp.readData();

        case 0xBF:
        case 0xBD:  // mirror
            return vdp.readStatus();

        // ── V-counter ─────────────────────────────
        // Linea corrente del VDP, usata da alcuni giochi
        // per effetti raster e sincronizzazione
        case 0x7E:
        case 0x7C:  // mirror (bit A1 ignorato)
            return (uint8_t)vdp.currentLine();

        // ── H-counter ─────────────────────────────
        // Non implementato: restituisce 0
        // Usato solo dalla Light Phaser (non supportata)
        case 0x7F:
        case 0x7D:
            return 0x00;

        // ── Joystick ──────────────────────────────
        case 0xDC: case 0xC0: case 0xD8: case 0xC8:
            return smsInput.readPort(0xDC);

        case 0xDD: case 0xC1: case 0xD9: case 0xC9:
            return smsInput.readPort(0xDD);

        default:
            return 0xFF;  // bus float
    }
}

void smsIOWrite(uint8_t port, uint8_t value) {
    switch (port & 0xFF) {

        // ── PSG ───────────────────────────────────
        // Port 0x7E e 0x7F, write-only
        // Nota: la scrittura PSG avviene su Core0,
        // il tick audio su Core1 → shadow register in PSG
        case 0x7E:
        case 0x7F:
            psg.write(value);
            break;

        // ── VDP data ──────────────────────────────
        case 0xBE:
            vdp.writeData(value);
            break;

        // ── VDP control ───────────────────────────
        case 0xBF:
        case 0xBD:
            vdp.writeControl(value);
            break;

        // ── Memory control register (0x3E) ────────
        // Controlla accesso a ROM, RAM, I/O, cartuccia
        // Non implementato: la maggior parte dei giochi
        // non lo usa esplicitamente
        case 0x3E:
            break;

        default:
            break;
    }
}

void rebootPicoISR() {
    rp2040.restart();
}

inline void initJoysticks(FunduinoJoystick** joy1, FunduinoJoystick** joy2) {
    #if ENABLE_JOYSTICK_1
        *joy1 = new FunduinoJoystick(JOYSTICK_PORT_1);
        (*joy1)->begin(J1_UP , J1_DOWN, J1_LEFT, J1_RIGHT, J1_BTN1);
        Serial.println("✅ Joystick Port 1 initialized");
    #else
        *joy1 = nullptr;
        Serial.println("⚠️  Joystick Port 1 disabled");
    #endif
    
    #if ENABLE_JOYSTICK_2
        *joy2 = new FunduinoJoystick(JOYSTICK_PORT_2);
        (*joy2)->begin(J2_UP , J2_DOWN, J2_LEFT, J2_RIGHT, J2_BTN1);
        Serial.println("✅ Joystick Port 2 initialized");
    #else
        *joy2 = nullptr;
        Serial.println("⚠️  Joystick Port 2 disabled");
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
        // TFT_eSPI usa di default channel 0 e 1
        dma_channel_abort(0);
        dma_channel_abort(1);
    }
    
    tft.fillScreen(TFT_BLACK);
    delay(50); // aspetta che fillScreen finisca
    
    tft.setViewport(0, 0, 320, 240);
    tft.setTextDatum(TL_DATUM);
    
    Serial.println("TFT clean");
}

void resetAllDMA() {
    Serial.println("Reset DMA channels...");
    
    // Aspetta che il TFT finisca prima di toccare qualsiasi canale
    tft.dmaWait();
    tft.endWrite();
    
    // Non fare abort su tutti i canali: distrugge lo stato TFT_eSPI
    // Basta assicurarsi che non ci siano trasferimenti pendenti
    delay(20);
    
    Serial.println("DMA reset complete");
}

// ============================================================================
// SETUP EMULATORE
// ============================================================================

void SMSsetup() {
    // Ferma Core1 se già in esecuzione
    multicore_reset_core1();
    delay(10);

    Serial.println("[SMS] Init display...");
    tft.init();
    tft.initDMA();
    tft.setRotation(1);
    tft.setSwapBytes(true);
    tft.fillScreen(TFT_BLACK);

    Serial.println("[SMS] Init VDP...");
    vdp.reset();

    Serial.println("[SMS] Init PSG (DMA-driven)...");
    // Configura PWM + claima canale/timer DMA + prepara i buffer.
    // NON avvia il DMA: lo fa Core1 in startAudioOnThisCore(), per
    // far girare l'IRQ di refill lì e non disturbare mai Core0.
    psg.initPWMDMA(PIN_AUDIO);

    Serial.println("[SMS] Init input...");
    smsInput.begin(
        J1_UP, J1_DOWN, J1_LEFT, J1_RIGHT, J1_BTN1, J1_BTN2,
        J2_UP, J2_DOWN, J2_LEFT, J2_RIGHT, J2_BTN1, J2_BTN2
    );

    Serial.println("[SMS] Init Z80...");
    // Stesso pattern del PiMSX: callback statiche
    cpu.begin(smsMemRead, smsMemWrite, smsIORead, smsIOWrite);
    cpu.reset();

    Serial.println("[SMS] Avvio Core1...");
    multicore_launch_core1(core1_main);
    delay(100);  // attendi init Core1

    Serial.println("[SMS] Pronto\n");
}

// ============================================================================
// CORE 1 — entry point e loop
// (pattern identico a startDisplayCore() nel PiMSX)
// ============================================================================

void core1_main() {
    // Abilita l'IRQ DMA SU QUESTO CORE e avvia il trasferimento.
    // Da qui in poi l'audio è autonomo: Core1 può dedicarsi
    // interamente al display.
    psg.startAudioOnThisCore();

    while (true) {

        // ── Display DMA ───────────────────────────────────────────
        // Scatta solo ogni FRAMESKIP_N frame: Core1 è idle per la
        // maggior parte del tempo e può dedicarsi interamente all'audio.
        if (vdp.frameReady) {
            vdp.frameReady   = false;
            vdp.frameSending = true;

            constexpr int X_OFF = (320 - SMS_SCREEN_W) / 2;
            constexpr int Y_OFF = (240 - SMS_SCREEN_H) / 2;

            tft.startWrite();
            tft.pushImageDMA(X_OFF, Y_OFF, SMS_SCREEN_W, SMS_SCREEN_H,
                             vdp.framebuffer);

            while (tft.dmaBusy()) { /* busy-wait */ }

            tft.dmaWait();
            tft.endWrite();

            vdp.frameSending = false;
        }
    }
}

// ============================================================================
// LOOP PRINCIPALE (Core0)
// Struttura identica al MSXloop() del PiMSX:
//   - scanline attive con cpu.step() + vdp.tick()
//   - VBlank + IRQ
//   - blanking
//   - throttle frame
// ============================================================================



void SMSloop() {
    uint32_t lastFrameUs  = micros();
    int      inputCycles  = 0;
    uint8_t  frameCounter = 0;
    uint32_t lastDebugMs  = 0;
    uint32_t lastUnderruns = 0;
    uint32_t lastWrites    = 0;

    // ── Profiling: accumulatori per fase, reset ogni 2s ───────────
    // Misurati con micros() (risoluzione 1µs): l'M0+ non ha un
    // contatore cicli (DWT), quindi questa è la massima risoluzione
    // disponibile. L'overhead delle chiamate a micros() stesse è
    // dell'ordine dell'1% del totale, trascurabile per individuare
    // quale fase domina.
    uint32_t profInputUs     = 0;
    uint32_t profCpuActiveUs = 0;
    uint32_t profPsgUs       = 0;
    uint32_t profRenderUs    = 0;
    uint32_t profCpuBlankUs  = 0;
    uint32_t profFrames      = 0;

    while (true) {

        bool renderThisFrame = (frameCounter % FRAMESKIP_N == 0);
        frameCounter++;
        profFrames++;

        // ── Scanline attive 0-191 ─────────────────────────────────
        for (int line = 0; line < SMS_LINES_ACTIVE; line++) {

            uint32_t t0 = micros();

            inputCycles += SMS_CYCLES_PER_LINE;
            if (inputCycles >= INPUT_POLL_CYCLES) {
                inputCycles = 0;
                smsInput.update();
            }

            uint32_t t1 = micros();
            profInputUs += (t1 - t0);

            int cyclesRun = 0;
            while (cyclesRun < SMS_CYCLES_PER_LINE) {
                int c = cpu.step();
                cyclesRun += c;
                if (vdp.pendingLineIRQ() && cpu.IFF1) {
                    cpu.triggerINT(0xFF);
                    vdp.clearLineIRQ();
                }
            }

            uint32_t t2 = micros();
            profCpuActiveUs += (t2 - t1);

            // PSG: gira sempre a 60fps interni, mai saltare.
            //psg.addCycles(SMS_CYCLES_PER_LINE);
            psg.addCycles(cyclesRun);

            uint32_t t3 = micros();
            profPsgUs += (t3 - t2);

            if (renderThisFrame) {
                vdp.renderScanline(line);
            }

            uint32_t t4 = micros();
            profRenderUs += (t4 - t3);

            vdp.tickLineCounter(line);
        }

        // ── VBlank ───────────────────────────────────────────────
        vdp.signalVBlank();
        if (!renderThisFrame) {
            vdp.frameReady = false;
        }

        if (vdp.pendingVBlankIRQ() && cpu.IFF1) {
            cpu.triggerINT(0xFF);
            vdp.clearVBlankIRQ();
        }

        // ── Blanking linee 192-261 ────────────────────────────────
        for (int line = 0; line < (SMS_LINES_TOTAL - SMS_LINES_ACTIVE); line++) {
            uint32_t tb0 = micros();

            int cyclesRun = 0;
            while (cyclesRun < SMS_CYCLES_PER_LINE) {
                int c = cpu.step();
                cyclesRun += c;
                if (vdp.pendingVBlankIRQ() && cpu.IFF1) {
                    cpu.triggerINT(0xFF);
                    vdp.clearVBlankIRQ();
                }
            }

            uint32_t tb1 = micros();
            profCpuBlankUs += (tb1 - tb0);

            //psg.addCycles(SMS_CYCLES_PER_LINE);
            psg.addCycles(cyclesRun);


            uint32_t tb2 = micros();
            profPsgUs += (tb2 - tb1);
        }

        // ── Diagnostica ogni 2s ───────────────────────────────────
        int32_t debtUs = (int32_t)(micros() - (lastFrameUs + SMS_TARGET_US));
        /*
        uint32_t nowMs = millis();
        if (nowMs - lastDebugMs >= 2000) {
            lastDebugMs = nowMs;
            uint32_t u = psg.underruns();
            uint32_t w = psg.writeCount();
            Serial.printf(
                "[PSG] underruns/2s=%lu  ring=%d/%d  writes/2s=%lu  "
                "vol(T0,T1,T2,N)=%d,%d,%d,%d  debt=%ldus\n",
                u - lastUnderruns, psg.ringUsed(), RING_SIZE,
                w - lastWrites,
                psg.volumeReg(0), psg.volumeReg(1),
                psg.volumeReg(2), psg.volumeReg(3),
                (long)debtUs);
            lastUnderruns = u;
            lastWrites    = w;

            // ── Stampa profiling: tempo per fase su finestra 2s ────
            // Percentuale = tempo fase / 2.000.000us totali disponibili.
            // idleUs = budget rimasto per il throttle (negativo = Core0
            // già in debito prima di arrivare al throttle).
            uint32_t sumUs = profInputUs + profCpuActiveUs + profPsgUs
                           + profRenderUs + profCpuBlankUs;
            int32_t idleUs = 2000000L - (int32_t)sumUs;

            Serial.printf(
                "[PROF] frames=%lu(att.%d)  input=%lums(%.1f%%)  "
                "cpuAtt=%lums(%.1f%%)  psg=%lums(%.1f%%)  "
                "render=%lums(%.1f%%)  cpuBlank=%lums(%.1f%%)  idle=%ldms\n",
                profFrames, SMS_FRAME_HZ * 2,
                profInputUs/1000,     100.0f*profInputUs/2000000.0f,
                profCpuActiveUs/1000, 100.0f*profCpuActiveUs/2000000.0f,
                profPsgUs/1000,       100.0f*profPsgUs/2000000.0f,
                profRenderUs/1000,    100.0f*profRenderUs/2000000.0f,
                profCpuBlankUs/1000,  100.0f*profCpuBlankUs/2000000.0f,
                (long)(idleUs/1000));
    
            profInputUs = profCpuActiveUs = profPsgUs = 0;
            profRenderUs = profCpuBlankUs = profFrames = 0;
        }
        */
        // ── Frame throttle ────────────────────────────────────────
        const uint32_t frameEnd = lastFrameUs + SMS_TARGET_US;
        while ((int32_t)(micros() - frameEnd) < 0) { /* wait */ }
        lastFrameUs = frameEnd;
    }
}


void handleGameBrowser() {
     // Crea browser solo una volta
    if (!gameBrowser) {
        // USA LOADER per init filesystem
        // Prova prima LittleFS, poi fallback su SD
        if (!loader.initFilesystem(FSType::FS_SD,SD_CS)) {
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
        
        // Crea browser (passa il filesystem dal Loader)
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
        
        // Browser OK, libera menu
        delete menu;
        menu = nullptr;
    }
    
    // Usa joystick per navigare
    FunduinoJoystick* activeJoy = (joystick1 != nullptr) ? joystick1 : joystick2;
    uint8_t joyState = activeJoy->read();
    
    // Aggiorna browser
    bool gameSelected = gameBrowser->update(joyState);
    
    if (gameSelected) {
        Serial.println("Game selected!");

        // Browser salva il path nel Loader
        gameBrowser->getPrgSelected();
        
        // ========================================
        // STEP 1: Libera browser
        // ========================================
        delete gameBrowser;
        gameBrowser = nullptr;
        Serial.println("Browser destroyed");
                
        // ========================================
        // STEP 2: Reset hardware
        // ========================================
        resetAllDMA();
        cleanupTFT();
        delay(100);
        
               
        // ========================================
        // STEP 3: Carica gioco con Loader!
        // ========================================
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
            // Nessuna azione
            break;
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== SMS Pico Emulator ===");

    // Overclock
    if (!set_sys_clock_khz(276000, true)) {
        Serial.println("WARNING: overclock 276MHz fallito");
    } else {
        Serial.printf("Clock: %lu Hz\n", clock_get_hz(clk_sys));
    }

    tft.init();
    tft.initDMA();
    tft.setRotation(1); // 320x240

    tft.fillScreen(TFT_BLACK);

    // Inizializza joystick e usa quello attivo
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
           // handleOpts();
            break;

      } 

}
