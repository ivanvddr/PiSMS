/*
 * Questo file fa parte del progetto PiSMS.
 *
 * Copyright (C) 2026 Ivan Vettori
 *
 * Questo file è opera originale dell'autore e viene rilasciato
 * sotto licenza GNU General Public License v3.0.
 * Vedi la licenza GPLv3 per maggiori dettagli.
 */

#pragma once
#include <Arduino.h>
#include <stdint.h>

#define VDP_RAMFUNC __attribute__((section(".time_critical.vdp")))

// 1 = i pixel in uscita sono già nel byte-order del TFT (usare
//     tft.setSwapBytes(false)).
#define VDP_PRESWAP_565 1

// ── Dimensioni schermo SMS ────────────────────────
static constexpr int SMS_SCREEN_W   = 256;
static constexpr int SMS_SCREEN_H   = 192;
static constexpr int SMS_VRAM_SIZE  = 0x4000;

// ── Costanti VDP ──────────────────────────────────
static constexpr int VDP_LINES_TOTAL    = 262;
static constexpr int VDP_LINES_ACTIVE   = 192;
static constexpr int VDP_COLS           = 256;
static constexpr int MAX_SPRITES_FRAME  = 64;
static constexpr int MAX_SPRITES_LINE   = 8;
static constexpr int VDP_REG_COUNT      = 11;

static constexpr uint8_t R0_VSYNC_IE    = 0x10;
static constexpr uint8_t R0_LSHIFT      = 0x08;
static constexpr uint8_t R0_M4          = 0x04;
static constexpr uint8_t R0_M2          = 0x02;
static constexpr uint8_t R0_OVERSCAN    = 0x20;

static constexpr uint8_t R1_SCREEN_ON   = 0x40;
static constexpr uint8_t R1_IE          = 0x20;
static constexpr uint8_t R1_M1          = 0x10;
static constexpr uint8_t R1_M3          = 0x08;
static constexpr uint8_t R1_SPR_16      = 0x02;
static constexpr uint8_t R1_SPR_DOUBLE  = 0x01;

static constexpr uint8_t STATUS_VBLANK    = 0x80;
static constexpr uint8_t STATUS_SPRITE_OV = 0x40;
static constexpr uint8_t STATUS_SPRITE_COL= 0x20;

// SMS colore 6 bit (BBGGRR) → RGB565
static inline uint16_t smsColorToRGB565(uint8_t c) {
    uint8_t r = (c & 0x03) * 85;
    uint8_t g = ((c >> 2) & 0x03) * 85;
    uint8_t b = ((c >> 4) & 0x03) * 85;
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// ─────────────────────────────────────────────────
//  Classe VDP
//
//  Architettura "snapshot":
//   Core0 emula Z80 + stato VDP. Quando Core1 è libera, a inizio frame
//   copia la VRAM (16 KB, ca 30 us) e durante le 192 linee registra per
//   ogni linea pochi byte di stato (scroll, registri, indice palette).
//   Core1 disegna TUTTO il frame dallo snapshot, a blocchi di linee,
//   e lo invia al display via DMA. Core0 non aspetta mai Core1: se
//   Core1 è ancora occupata col frame precedente, quel frame logico
//   semplicemente non viene catturato (frameskip adattivo).
//
//   Handshake: _snapState FREE > (Core0) READY > (Core1) BUSY > FREE
// ─────────────────────────────────────────────────
class VDP {
public:
    static constexpr int PAL_POOL = 64;   // snapshot palette per frame

    VDP();
    void reset();

    // ── Accesso I/O dalla CPU (Core0) ─────────────
    void    writeControl(uint8_t value);
    void    writeData(uint8_t value);
    uint8_t readData();
    uint8_t readStatus();

    // ── Timing per linea (Core0) ──────────────────
    void beginLine(int line)  { _currentLine = line; }
    void tickLineCounter(int line);
    void signalVBlank();

    // ── Cattura frame (Core0) ─────────────────────
    bool beginCapture();          // false se Core1 ancora occupata
    void captureLine(int line);   // dopo l'esecuzione CPU della linea
    void endCapture();            // frame completo > READY

    // ── Rendering (Core1) ─────────────────────────
    bool acquireSnapshot();       // true se c'è un frame da disegnare
    void renderChunk(int firstLine, int count, uint16_t* out);
    void releaseSnapshot();       // dopo l'ultimo renderChunk

    // ── Interrupt ─────────────────────────────────
    bool pendingVBlankIRQ() const { return _vblankIRQ; }
    bool pendingLineIRQ()   const { return _lineIRQ;   }
    void clearVBlankIRQ()         { _vblankIRQ = false; }
    void clearLineIRQ()           { _lineIRQ   = false; }

    int  currentLine() const { return _currentLine; }
    bool isVBlank()    const { return _currentLine >= VDP_LINES_ACTIVE; }

    uint8_t readVRAM(uint16_t addr) const { return _vram[addr & 0x3FFF]; }
    void    writeVRAM(uint16_t addr, uint8_t v) { _vram[addr & 0x3FFF] = v; }

private:
    enum : uint8_t { SNAP_FREE = 0, SNAP_READY = 1, SNAP_BUSY = 2 };

    struct LineState {
        uint8_t scrollX, scrollY;
        uint8_t r0, r1, r2, r5, r6, r7;
        uint8_t pal;                      // indice nel pool palette
    };

    struct Snapshot {
        uint8_t   vram[SMS_VRAM_SIZE];
        LineState line[VDP_LINES_ACTIVE];
        uint16_t  pal[PAL_POOL][32];      // RGB565 (non swappato)
        uint8_t   palCount;
    };

    // ── Stato "vivo" (Core0) ──────────────────────
    uint8_t  _vram[SMS_VRAM_SIZE];
    uint8_t  _cram[32];
    uint16_t _cramRGB[32];
    uint8_t  _reg[VDP_REG_COUNT];
    uint8_t  _statusFlags;
    uint16_t _addrLatch;
    uint8_t  _addrLatchByte;
    uint8_t  _codeReg;
    uint8_t  _readBuffer;
    bool     _cramWrite;
    uint8_t  _cramAddr;
    int      _currentLine;
    int      _lineCounter;
    bool     _vblankIRQ;
    bool     _lineIRQ;

    // ── Snapshot ──────────────────────────────────
    Snapshot _snap;
    volatile uint8_t _snapState;
    bool     _cramDirty;
    uint8_t  _curPal;

    void updateCRAM(uint8_t addr, uint8_t value);
    void renderBG(const LineState& L, int line, uint16_t* dst, const uint16_t* pal);
    void renderSprites(const LineState& L, int line, uint16_t* dst, const uint16_t* pal);
};