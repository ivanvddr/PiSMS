/*
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
 */
 
#pragma once
#include <Arduino.h>
#include <stdint.h>

// ── Dimensioni schermo SMS ────────────────────────
static constexpr int SMS_SCREEN_W   = 256;
static constexpr int SMS_SCREEN_H   = 192;   // modalità standard; 224 per modalità estesa
static constexpr int SMS_VRAM_SIZE  = 0x4000; // 16 KB

// ── Costanti VDP ──────────────────────────────────
static constexpr int VDP_LINES_TOTAL    = 262;  // linee totali per frame (NTSC)
static constexpr int VDP_LINES_ACTIVE   = 192;  // linee visibili standard
static constexpr int VDP_COLS           = 256;
static constexpr int MAX_SPRITES_FRAME  = 64;
static constexpr int MAX_SPRITES_LINE   = 8;

// ── Registri VDP (R0–R10) ─────────────────────────
static constexpr int VDP_REG_COUNT = 11;

// ── Flag registro 0 ───────────────────────────────
static constexpr uint8_t R0_VSYNC_IE    = 0x10; // abilita interrupt V-Blank
static constexpr uint8_t R0_LSHIFT      = 0x08; // shift colonne di 8px a sinistra
static constexpr uint8_t R0_M4          = 0x04; // Mode 4 (SMS nativa)
static constexpr uint8_t R0_M2          = 0x02; // modalità 2
static constexpr uint8_t R0_OVERSCAN    = 0x20; // bordo overscan

// ── Flag registro 1 ───────────────────────────────
static constexpr uint8_t R1_SCREEN_ON   = 0x40; // display attivo
static constexpr uint8_t R1_IE          = 0x20; // interrupt abilitato
static constexpr uint8_t R1_M1          = 0x10; // modalità 1
static constexpr uint8_t R1_M3          = 0x08; // modalità 3
static constexpr uint8_t R1_SPR_16      = 0x02; // sprite 16×16 (altrimenti 8×8)
static constexpr uint8_t R1_SPR_DOUBLE  = 0x01; // sprite ingranditi ×2

// ── Bit di stato VDP ──────────────────────────────
static constexpr uint8_t STATUS_VBLANK    = 0x80;
static constexpr uint8_t STATUS_SPRITE_OV = 0x40; // overflow sprite per linea
static constexpr uint8_t STATUS_SPRITE_COL= 0x20; // collisione sprite

// ── Palette colori SMS → RGB565 ───────────────────
//  Il SMS usa una palette di 64 colori (6 bit: BBGGRR 2bit ciascuno)
//  Vengono usati al massimo 32 entry (16 BG + 16 sprite) dalla CRAM

// Conversione: colore SMS (6bit) → RGB565
// I 6 bit sono: B1B0 G1G0 R1R0
// Ogni coppia viene scalata: 0→0, 1→85, 2→170, 3→255
static inline uint16_t smsColorToRGB565(uint8_t c) {
    uint8_t r = (c & 0x03) * 85;
    uint8_t g = ((c >> 2) & 0x03) * 85;
    uint8_t b = ((c >> 4) & 0x03) * 85;
    // RGB565: RRRRRGGGGGGBBBBB
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// ─────────────────────────────────────────────────
//  Classe VDP
// ─────────────────────────────────────────────────
class VDP {
public:
    // ── Framebuffer condiviso Core0/Core1 ─────────
    // Single buffer RGB565: risparmio 96 KB rispetto al double buffer.
    // Strategia tearing-safe:
    //   Core0 scrive durante le scanline attive (0-191)
    //   Core1 legge durante il VBlank (linee 192-261, ~5.6 ms a 60fps)
    //   Il DMA ha tutto il periodo di blanking per trasferire 96 KB
    //   (a 80 MHz SPI → ~9.8 ms, quindi serve 30fps non 60fps)
    //
    // Sincronizzazione:
    //   frameReady  = Core0 → Core1: "frame completo, puoi inviare"
    //   frameSending = Core1 → Core0: "DMA in corso, non scrivere"
    //   (Core0 non inizia il frame successivo finché frameSending è true)
    static constexpr int FB_SIZE = SMS_SCREEN_W * SMS_SCREEN_H;
    uint16_t framebuffer[FB_SIZE];      // single buffer RGB565 (96 KB)
    volatile bool frameReady;           // Core0 → Core1: frame pronto
    volatile bool frameSending;         // Core1 → Core0: DMA in corso
    // Compat: activeWriteBuf non più usato ma mantenuto per evitare
    // regressioni in codice che lo referenzia
    volatile uint8_t activeWriteBuf;

    // ── Costruttore ───────────────────────────────
    VDP();

    // ── Reset hardware ────────────────────────────
    void reset();

    // ── Accesso I/O dalla CPU (Z80) ───────────────
    // Port 0xBF: scrittura controllo (registri / indirizzi VRAM)
    void writeControl(uint8_t value);
    // Port 0xBE: lettura/scrittura dati VRAM
    void writeData(uint8_t value);
    uint8_t readData();
    // Port 0xBF: lettura status
    uint8_t readStatus();

    // ── Tick: avanza di N cicli di clock Z80 ─────
    // Chiamato dal loop principale su Core0
    // Restituisce true se va generato un interrupt (HBlank o VBlank)
    void renderScanline(int line);
    void tickLineCounter(int line);
    void signalVBlank();

    // Tick legacy: gestisce tutto internamente (cicli Z80 liberi)
    bool tick(int cycles);

    // ── Interrupt ─────────────────────────────────
    bool pendingVBlankIRQ() const { return _vblankIRQ; }
    bool pendingLineIRQ()   const { return _lineIRQ;   }
    void clearVBlankIRQ()         { _vblankIRQ = false; }
    void clearLineIRQ()           { _lineIRQ   = false; }

    // ── Stato linea corrente (per debug/sync) ─────
    int  currentLine()      const { return _currentLine; }
    bool isVBlank()         const { return _currentLine >= VDP_LINES_ACTIVE; }

    // ── Accesso diretto VRAM (per debug) ──────────
    uint8_t  readVRAM(uint16_t addr) const { return _vram[addr & 0x3FFF]; }
    void     writeVRAM(uint16_t addr, uint8_t v) { _vram[addr & 0x3FFF] = v; }

private:
    // ── Memoria ───────────────────────────────────
    uint8_t  _vram[SMS_VRAM_SIZE];      // 16 KB VRAM
    uint8_t  _cram[32];                 // 32 byte CRAM (palette, 6bit per entry)
    uint16_t _cramRGB[32];              // palette precalcolata in RGB565

    // ── Registri interni ──────────────────────────
    uint8_t  _reg[VDP_REG_COUNT];       // R0–R10
    uint8_t  _statusFlags;              // byte di stato
    uint16_t _addrLatch;                // indirizzo VRAM latch a 14 bit
    uint8_t  _addrLatchByte;            // primo o secondo byte del latch (0/1)
    uint8_t  _codeReg;                  // bits 6-7 del secondo byte controllo
    uint8_t  _readBuffer;               // buffer lettura VRAM (pre-fetch)

    // ── CRAM write latch ──────────────────────────
    bool     _cramWrite;                // true: prossima writeData va in CRAM
    uint8_t  _cramAddr;                 // indirizzo CRAM corrente

    // ── Timing ────────────────────────────────────
    int      _cycleAccum;               // cicli Z80 accumulati
    int      _currentLine;             // linea corrente 0–261
    int      _lineCounter;             // contatore linea per interrupt HBlank
    bool     _vblankIRQ;
    bool     _lineIRQ;

    // ── Sprite per la linea corrente ──────────────
    struct SpriteEntry {
        int x, y;
        uint8_t tile;
        uint8_t flags;      // usato in SMS2; in SMS1 non c'è attributo colore sprite
    };
    SpriteEntry _lineSprites[MAX_SPRITES_LINE];
    int         _lineSpriteCnt;

    // ── Render ────────────────────────────────────
    void renderLine(int line);
    void renderBackground(int line, uint16_t* dst);
    void collectSprites(int line);
    void renderSprites(int line, uint16_t* dst);
    void updateCRAM(uint8_t addr, uint8_t value);

    // ── Helper ────────────────────────────────────
    // Indirizzo base Name Table (mappa BG): reg[2] bits 3-1 × 0x800
    inline uint16_t nameTableBase() const {
        return (uint16_t)(_reg[2] & 0x0E) << 10;
    }
    // Indirizzo base Sprite Attribute Table: reg[5] bits 6-1 × 0x80
    inline uint16_t satBase() const {
        return (uint16_t)(_reg[5] & 0x7E) << 7;
    }
    // Indirizzo base tile patterns sprite: reg[6] bit 2 seleziona bank
    inline uint16_t spriteTileBase() const {
        return (_reg[6] & 0x04) ? 0x2000 : 0x0000;
    }
    // Dimensione sprite: 8 o 16 pixel
    inline int spriteSize() const {
        return (_reg[1] & R1_SPR_16) ? 16 : 8;
    }
    // Colore di sfondo (overscan): CRAM entry 16 + (reg[7] & 0x0F)
    inline uint16_t bgColor() const {
        return _cramRGB[16 + (_reg[7] & 0x0F)];
    }
    // Scroll orizzontale (negativo = scorre a destra)
    inline uint8_t scrollX() const { return _reg[8]; }
    // Scroll verticale
    inline uint8_t scrollY() const { return _reg[9]; }
};
