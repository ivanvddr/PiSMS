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

#include "VDP.h"

// ─────────────────────────────────────────────────
//  Cicli Z80 per scanline
// ─────────────────────────────────────────────────
static constexpr int CYCLES_PER_LINE = 228;

static constexpr uint32_t spreadByte(uint8_t b) {
    uint32_t r = 0;
    for (int i = 0; i < 8; i++) {
        if (b & (1u << i)) r |= (1u << (i * 4));
    }
    return r;
}

struct SpreadTable {
    uint32_t v[256];
    constexpr SpreadTable() : v{} {
        for (int i = 0; i < 256; i++) v[i] = spreadByte((uint8_t)i);
    }
};
static constexpr SpreadTable kSpread;

// Estrae il cidx (0-15) del pixel "px" (0=sinistra) da un combined
// già costruito con spreadByte, rispettando hFlip.
// Senza flip: bit=7-px → nibble (7-px). Con flip: bit=px → nibble px.
static inline uint8_t extractCidx(uint32_t combined, int px, bool hFlip) {
    int nibble = hFlip ? px : (7 - px);
    return (uint8_t)((combined >> (nibble * 4)) & 0xF);
}

// ─────────────────────────────────────────────────
//  Costruttore / Reset (invariati)
// ─────────────────────────────────────────────────
VDP::VDP() {
    reset();
}

void VDP::reset() {
    memset(_vram,  0, sizeof(_vram));
    memset(_cram,  0, sizeof(_cram));
    memset(_cramRGB, 0, sizeof(_cramRGB));
    memset(_reg,   0, sizeof(_reg));
    memset(framebuffer, 0, sizeof(framebuffer));
    frameReady   = false;
    frameSending = false;
    activeWriteBuf = 0;

    _statusFlags    = 0;
    _addrLatch      = 0;
    _addrLatchByte  = 0;
    _codeReg        = 0;
    _readBuffer     = 0;
    _cramWrite      = false;
    _cramAddr       = 0;
    _cycleAccum     = 0;
    _currentLine    = 0;
    _lineCounter    = 0;
    _vblankIRQ      = false;
    _lineIRQ        = false;
    _lineSpriteCnt  = 0;
    activeWriteBuf  = 0;
    frameReady      = false;

    _reg[0] = 0x36;
    _reg[1] = 0x80;
    _reg[2] = 0xFF;
    _reg[3] = 0xFF;
    _reg[4] = 0xFF;
    _reg[5] = 0xFF;
    _reg[6] = 0xFF;
    _reg[7] = 0x00;
    _reg[8] = 0x00;
    _reg[9] = 0x00;
    _reg[10]= 0xFF;
}

// ─────────────────────────────────────────────────
//  Scrittura porta controllo (0xBF) — invariata
// ─────────────────────────────────────────────────
void VDP::writeControl(uint8_t value) {
    if (_addrLatchByte == 0) {
        _addrLatch = (_addrLatch & 0xFF00) | value;
        _addrLatchByte = 1;
    } else {
        _addrLatchByte = 0;
        _codeReg  = (value >> 6) & 0x03;
        _addrLatch = ((uint16_t)(value & 0x3F) << 8) | (_addrLatch & 0x00FF);

        switch (_codeReg) {
            case 0x00:
                _readBuffer = _vram[_addrLatch & 0x3FFF];
                _addrLatch  = (_addrLatch + 1) & 0x3FFF;
                _cramWrite  = false;
                break;

            case 0x01:
                _cramWrite = false;
                break;

            case 0x02: {
                uint8_t regNum = value & 0x0F;
                if (regNum < VDP_REG_COUNT) {
                    _reg[regNum] = (uint8_t)(_addrLatch & 0x00FF);
                }
                _cramWrite = false;
                break;
            }

            case 0x03:
                _cramWrite = true;
                _cramAddr  = (uint8_t)(_addrLatch & 0x1F);
                break;
        }
    }
}

void VDP::writeData(uint8_t value) {
    _addrLatchByte = 0;

    if (_cramWrite) {
        updateCRAM(_cramAddr, value);
        _cramAddr = (_cramAddr + 1) & 0x1F;
    } else {
        _vram[_addrLatch & 0x3FFF] = value;
        _addrLatch = (_addrLatch + 1) & 0x3FFF;
    }
}

uint8_t VDP::readData() {
    _addrLatchByte = 0;
    uint8_t ret    = _readBuffer;
    _readBuffer    = _vram[_addrLatch & 0x3FFF];
    _addrLatch     = (_addrLatch + 1) & 0x3FFF;
    return ret;
}

uint8_t VDP::readStatus() {
    _addrLatchByte = 0;
    uint8_t s      = _statusFlags;
    _statusFlags   = 0;
    _vblankIRQ     = false;
    _lineIRQ       = false;
    return s;
}

void VDP::updateCRAM(uint8_t addr, uint8_t value) {
    addr &= 0x1F;
    _cram[addr]    = value;
    _cramRGB[addr] = smsColorToRGB565(value);
}

// ─────────────────────────────────────────────────
//  Tick legacy (invariato)
// ─────────────────────────────────────────────────
bool VDP::tick(int cycles) {
    _cycleAccum += cycles;
    bool irqPending = false;

    while (_cycleAccum >= CYCLES_PER_LINE) {
        _cycleAccum -= CYCLES_PER_LINE;
        int line = _currentLine;

        if (line < VDP_LINES_ACTIVE) {
            if (_reg[1] & R1_SCREEN_ON) {
                renderLine(line);
            } else {
                uint16_t* dst = framebuffer + line * SMS_SCREEN_W;
                uint16_t  col = bgColor();
                for (int i = 0; i < SMS_SCREEN_W; i++) dst[i] = col;
            }
        }

        if (line <= VDP_LINES_ACTIVE) {
            if (_lineCounter == 0) {
                _lineCounter = _reg[10];
                if (_reg[0] & R0_VSYNC_IE) {
                    _lineIRQ   = true;
                    irqPending = true;
                }
            } else {
                _lineCounter--;
            }
        } else {
            _lineCounter = _reg[10];
        }

        if (line == VDP_LINES_ACTIVE) {
            _statusFlags |= STATUS_VBLANK;
            if (_reg[1] & R1_IE) {
                _vblankIRQ = true;
                irqPending = true;
            }
            frameReady = true;
        }

        _currentLine++;
        if (_currentLine >= VDP_LINES_TOTAL) {
            _currentLine = 0;
        }
    }

    return irqPending;
}

void VDP::renderLine(int line) {
    uint16_t* dst = framebuffer + line * SMS_SCREEN_W;
    collectSprites(line);
    renderBackground(line, dst);
    renderSprites(line, dst);
}

void VDP::renderBackground(int line, uint16_t* dst) {
    uint16_t ntBase = nameTableBase();
    int sY = scrollY();
    int scrolledLine = (line + sY) % 224;
    int tileRow = scrolledLine >> 3;
    int pixelY = scrolledLine & 7;
    int sX = scrollX();

    // ── Precompute per colonna (32 tile della riga) ──────────────
    // produciamo direttamente "combined" pronto per
    // l'estrazione O(1) per pixel, invece di un pattern[] da
    // re-interpretare bit-a-bit 8 volte nel loop sotto.
    uint32_t combined[32];
    bool     hFlipCol[32];
    uint8_t  palOffCol[32];

    for (int col = 0; col < 32; col++) {
        uint16_t ntAddr = ntBase + (tileRow * 32 + col) * 2;
        uint8_t lo = _vram[ntAddr & 0x3FFF];
        uint8_t hi = _vram[(ntAddr + 1) & 0x3FFF];
        uint16_t tileNum = lo | ((hi & 1) << 8);
        bool vFlip = hi & 0x04;
        int py = vFlip ? (7 - pixelY) : pixelY;
        uint16_t tileAddr = tileNum * 32 + py * 4;
        uint8_t p0 = _vram[tileAddr & 0x3FFF];
        uint8_t p1 = _vram[(tileAddr + 1) & 0x3FFF];
        uint8_t p2 = _vram[(tileAddr + 2) & 0x3FFF];
        uint8_t p3 = _vram[(tileAddr + 3) & 0x3FFF];

        // 4 lookup invece di 8 estrazioni bit-a-bit ripetute per pixel
        combined[col]  = kSpread.v[p0] | (kSpread.v[p1] << 1)
                        | (kSpread.v[p2] << 2) | (kSpread.v[p3] << 3);
        hFlipCol[col]  = hi & 0x02;
        palOffCol[col] = (hi & 0x08) ? 16 : 0;
    }

    bool statusBar = (_reg[0] & 0x40);

    // Status bar: primi 8 pixel senza scroll 
    if (statusBar) {
        for (int screenX = 0; screenX < 8; screenX++) {
            int col = screenX >> 3;
            int px  = screenX & 7;
            uint8_t cidx = extractCidx(combined[col], px, hFlipCol[col]);
            dst[screenX] = _cramRGB[cidx + palOffCol[col]];
        }
    }

    // Resto dei pixel con scroll orizzontale
    for (int screenX = 8; screenX < SMS_SCREEN_W; screenX++) {
        int sourceX = (screenX - sX + 256) & 0xFF;
        int col = sourceX >> 3;
        int px  = sourceX & 7;
        uint8_t cidx = extractCidx(combined[col], px, hFlipCol[col]);
        dst[screenX] = _cramRGB[cidx + palOffCol[col]];
    }
}

// ─────────────────────────────────────────────────
//  Raccolta sprite
// ─────────────────────────────────────────────────
void VDP::collectSprites(int line) {
    _lineSpriteCnt = 0;
    uint16_t satY  = satBase();
    uint16_t satX  = satY + 128;
    int      sz    = spriteSize();

    for (int i = 0; i < MAX_SPRITES_FRAME; i++) {
        uint8_t y = _vram[(satY + i) & 0x3FFF];
        if (y == 0xD0) break;

        int sprY = (int)y + 1;

        if (line >= sprY && line < sprY + sz) {
            if (_lineSpriteCnt >= MAX_SPRITES_LINE) {
                _statusFlags |= STATUS_SPRITE_OV;
                break;
            }
            SpriteEntry& e = _lineSprites[_lineSpriteCnt++];
            e.y    = sprY;
            e.x    = _vram[(satX + i * 2)     & 0x3FFF];
            e.tile = _vram[(satX + i * 2 + 1) & 0x3FFF];
            e.flags = 0;
        }
    }
}

// ─────────────────────────────────────────────────
//  Render sprite 
//  p0..p3 letti una volta per sprite, l'estrazione
//  cidx per gli 8 pixel dello sprite usa kSpread invece di 4
//  shift+4 and+3 or per pixel. Lo sprite non ha hFlip (il SMS1 non
//  lo supporta per sprite), quindi l'ordine è sempre bit=7-px.
// ─────────────────────────────────────────────────
void VDP::renderSprites(int line, uint16_t* dst) {
    if (_lineSpriteCnt == 0) return;

    bool xShift = (_reg[0] & 0x08) != 0;
    int sz = spriteSize();
    uint16_t tileBase = spriteTileBase();

    for (int s = _lineSpriteCnt - 1; s >= 0; s--) {
        const SpriteEntry& e = _lineSprites[s];
        int pyInTile = line - e.y;
        int sprX = e.x - (xShift ? 8 : 0);

        uint8_t tileNum = (sz == 16) ? (e.tile & 0xFE) : e.tile;
        uint8_t actualTile = tileNum + (pyInTile >= 8 ? 1 : 0);
        int py = pyInTile & 7;

        uint16_t tileAddr = tileBase + actualTile * 32 + py * 4;
        uint8_t p0 = _vram[(tileAddr + 0) & 0x3FFF];
        uint8_t p1 = _vram[(tileAddr + 1) & 0x3FFF];
        uint8_t p2 = _vram[(tileAddr + 2) & 0x3FFF];
        uint8_t p3 = _vram[(tileAddr + 3) & 0x3FFF];

        // Un solo combined per l'intero sprite (8 pixel)
        uint32_t combined = kSpread.v[p0] | (kSpread.v[p1] << 1)
                           | (kSpread.v[p2] << 2) | (kSpread.v[p3] << 3);

        for (int px = 0; px < 8; px++) {
            int screenX = sprX + px;
            if (screenX < 0 || screenX >= SMS_SCREEN_W) continue;

            uint8_t cidx = extractCidx(combined, px, false);
            if (cidx == 0) continue;
            dst[screenX] = _cramRGB[16 + cidx];
        }
    }
}

// ─────────────────────────────────────────────────
//  API per-scanline
// ─────────────────────────────────────────────────
void VDP::renderScanline(int line) {
    if (line < 0 || line >= VDP_LINES_ACTIVE) return;
    _currentLine = line;

    if (_reg[1] & R1_SCREEN_ON) {
        renderLine(line);
    } else {
        uint16_t* dst = framebuffer + line * SMS_SCREEN_W;
        uint16_t  col = bgColor();
        for (int i = 0; i < SMS_SCREEN_W; i++) dst[i] = col;
    }
}

void VDP::tickLineCounter(int line) {
    if (line <= VDP_LINES_ACTIVE) {
        if (_lineCounter == 0) {
            _lineCounter = _reg[10];
            if (_reg[0] & R0_VSYNC_IE) {
                _lineIRQ = true;
            }
        } else {
            _lineCounter--;
        }
    }
}

void VDP::signalVBlank() {
    _currentLine  = VDP_LINES_ACTIVE;
    _statusFlags |= STATUS_VBLANK;
    _lineCounter  = _reg[10];

    if (_reg[1] & R1_IE) {
        _vblankIRQ = true;
    }

    frameReady = true;
}