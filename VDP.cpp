/*
 * Questo file fa parte del progetto PiSMS.
 *
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 */

#include "VDP.h"
#include "hardware/sync.h"

// Tabella "spread" in RAM (non constexpr/flash): Core1 che legge dalla
// flash XIP sporcherebbe la cache condivisa e rallenterebbe Core0.
static uint32_t sSpread[256];

static inline uint16_t outPx(uint16_t v) {
#if VDP_PRESWAP_565
    return (uint16_t)((v << 8) | (v >> 8));
#else
    return v;
#endif
}

// ─────────────────────────────────────────────────
VDP::VDP() {
    for (int i = 0; i < 256; i++) {
        uint32_t r = 0;
        for (int b = 0; b < 8; b++)
            if (i & (1 << b)) r |= (1u << (b * 4));
        sSpread[i] = r;
    }
    reset();
}

void VDP::reset() {
    memset(_vram,    0, sizeof(_vram));
    memset(_cram,    0, sizeof(_cram));
    memset(_cramRGB, 0, sizeof(_cramRGB));
    memset(_reg,     0, sizeof(_reg));

    _statusFlags   = 0;
    _addrLatch     = 0;
    _addrLatchByte = 0;
    _codeReg       = 0;
    _readBuffer    = 0;
    _cramWrite     = false;
    _cramAddr      = 0;
    _currentLine   = 0;
    _lineCounter   = 0;
    _vblankIRQ     = false;
    _lineIRQ       = false;

    _snapState     = SNAP_FREE;
    _snap.palCount = 0;
    _cramDirty     = true;
    _curPal        = 0;

    _reg[0] = 0x36; _reg[1] = 0x80; _reg[2] = 0xFF; _reg[3] = 0xFF;
    _reg[4] = 0xFF; _reg[5] = 0xFF; _reg[6] = 0xFF; _reg[7] = 0x00;
    _reg[8] = 0x00; _reg[9] = 0x00; _reg[10] = 0xFF;
}

// ─────────────────────────────────────────────────
//  Porte I/O (Core0, hot path > RAM)
// ─────────────────────────────────────────────────
VDP_RAMFUNC void VDP::writeControl(uint8_t value) {
    if (_addrLatchByte == 0) {
        _addrLatch = (_addrLatch & 0xFF00) | value;
        _addrLatchByte = 1;
    } else {
        _addrLatchByte = 0;
        _codeReg   = (value >> 6) & 0x03;
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
                if (regNum < VDP_REG_COUNT)
                    _reg[regNum] = (uint8_t)(_addrLatch & 0x00FF);
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

VDP_RAMFUNC void VDP::updateCRAM(uint8_t addr, uint8_t value) {
    addr &= 0x1F;
    if (_cram[addr] == value) return;
    _cram[addr]    = value;
    _cramRGB[addr] = smsColorToRGB565(value);
    _cramDirty     = true;
}

VDP_RAMFUNC void VDP::writeData(uint8_t value) {
    _addrLatchByte = 0;
    if (_cramWrite) {
        updateCRAM(_cramAddr, value);
        _cramAddr = (_cramAddr + 1) & 0x1F;
    } else {
        _vram[_addrLatch & 0x3FFF] = value;
        _addrLatch = (_addrLatch + 1) & 0x3FFF;
    }
}

VDP_RAMFUNC uint8_t VDP::readData() {
    _addrLatchByte = 0;
    uint8_t ret  = _readBuffer;
    _readBuffer  = _vram[_addrLatch & 0x3FFF];
    _addrLatch   = (_addrLatch + 1) & 0x3FFF;
    return ret;
}

VDP_RAMFUNC uint8_t VDP::readStatus() {
    _addrLatchByte = 0;
    uint8_t s    = _statusFlags;
    _statusFlags = 0;
    _vblankIRQ   = false;
    _lineIRQ     = false;
    return s;
}

// ─────────────────────────────────────────────────
//  Timing (Core0)
// ─────────────────────────────────────────────────
VDP_RAMFUNC void VDP::tickLineCounter(int line) {
    if (line <= VDP_LINES_ACTIVE) {
        if (_lineCounter == 0) {
            _lineCounter = _reg[10];
            if (_reg[0] & R0_VSYNC_IE) _lineIRQ = true;
        } else {
            _lineCounter--;
        }
    }
}

VDP_RAMFUNC void VDP::signalVBlank() {
    _currentLine  = VDP_LINES_ACTIVE;
    _statusFlags |= STATUS_VBLANK;
    _lineCounter  = _reg[10];
    if (_reg[1] & R1_IE) _vblankIRQ = true;
}

// ─────────────────────────────────────────────────
//  Cattura frame (Core0)
// ─────────────────────────────────────────────────
VDP_RAMFUNC bool VDP::beginCapture() {
    if (_snapState != SNAP_FREE) return false;
    memcpy(_snap.vram, _vram, SMS_VRAM_SIZE);
    _snap.palCount = 0;
    _curPal        = 0;
    _cramDirty     = true;          // forza lo snapshot palette alla linea 0
    return true;
}

VDP_RAMFUNC void VDP::captureLine(int line) {
    LineState& L = _snap.line[line];
    L.scrollX = _reg[8];
    L.scrollY = _reg[9];
    L.r0 = _reg[0]; L.r1 = _reg[1]; L.r2 = _reg[2];
    L.r5 = _reg[5]; L.r6 = _reg[6]; L.r7 = _reg[7];

    // Palette: nuovo snapshot (64 byte) solo se la CRAM è cambiata
    if (_cramDirty && _snap.palCount < PAL_POOL) {
        memcpy(_snap.pal[_snap.palCount], _cramRGB, sizeof(_cramRGB));
        _curPal    = _snap.palCount++;
        _cramDirty = false;
    }
    L.pal = _curPal;
}

VDP_RAMFUNC void VDP::endCapture() {
    __dmb();
    _snapState = SNAP_READY;
}

// ─────────────────────────────────────────────────
//  Handshake Core1
// ─────────────────────────────────────────────────
VDP_RAMFUNC bool VDP::acquireSnapshot() {
    if (_snapState != SNAP_READY) return false;
    __dmb();
    _snapState = SNAP_BUSY;
    return true;
}

VDP_RAMFUNC void VDP::releaseSnapshot() {
    __dmb();
    _snapState = SNAP_FREE;
}

// ─────────────────────────────────────────────────
//  Rendering (Core1, tutto in RAM per non toccare la cache XIP di Core0)
// ─────────────────────────────────────────────────
VDP_RAMFUNC void VDP::renderChunk(int firstLine, int count, uint16_t* out) {
    uint16_t palSw[32];
    int curPal = -1;

    for (int i = 0; i < count; i++) {
        int line = firstLine + i;
        const LineState& L = _snap.line[line];
        uint16_t* dst = out + i * SMS_SCREEN_W;

        if ((int)L.pal != curPal) {
            curPal = L.pal;
            const uint16_t* src = _snap.pal[curPal];
            for (int k = 0; k < 32; k++) palSw[k] = outPx(src[k]);
        }

        if (!(L.r1 & R1_SCREEN_ON)) {
            uint16_t c = palSw[16 + (L.r7 & 0x0F)];
            uint32_t c2 = c | ((uint32_t)c << 16);
            uint32_t* d32 = (uint32_t*)dst;
            for (int x = 0; x < SMS_SCREEN_W / 2; x++) d32[x] = c2;
            continue;
        }

        renderBG(L, line, dst, palSw);
        renderSprites(L, line, dst, palSw);
    }
}

VDP_RAMFUNC void VDP::renderBG(const LineState& L, int line,
                               uint16_t* dst, const uint16_t* pal) {
    const uint8_t* vram = _snap.vram;

    uint16_t ntBase = (uint16_t)(L.r2 & 0x0E) << 10;
    int scrolled = (line + L.scrollY) % 224;
    int tileRow  = scrolled >> 3;
    int pixelY   = scrolled & 7;

    // R0 bit6: scroll orizzontale disattivato per le prime 2 righe di tile
    int sX = L.scrollX;
    if ((L.r0 & 0x40) && line < 16) sX = 0;

    uint32_t comb[32];
    uint8_t  attr[32];      // bit0 = hFlip, bit4 = offset palette sprite (16)

    for (int col = 0; col < 32; col++) {
        uint16_t ntAddr = ntBase + ((tileRow * 32 + col) << 1);
        uint8_t lo = vram[ntAddr & 0x3FFF];
        uint8_t hi = vram[(ntAddr + 1) & 0x3FFF];
        uint16_t tileNum = lo | ((hi & 1) << 8);
        int py = (hi & 0x04) ? (7 - pixelY) : pixelY;
        uint16_t ta = tileNum * 32 + py * 4;

        comb[col] = sSpread[vram[ta & 0x3FFF]]
                  | (sSpread[vram[(ta + 1) & 0x3FFF]] << 1)
                  | (sSpread[vram[(ta + 2) & 0x3FFF]] << 2)
                  | (sSpread[vram[(ta + 3) & 0x3FFF]] << 3);
        attr[col] = ((hi & 0x02) ? 1 : 0) | ((hi & 0x08) ? 16 : 0);
    }

    // Per tile (non per pixel): una lookup per colonna, poi 8 pixel
    int srcX = (256 - sX) & 0xFF;
    int col  = srcX >> 3;
    int px0  = srcX & 7;
    int x    = 0;
    while (x < SMS_SCREEN_W) {
        uint32_t c = comb[col];
        uint8_t  a = attr[col];
        const uint16_t* p = pal + (a & 16);
        if (a & 1) {
            for (int px = px0; px < 8 && x < SMS_SCREEN_W; px++, x++)
                dst[x] = p[(c >> (px << 2)) & 0xF];
        } else {
            for (int px = px0; px < 8 && x < SMS_SCREEN_W; px++, x++)
                dst[x] = p[(c >> ((7 - px) << 2)) & 0xF];
        }
        px0 = 0;
        col = (col + 1) & 31;
    }
}

VDP_RAMFUNC void VDP::renderSprites(const LineState& L, int line,
                                    uint16_t* dst, const uint16_t* pal) {
    const uint8_t* vram = _snap.vram;

    uint16_t satY = (uint16_t)(L.r5 & 0x7E) << 7;
    uint16_t satX = satY + 128;
    int      sz   = (L.r1 & R1_SPR_16) ? 16 : 8;
    uint16_t tileBase = (L.r6 & 0x04) ? 0x2000 : 0x0000;
    int      xShift   = (L.r0 & 0x08) ? 8 : 0;

    int16_t sx[MAX_SPRITES_LINE];
    int16_t sy[MAX_SPRITES_LINE];
    uint8_t st[MAX_SPRITES_LINE];
    int n = 0;

    for (int i = 0; i < MAX_SPRITES_FRAME; i++) {
        uint8_t y = vram[(satY + i) & 0x3FFF];
        if (y == 0xD0) break;
        int sprY = (int)y + 1;
        if (line >= sprY && line < sprY + sz) {
            if (n >= MAX_SPRITES_LINE) break;
            sy[n] = sprY;
            sx[n] = vram[(satX + i * 2) & 0x3FFF];
            st[n] = vram[(satX + i * 2 + 1) & 0x3FFF];
            n++;
        }
    }
    if (n == 0) return;

    // dal meno al più prioritario (indice 0 = priorità massima, disegnato per ultimo)
    for (int s = n - 1; s >= 0; s--) {
        int pyInTile = line - sy[s];
        uint8_t tileNum    = (sz == 16) ? (st[s] & 0xFE) : st[s];
        uint8_t actualTile = tileNum + (pyInTile >= 8 ? 1 : 0);
        int py = pyInTile & 7;

        uint16_t ta = tileBase + actualTile * 32 + py * 4;
        uint32_t comb = sSpread[vram[ta & 0x3FFF]]
                      | (sSpread[vram[(ta + 1) & 0x3FFF]] << 1)
                      | (sSpread[vram[(ta + 2) & 0x3FFF]] << 2)
                      | (sSpread[vram[(ta + 3) & 0x3FFF]] << 3);

        int sprX = sx[s] - xShift;
        int p0 = (sprX < 0) ? -sprX : 0;
        int p1 = (sprX + 8 > SMS_SCREEN_W) ? (SMS_SCREEN_W - sprX) : 8;

        for (int px = p0; px < p1; px++) {
            uint8_t cidx = (comb >> ((7 - px) << 2)) & 0xF;
            if (cidx) dst[sprX + px] = pal[16 + cidx];
        }
    }
}