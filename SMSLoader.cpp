/*
 * SMSLoader.cpp — Loader ROM per SMS Emulator (RP2040)
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 * Basato su MSXLoader del progetto PiMSX.
 */
#include "SMSLoader.h"

// ─────────────────────────────────────────────────
SMSLoader::SMSLoader()
    : _stamp(0)
    , _cartRam(nullptr), _cartRamAllocated(false)
    , _regFFFC(0), _cartRamEnabled(false), _cartRamMapped(false), _cartRamBank(0)
    , _loaded(false), _romSize(0), _numBanks(0)
    , _mapper(SMSMapper::NONE), _hasHeader(false)
    , _currentFS(FSType::FS_NONE), _fsReady(false)
    , _sdCsPin(5), _fileOpen(false)
{
    memset(_pool,       0xFF, sizeof(_pool));
    memset(_poolBank,   0xFF, sizeof(_poolBank));
    memset(_poolStamp,  0,    sizeof(_poolStamp));
    memset(_bank0Fixed, 0xFF, sizeof(_bank0Fixed));
    memset(_ram,        0x00, sizeof(_ram));
    memset(_sink,       0x00, sizeof(_sink));
    memset(_bankReg,    0,    sizeof(_bankReg));
    _slotPool[0] = 0; _slotPool[1] = 1; _slotPool[2] = 2;
    _romName[0] = '\0';
    _romPath[0] = '\0';
    rebuildPages();
}

// ─────────────────────────────────────────────────
//  Tabelle di pagina
// ─────────────────────────────────────────────────
LOADER_RAMFUNC void SMSLoader::mapSlot(uint8_t s) {
    const uint8_t* base = _pool[_slotPool[s]];
    for (int i = 0; i < 16; i++) {
        _rd[s * 16 + i] = base + i * 1024;
        _wr[s * 16 + i] = _sink;                       // scritture su ROM ignorate
    }
    if (s == 0) _rd[0] = _bank0Fixed;                  // primo KB sempre dal bank 0
    if (_mapper == SMSMapper::CODEMASTERS) _wr[s * 16] = nullptr;  // $0000/$4000/$8000

    if (s == 2 && _cartRamMapped && _cartRam) {
        for (int i = 0; i < 16; i++) {
            uint8_t* p = _cartRam + (uint32_t)_cartRamBank * SMS_BANK_SIZE + i * 1024;
            _rd[32 + i] = p;
            _wr[32 + i] = p;
        }
    }
}

void SMSLoader::mapRam() {
    for (int p = 48; p < 64; p++) {
        uint8_t* r = _ram + ((p - 48) & 7) * 1024;    // $E000-$FFFF = mirror
        _rd[p] = r;
        _wr[p] = r;
    }
    _wr[63] = nullptr;    // $FC00-$FFFF: registri mapper > percorso lento
}

void SMSLoader::rebuildPages() {
    mapRam();
    for (uint8_t s = 0; s < SMS_SLOTS; s++) mapSlot(s);
}

// ─────────────────────────────────────────────────
//  Scritture "lente" (registri mapper & co.)
// ─────────────────────────────────────────────────
LOADER_RAMFUNC void SMSLoader::writeSlow(uint16_t addr, uint8_t value) {
    if (addr < 0x8000) {
        if (_mapper == SMSMapper::CODEMASTERS) writeCodemastersMapper(addr, value);
        return;
    }
    if (addr < 0xC000) {
        // (solo se la pagina non è RAM cartuccia: quella ha scrittura diretta)
        if (_mapper == SMSMapper::CODEMASTERS) writeCodemastersMapper(addr, value);
        return;
    }
    _ram[addr & 0x1FFF] = value;
    if (addr >= 0xFFFC && (_mapper == SMSMapper::SEGA || _mapper == SMSMapper::NONE)) {
        writeSegaMapper(addr, value);
    }
}

// ─────────────────────────────────────────────────
//  Filesystem
// ─────────────────────────────────────────────────
bool SMSLoader::initFilesystem(FSType preferred, int sdCsPin) {
    if (_fsReady) return true;
    _sdCsPin = sdCsPin;

    auto trySD = [&]() -> bool {
        Serial.println("[Loader] SD...");
        if (initSD()) {
            _currentFS = FSType::FS_SD;
            _fsReady   = true;
            Serial.println("[Loader] SD OK");
            return true;
        }
        Serial.println("[Loader] SD not available");
        return false;
    };
    auto tryLFS = [&]() -> bool {
        Serial.println("[Loader] LittleFS...");
        if (LittleFS.begin()) {
            _currentFS = FSType::FS_LITTLEFS;
            _fsReady   = true;
            Serial.println("[Loader] LittleFS OK");
            return true;
        }
        Serial.println("[Loader] LittleFS not available");
        return false;
    };

    if (preferred == FSType::FS_SD) {
        if (trySD())  return true;
        if (tryLFS()) return true;
    } else {
        if (tryLFS()) return true;
        if (trySD())  return true;
    }
    Serial.println("[Loader] ERROR: no filesystem found");
    return false;
}

void SMSLoader::closeFilesystem() {
    closeROMFile();
    if (_currentFS == FSType::FS_SD) {
        SD.end();
        Serial.println("[Loader] SD closed");
    }
    _fsReady   = false;
    _currentFS = FSType::FS_NONE;
}

bool SMSLoader::initSD() {
    SPI1.setRX(SD_MISO);
    SPI1.setTX(SD_MOSI);
    SPI1.setSCK(SD_SCLK);
    SPI1.setCS(SD_CS);
    SPI1.begin();
    delay(100);
    return SD.begin(SD_CS, 10000000, SPI1);
}

// ─────────────────────────────────────────────────
//  loadROM
// ─────────────────────────────────────────────────
bool SMSLoader::loadROM(const char* path) {
    if (!path || path[0] == '\0') return false;

    if (!_fsReady) {
        if (!initFilesystem()) {
            Serial.println("[Loader] ERROR: filesystem not ready");
            return false;
        }
    }
    if (_loaded) unloadROM();

    strncpy(_romPath, path, sizeof(_romPath) - 1);
    _romPath[sizeof(_romPath) - 1] = '\0';
    extractName(path);

    Serial.printf("\n[Loader] Loading: %s\n", path);

    if (!openROMFile()) return false;

    uint32_t fileSize = _romFile.size();
    Serial.printf("[Loader] File size: %lu byte\n", fileSize);

    if (fileSize < 16) {
        Serial.println("[Loader] ERROR: file too small");
        closeROMFile(); return false;
    }

    _hasHeader = detectHeader();
    uint32_t headerOffset = _hasHeader ? 512 : 0;
    _romSize = fileSize - headerOffset;

    if (_romSize == 0 || _romSize > (uint32_t)SMS_MAX_BANKS * SMS_BANK_SIZE) {
        Serial.printf("[Loader] ERROR: ROM size invalid (%lu byte)\n", _romSize);
        closeROMFile(); return false;
    }

    _numBanks = (uint8_t)((_romSize + SMS_BANK_SIZE - 1) / SMS_BANK_SIZE);
    _mapper   = detectMapper();

    Serial.printf("[Loader] ROM: %-32s  %lu KB  %d bank\n",
                  _romName, _romSize / 1024, _numBanks);
    Serial.printf("[Loader] Mapper: %s  Header: %s\n",
                  _mapper == SMSMapper::NONE        ? "NONE"        :
                  _mapper == SMSMapper::SEGA        ? "SEGA"        :
                  _mapper == SMSMapper::CODEMASTERS ? "CODEMASTERS" : "KOREAN",
                  _hasHeader ? "yes (512B skipped)" : "no");

    // ── Pool vuoto ───────────────────────────────
    memset(_poolBank,  0xFF, sizeof(_poolBank));
    memset(_poolStamp, 0,    sizeof(_poolStamp));
    _stamp = 0;
    _slotPool[0] = 0; _slotPool[1] = 1; _slotPool[2] = 2;

    uint8_t init[SMS_SLOTS];
    init[0] = 0;
    init[1] = (_numBanks > 1) ? 1 : 0;
    init[2] = (_numBanks > 2) ? 2 : 0;

    _cartRamEnabled = false;
    _cartRamMapped  = false;
    _cartRamBank    = 0;
    _regFFFC        = 0;

    if (_numBanks <= SMS_POOL_BANKS) {
        // ROM interamente in RAM: nessun accesso SD durante il gioco
        for (uint8_t b = 0; b < _numBanks; b++) {
            if (!readBank(b, _pool[b])) {
                Serial.printf("[Loader] ERROR reading bank %d\n", b);
                closeROMFile(); return false;
            }
            _poolBank[b] = b;
            _poolStamp[b] = ++_stamp;
        }
        for (uint8_t s = 0; s < SMS_SLOTS; s++) {
            _slotPool[s] = init[s];
            _bankReg[s]  = init[s];
        }
        closeROMFile();
        Serial.println("[Loader] ROM all in RAM: file closed");
    } else {
        for (uint8_t s = 0; s < SMS_SLOTS; s++) {
            if (!loadBank(s, init[s])) {
                Serial.printf("[Loader] ERROR bank init slot %d\n", s);
                closeROMFile(); return false;
            }
        }
        Serial.println("[Loader] ROM big: cache LRU, file open");
    }

    memcpy(_bank0Fixed, _pool[_slotPool[0]], SMS_FIXED_KB);
    memset(_ram, 0x00, sizeof(_ram));
    _loaded = true;
    rebuildPages();

    Serial.println("[Loader] ROM pronta!\n");
    return true;
}

void SMSLoader::unloadROM() {
    closeROMFile();
    memset(_poolBank,   0xFF, sizeof(_poolBank));
    memset(_poolStamp,  0,    sizeof(_poolStamp));
    memset(_bank0Fixed, 0xFF, sizeof(_bank0Fixed));
    memset(_ram,        0x00, sizeof(_ram));
    memset(_bankReg,    0,    sizeof(_bankReg));
    _slotPool[0] = 0; _slotPool[1] = 1; _slotPool[2] = 2;
    _stamp = 0;

    _cartRamEnabled = false;
    _cartRamMapped  = false;
    _cartRamBank    = 0;
    _regFFFC        = 0;
    rebuildPages();                       // prima di liberare la cart RAM

    if (_cartRamAllocated && _cartRam) {
        free(_cartRam);
        _cartRam = nullptr;
        _cartRamAllocated = false;
        Serial.println("[Loader] CartRAM free");
    }
    _loaded    = false;
    _romSize   = 0;
    _numBanks  = 0;
    _mapper    = SMSMapper::NONE;
    _hasHeader = false;
    _romName[0] = '\0';
    _romPath[0] = '\0';
}

// ─────────────────────────────────────────────────
//  Bank: lettura da file e gestione pool LRU
// ─────────────────────────────────────────────────
bool SMSLoader::readBank(uint8_t bank, uint8_t* dst) {
    uint32_t hdr    = _hasHeader ? 512 : 0;
    uint32_t offset = (uint32_t)bank * SMS_BANK_SIZE;
    uint32_t len    = SMS_BANK_SIZE;
    if (offset + len > _romSize) len = _romSize - offset;

    if (!readFromFile(offset + hdr, dst, len)) {
        memset(dst, 0xFF, SMS_BANK_SIZE);
        return false;
    }
    if (len < SMS_BANK_SIZE) memset(dst + len, 0xFF, SMS_BANK_SIZE - len);
    return true;
}

LOADER_RAMFUNC bool SMSLoader::loadBank(uint8_t slot, uint8_t bank) {
    if (slot >= SMS_SLOTS) return false;
    if (_numBanks > 0) bank = bank % _numBanks;

    // Già mappato su questo slot: caso più frequente, costo ca 0
    if (_poolBank[_slotPool[slot]] == bank) {
        _bankReg[slot] = bank;
        return true;
    }

    // Già nel pool?
    int idx = -1;
    for (int i = 0; i < SMS_POOL_BANKS; i++) {
        if (_poolBank[i] == bank) { idx = i; break; }
    }

    if (idx < 0) {
        // Vittima: buffer vuoto, altrimenti il meno recente non usato da altri slot
        uint32_t best = 0xFFFFFFFFUL;
        for (int i = 0; i < SMS_POOL_BANKS; i++) {
            bool used = false;
            for (uint8_t s2 = 0; s2 < SMS_SLOTS; s2++)
                if (s2 != slot && _slotPool[s2] == i) used = true;
            if (used) continue;
            if (_poolBank[i] == 0xFF) { idx = i; break; }
            if (_poolStamp[i] < best) { best = _poolStamp[i]; idx = i; }
        }
        if (idx < 0) return false;

        if (!readBank(bank, _pool[idx])) {
            Serial.printf("[Loader] ERROR bank reading %d\n", bank);
            _poolBank[idx] = 0xFF;
            return false;
        }
        _poolBank[idx] = bank;
    }

    _poolStamp[idx] = ++_stamp;
    _slotPool[slot] = idx;
    _bankReg[slot]  = bank;
    mapSlot(slot);
    return true;
}

bool SMSLoader::readFromFile(uint32_t offset, uint8_t* dst, uint32_t len) {
    if (!_fileOpen) {
        if (!openROMFile()) return false;
    }
    if (!_romFile.seek(offset)) {
        Serial.printf("[Loader] ERROR seek offset %lu\n", offset);
        return false;
    }
    uint32_t r = _romFile.read(dst, len);
    return (r == len);
}

// ─────────────────────────────────────────────────
//  Mapper SEGA ($FFFC-$FFFF)
// ─────────────────────────────────────────────────
LOADER_RAMFUNC void SMSLoader::writeSegaMapper(uint16_t addr, uint8_t value) {
    switch (addr) {
        case 0xFFFC:
            _regFFFC        = value;
            _cartRamEnabled = (value & 0x08) != 0;
            _cartRamBank    = (value & 0x02) ? 1 : 0;

            if (_cartRamEnabled && !_cartRamAllocated) {
                _cartRam = (uint8_t*)malloc(SMS_CART_RAM_SIZE);
                if (_cartRam) {
                    memset(_cartRam, 0xFF, SMS_CART_RAM_SIZE);
                    _cartRamAllocated = true;
                    Serial.printf("[Loader] CartRAM allocated: %lu KB\n",
                                  (uint32_t)SMS_CART_RAM_SIZE / 1024);
                } else {
                    Serial.println("[Loader] WARN: CartRAM malloc failed");
                    _cartRamEnabled = false;
                }
            }
            _cartRamMapped = _cartRamEnabled && (value & 0x04) != 0;
            mapSlot(2);                    // ricalcola le pagine $8000-$BFFF
            break;

        case 0xFFFD: loadBank(0, value); break;   // il primo KB resta fisso
        case 0xFFFE: loadBank(1, value); break;
        case 0xFFFF:
            if (!_cartRamMapped) loadBank(2, value);
            break;
        default: break;
    }
}

LOADER_RAMFUNC void SMSLoader::writeCodemastersMapper(uint16_t addr, uint8_t value) {
    if      (addr == 0x0000) loadBank(0, value);
    else if (addr == 0x4000) loadBank(1, value);
    else if (addr == 0x8000) loadBank(2, value);
}

// ─────────────────────────────────────────────────
bool SMSLoader::detectHeader() {
    if (!_fileOpen) return false;
    uint32_t fileSize = _romFile.size();
    bool has = ((fileSize % SMS_BANK_SIZE) == 512);
    if (has) Serial.println("[Loader] Header .sms found (512B)");
    return has;
}

SMSMapper SMSLoader::detectMapper() {
    if (_romSize <= SMS_CACHE_SIZE) return SMSMapper::NONE;

    uint32_t hdrOff = _hasHeader ? 512 : 0;

    uint8_t buf[16] = {0};
    if (_romSize >= 0x8000) readFromFile(hdrOff + 0x7FF0, buf, 16);

    const uint8_t tmrSega[] = {'T','M','R',' ','S','E','G','A'};
    bool hasTMR = (memcmp(buf, tmrSega, 8) == 0);

    uint8_t cmBuf[8] = {0};
    if (_romSize >= 0x8000) readFromFile(hdrOff + 0x7FE0, cmBuf, 8);

    bool isCodemasters = !hasTMR &&
                         ((uint8_t)(cmBuf[3] + cmBuf[7]) == 0) &&
                         (cmBuf[3] != 0);

    return isCodemasters ? SMSMapper::CODEMASTERS : SMSMapper::SEGA;
}

// ─────────────────────────────────────────────────
//  Helpers filesystem
// ─────────────────────────────────────────────────
bool SMSLoader::openROMFile() {
    if (_fileOpen) return true;
    if (!_fsReady) return false;

    if (_currentFS == FSType::FS_SD) _romFile = SD.open(_romPath, FILE_READ);
    else                             _romFile = LittleFS.open(_romPath, "r");

    _fileOpen = (bool)_romFile;
    if (!_fileOpen) Serial.printf("[Loader] ERRORE: impossible to open %s\n", _romPath);
    return _fileOpen;
}

void SMSLoader::closeROMFile() {
    if (_fileOpen) {
        _romFile.close();
        _fileOpen = false;
    }
}

File SMSLoader::openFile(const char* path, const char* mode) {
    if (_currentFS == FSType::FS_SD)       return SD.open(path, mode);
    if (_currentFS == FSType::FS_LITTLEFS) return LittleFS.open(path, mode);
    return File();
}

void SMSLoader::extractName(const char* path) {
    if (!path) { _romName[0] = '\0'; return; }
    const char* slash = strrchr(path, '/');
    if (!slash) slash = strrchr(path, '\\');
    const char* base = slash ? slash + 1 : path;
    strncpy(_romName, base, sizeof(_romName) - 1);
    _romName[sizeof(_romName) - 1] = '\0';
    char* dot = strrchr(_romName, '.');
    if (dot) *dot = '\0';
}

void SMSLoader::setSelectedROM(const char* path) {
    if (!path) return;
    strncpy(_romPath, path, sizeof(_romPath) - 1);
    _romPath[sizeof(_romPath) - 1] = '\0';
    Serial.printf("SMSLoader: ROM selected > %s\n", _romPath);
}

void SMSLoader::debugStatus() const {
    if (!_loaded) { Serial.println("[Loader] No ROM loaded"); return; }

    Serial.println("\n=== SMSLoader Status ===");
    Serial.printf("ROM     : %s  (%lu KB)\n", _romName, _romSize / 1024);
    Serial.printf("Mapper  : %s   Bank: %d\n",
                  _mapper == SMSMapper::NONE        ? "NONE"        :
                  _mapper == SMSMapper::SEGA        ? "SEGA"        :
                  _mapper == SMSMapper::CODEMASTERS ? "CODEMASTERS" : "KOREAN",
                  _numBanks);
    Serial.printf("Header  : %s\n", _hasHeader ? "sì" : "no");
    Serial.printf("File    : %s\n", _fileOpen ? "open (cache LRU active)" : "close (ROM all in RAM)");
    Serial.printf("CartRAM : enabled=%d mapped=%d bank=%d\n",
                  _cartRamEnabled, _cartRamMapped, _cartRamBank);
    for (int s = 0; s < SMS_SLOTS; s++) {
        Serial.printf("  Slot %d ($%04X-$%04X) → bank %d (pool %d)\n",
                      s, s * 0x4000, s * 0x4000 + 0x3FFF, _bankReg[s], _slotPool[s]);
    }
    Serial.println("========================\n");
}

bool SMSLoader::fileExists(const char* path) {
    if (_currentFS == FSType::FS_SD)       return SD.exists(path);
    if (_currentFS == FSType::FS_LITTLEFS) return LittleFS.exists(path);
    return false;
}

void SMSLoader::listROMs(const char* folder) {
    if (!_fsReady) { Serial.println("[Loader] Filesystem not ready"); return; }

    Serial.printf("ROM in %s:\n", folder);
    Serial.println("==========================================");

    auto isROM = [](const char* name) -> bool {
        size_t len = strlen(name);
        if (len < 5) return false;
        const char* ext = name + len - 4;
        return strcasecmp(ext, ".sms") == 0 ||
               strcasecmp(ext, ".gg")  == 0 ||
               strcasecmp(ext, ".bin") == 0;
    };

    if (_currentFS == FSType::FS_SD) {
        File root = SD.open(folder);
        if (!root) { Serial.println("  ERROR: folder not found"); return; }
        File entry = root.openNextFile();
        while (entry) {
            if (!entry.isDirectory() && isROM(entry.name())) {
                Serial.printf("  %-40s  %6lu KB\n",
                              entry.name(), (uint32_t)entry.size() / 1024);
            }
            entry.close();
            entry = root.openNextFile();
        }
        root.close();
    } else {
        Dir dir = LittleFS.openDir(folder);
        while (dir.next()) {
            if (isROM(dir.fileName().c_str())) {
                File f = dir.openFile("r");
                if (f) {
                    Serial.printf("  %-40s  %6lu KB\n",
                                  dir.fileName().c_str(), (uint32_t)f.size() / 1024);
                    f.close();
                }
            }
        }
    }
    Serial.println("==========================================");
}