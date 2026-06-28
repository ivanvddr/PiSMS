/*
 * SMSLoader.cpp — Loader ROM per SMS Emulator (RP2040)
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 * Basato su MSXLoader del progetto PiMSX.
 */
#include "SMSLoader.h"

// ─────────────────────────────────────────────────
//  Costruttore
// ─────────────────────────────────────────────────
SMSLoader::SMSLoader()
    : _loaded(false), _romSize(0), _numBanks(0)
    , _mapper(SMSMapper::NONE), _hasHeader(false)
    , _currentFS(FSType::FS_NONE), _fsReady(false)
    , _sdCsPin(5), _fileOpen(false)
    , _cartRamEnabled(false), _cartRamMapped(false)
    , _cartRamBank(0), _regFFFC(0)
    , _cartRam(nullptr), _cartRamAllocated(false)
{
    memset(_romCache,    0xFF, sizeof(_romCache));
    memset(_bank0Fixed,  0xFF, sizeof(_bank0Fixed));
    memset(_ram,         0x00, sizeof(_ram));
    // _cartRam NON allocata qui: lazy su richiesta del gioco
    memset(_bankReg,     0,    sizeof(_bankReg));
    _romName[0] = '\0';
    _romPath[0] = '\0';
}

// ─────────────────────────────────────────────────
//  initFilesystem
//  Identico a MSXLoader::initFilesystem():
//  prova la sorgente preferita, poi fallback
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
        Serial.println("[Loader] SD non disponibile");
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
        Serial.println("[Loader] LittleFS non disponibile");
        return false;
    };

    if (preferred == FSType::FS_SD) {
        if (trySD())  return true;
        if (tryLFS()) return true;
    } else {
        if (tryLFS()) return true;
        if (trySD())  return true;
    }

    Serial.println("[Loader] ERRORE: nessun filesystem disponibile");
    return false;
}

void SMSLoader::closeFilesystem() {
    closeROMFile();
    if (_currentFS == FSType::FS_SD) {
        SD.end();
        Serial.println("[Loader] SD chiusa");
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
    return SD.begin(SD_CS, SPI1);
}

// ─────────────────────────────────────────────────
//  loadROM — entry point principale
//  Pattern identico a MSXLoader::caricaROM()
// ─────────────────────────────────────────────────
bool SMSLoader::loadROM(const char* path) {
    if (!path || path[0] == '\0') return false;

    if (!_fsReady) {
        if (!initFilesystem()) {
            Serial.println("[Loader] ERRORE: filesystem non disponibile");
            return false;
        }
    }

    if (_loaded) unloadROM();

    strncpy(_romPath, path, sizeof(_romPath) - 1);
    _romPath[sizeof(_romPath) - 1] = '\0';
    extractName(path);

    Serial.printf("\n[Loader] Caricamento: %s\n", path);

    // ── Apri file ────────────────────────────────
    if (!openROMFile()) return false;

    uint32_t fileSize = _romFile.size();
    Serial.printf("[Loader] Dimensione file: %lu byte\n", fileSize);

    if (fileSize < 16) {
        Serial.println("[Loader] ERRORE: file troppo piccolo");
        closeROMFile(); return false;
    }

    // ── Rileva header .sms (512 byte) ────────────
    // Alcune ROM hanno un header aggiuntivo all'inizio.
    // Se fileSize % 16384 == 512 → header presente
    _hasHeader = detectHeader();
    uint32_t headerOffset = _hasHeader ? 512 : 0;
    _romSize = fileSize - headerOffset;

    if (_romSize == 0 || _romSize > (uint32_t)SMS_MAX_BANKS * SMS_BANK_SIZE) {
        Serial.printf("[Loader] ERRORE: dimensione ROM non valida (%lu byte)\n", _romSize);
        closeROMFile(); return false;
    }

    _numBanks = (uint8_t)((_romSize + SMS_BANK_SIZE - 1) / SMS_BANK_SIZE);

    // ── Rileva mapper ────────────────────────────
    _mapper = detectMapper();

    Serial.printf("[Loader] ROM: %-32s  %lu KB  %d bank\n",
                  _romName, _romSize / 1024, _numBanks);
    Serial.printf("[Loader] Mapper: %s  Header: %s\n",
                  _mapper == SMSMapper::NONE         ? "NONE"        :
                  _mapper == SMSMapper::SEGA         ? "SEGA"        :
                  _mapper == SMSMapper::CODEMASTERS  ? "CODEMASTERS" : "KOREAN",
                  _hasHeader ? "sì (512B skippati)" : "no");

    // ── Carica i 3 bank iniziali ─────────────────
    memset(_romCache, 0xFF, sizeof(_romCache));
    _bankReg[0] = 0;
    _bankReg[1] = (_numBanks > 1) ? 1 : 0;
    _bankReg[2] = (_numBanks > 2) ? 2 : 0;

    for (uint8_t s = 0; s < SMS_SLOTS; s++) {
        if (!loadBank(s, _bankReg[s])) {
            Serial.printf("[Loader] ERRORE caricamento bank iniziale slot %d\n", s);
            closeROMFile(); return false;
        }
    }

    // ── Copia primo KB fisso (bank 0 fisico) ─────
    // Identico al quirk hardware: $0000-$03FF sempre da bank 0
    memcpy(_bank0Fixed, _romCache, SMS_FIXED_KB);

    // ── Chiudi file se ROM piccola (≤ 48 KB) ─────
    // Come MSXLoader: se non serve swap, chiudi subito
    if (_mapper == SMSMapper::NONE) {
        closeROMFile();
        Serial.println("[Loader] ROM ≤ 48KB: file chiuso (no swap necessario)");
    }
    // Altrimenti il file rimane aperto per i bank switch

    _loaded = true;

    // Reset RAM sistema e cartuccia
    memset(_ram,     0x00, sizeof(_ram));
    memset(_cartRam, 0xFF, sizeof(_cartRam));
    _regFFFC       = 0;
    _cartRamEnabled = false;
    _cartRamMapped  = false;
    _cartRamBank    = 0;

    Serial.println("[Loader] ROM pronta!\n");
    return true;
}

void SMSLoader::unloadROM() {
    closeROMFile();
    memset(_romCache,   0xFF, sizeof(_romCache));
    memset(_bank0Fixed, 0xFF, sizeof(_bank0Fixed));
    memset(_ram,        0x00, sizeof(_ram));
    memset(_bankReg,    0,    sizeof(_bankReg));
    // Libera CartRAM se allocata
    if (_cartRamAllocated && _cartRam) {
        free(_cartRam);
        _cartRam          = nullptr;
        _cartRamAllocated = false;
        Serial.println("[Loader] CartRAM liberata");
    }
    _loaded     = false;
    _romSize    = 0;
    _numBanks   = 0;
    _mapper     = SMSMapper::NONE;
    _hasHeader  = false;
    _regFFFC        = 0;
    _cartRamEnabled = false;
    _cartRamMapped  = false;
    _cartRamBank    = 0;
    _romName[0] = '\0';
    _romPath[0] = '\0';
}

// ─────────────────────────────────────────────────
//  loadBank — swap bank su slot
//  Pattern identico a MSXLoader::caricaBanco()
//  Ricarica il bank solo se diverso da quello attuale
// ─────────────────────────────────────────────────
bool SMSLoader::loadBank(uint8_t slot, uint8_t bank) {
    if (slot >= SMS_SLOTS) return false;

    // Clampa al numero reale di bank (come MSXLoader)
    if (_numBanks > 0) bank = bank % _numBanks;

    // Evita reload inutile (identico a MSXLoader)
    if (_loaded && _bankReg[slot] == bank) return true;

    uint32_t offset = (uint32_t)bank * SMS_BANK_SIZE + (_hasHeader ? 512 : 0);
    uint32_t len    = SMS_BANK_SIZE;

    // Ultimo bank: potrebbe essere più corto
    if (offset + len > (_romSize + (_hasHeader ? 512 : 0))) {
        len = _romSize - (offset - (_hasHeader ? 512 : 0));
    }

    uint8_t* dst = slotPtr(slot);

    if (!readFromFile(offset, dst, len)) {
        Serial.printf("[Loader] ERRORE lettura bank %d slot %d\n", bank, slot);
        memset(dst, 0xFF, SMS_BANK_SIZE);
        return false;
    }

    // Padding se bank parziale (ultimo bank più corto di 16 KB)
    if (len < SMS_BANK_SIZE) {
        memset(dst + len, 0xFF, SMS_BANK_SIZE - len);
    }

    _bankReg[slot] = bank;

    //Serial.printf("[Loader] Slot %d → bank %d ($%04X)\n",
    //              slot, bank, slot * (uint32_t)SMS_BANK_SIZE);
    return true;
}

// ─────────────────────────────────────────────────
//  readFromFile
//  Identico a MSXLoader: apre, seek, legge, chiude
//  (il file può essere già aperto per mapper attivi)
// ─────────────────────────────────────────────────
bool SMSLoader::readFromFile(uint32_t offset, uint8_t* dst, uint32_t len) {
    // Se il file non è aperto (ROM piccola già caricata) > errore
    // Non dovrebbe mai accadere per mapper attivi
    if (!_fileOpen) {
        if (!openROMFile()) return false;		 
    }
	
    if (!_romFile.seek(offset)) {
        Serial.printf("[Loader] ERRORE seek offset %lu\n", offset);	  
        return false;			   
    }

    uint32_t read = _romFile.read(dst, len);
    return (read == len);
}

// ─────────────────────────────────────────────────
//  Mapper SEGA ($FFFC-$FFFF nel mirror RAM)
//
//  $FFFC: registro controllo RAM cartuccia
//    bit 3: abilita RAM cartuccia
//    bit 2: mappa RAM su slot 2
//    bit 1: seleziona bank RAM (0 o 1)
//  $FFFD: bank slot 0
//  $FFFE: bank slot 1
//  $FFFF: bank slot 2
// ─────────────────────────────────────────────────
void SMSLoader::writeSegaMapper(uint16_t addr, uint8_t value) {
    switch (addr) {
        case 0xFFFC:
            _regFFFC        = value;
            _cartRamEnabled = (value & 0x08) != 0;
            _cartRamBank    = (value & 0x02) ? 1 : 0;

            // Alloca CartRAM lazy solo se il gioco la abilita per la prima volta
            if (_cartRamEnabled && !_cartRamAllocated) {
                _cartRam = (uint8_t*)malloc(SMS_CART_RAM_SIZE);
                if (_cartRam) {
                    memset(_cartRam, 0xFF, SMS_CART_RAM_SIZE);
                    _cartRamAllocated = true;
                    Serial.printf("[Loader] CartRAM allocata: %lu KB\n",
                                  (uint32_t)SMS_CART_RAM_SIZE / 1024);
                } else {
                    Serial.println("[Loader] WARN: CartRAM malloc fallito, gioco potrebbe non funzionare");
                    _cartRamEnabled = false;
                }
            }
            _cartRamMapped = _cartRamEnabled && (value & 0x04) != 0;
            break;

        case 0xFFFD: {
            uint8_t bank = value % _numBanks;
            if (bank != _bankReg[0]) {
                loadBank(0, bank);
                // Dopo ogni bank switch dello slot 0,
                // il primo KB fisso NON cambia (quirk hardware)
                // non aggiorniamo _bank0Fixed
            }
            break;
        }
        case 0xFFFE:
            loadBank(1, value % _numBanks);
            break;

        case 0xFFFF:
            if (!_cartRamMapped) {
                loadBank(2, value % _numBanks);
            }
            break;

        default:
            break;
    }
}

// ─────────────────────────────────────────────────
//  Mapper Codemasters
//  Bank switch su scritture nella ROM:
//  $0000 → slot 0, $4000 → slot 1, $8000 → slot 2
// ─────────────────────────────────────────────────
void SMSLoader::writeCodemastersMapper(uint16_t addr, uint8_t value) {
    uint8_t bank = value % _numBanks;
    if      (addr == 0x0000 && bank != _bankReg[0]) loadBank(0, bank);
    else if (addr == 0x4000 && bank != _bankReg[1]) loadBank(1, bank);
    else if (addr == 0x8000 && bank != _bankReg[2]) loadBank(2, bank);
}

// ─────────────────────────────────────────────────
//  Mapper coreano
//  Bank switch su scrittura a $A000 nella RAM
// ─────────────────────────────────────────────────
void SMSLoader::writeKoreanMapper(uint16_t addr, uint8_t value) {
    if (addr == 0xA000) {
        uint8_t bank = value % _numBanks;
        if (bank != _bankReg[2]) loadBank(2, bank);
    }
}

// ─────────────────────────────────────────────────
//  detectHeader
//  Un file .sms ha header se fileSize % 16384 == 512
// ─────────────────────────────────────────────────
bool SMSLoader::detectHeader() {
    if (!_fileOpen) return false;
    uint32_t fileSize = _romFile.size();
    bool has = ((fileSize % SMS_BANK_SIZE) == 512);
    if (has) Serial.println("[Loader] Header .sms rilevato (512B)");
    return has;
}

// ─────────────────────────────────────────────────
//  detectMapper
//  Analisi della ROM per identificare il mapper.
//  Identico a MemoryMapper::detectMapper() ma
//  legge dal file già aperto (non dal buffer).
//
//  Euristica:
//  1. ROM ≤ 48 KB → NONE (flat, no swap)
//  2. Signature "TMR SEGA" a $7FF0 → SEGA
//  3. Checksum Codemasters a $7FE0 → CODEMASTERS
//  4. Default: SEGA
// ─────────────────────────────────────────────────
SMSMapper SMSLoader::detectMapper() {
    if (_romSize <= SMS_CACHE_SIZE) return SMSMapper::NONE;

    uint32_t hdrOff = _hasHeader ? 512 : 0;

    // Leggi zona header SMS a $7FF0 (nel secondo bank)
    uint8_t buf[16] = {0};
    if (_romSize >= 0x8000) {
        readFromFile(hdrOff + 0x7FF0, buf, 16);
    }

    const uint8_t tmrSega[] = {'T','M','R',' ','S','E','G','A'};
    bool hasTMR = (memcmp(buf, tmrSega, 8) == 0);

    // Leggi zona Codemasters a $7FE0
    uint8_t cmBuf[8] = {0};
    if (_romSize >= 0x8000) {
        readFromFile(hdrOff + 0x7FE0, cmBuf, 8);
    }

    // Check Codemasters: byte[3] + byte[7] == 0, e byte[3] != 0
    bool isCodemasters = !hasTMR &&
                         ((uint8_t)(cmBuf[3] + cmBuf[7]) == 0) &&
                         (cmBuf[3] != 0);

    if (isCodemasters) return SMSMapper::CODEMASTERS;
    return SMSMapper::SEGA;
}

// ─────────────────────────────────────────────────
//  Helpers filesystem
// ─────────────────────────────────────────────────
bool SMSLoader::openROMFile() {
    if (_fileOpen) return true;
    if (!_fsReady) return false;

    if (_currentFS == FSType::FS_SD) {
        _romFile = SD.open(_romPath, FILE_READ);
    } else {
        _romFile = LittleFS.open(_romPath, "r");
    }

    _fileOpen = (bool)_romFile;
    if (!_fileOpen) {
        Serial.printf("[Loader] ERRORE: impossibile aprire %s\n", _romPath);
    }
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

// ============================================================================
// SELEZIONE ROM
// ============================================================================

void SMSLoader::setSelectedROM(const char* path) {
    if (!path) return;
    strncpy(_romPath, path, sizeof(_romPath) - 1);
    _romPath[sizeof(_romPath) - 1] = '\0';
    Serial.printf("SMSLoader: ROM selected > %s\n", _romPath);
}

// ─────────────────────────────────────────────────
//  debugStatus — identico a MSXLoader::debugMapperStatus()
// ─────────────────────────────────────────────────
void SMSLoader::debugStatus() const {
    if (!_loaded) { Serial.println("[Loader] Nessuna ROM caricata"); return; }

    Serial.println("\n=== SMSLoader Status ===");
    Serial.printf("ROM     : %s  (%lu KB)\n", _romName, _romSize / 1024);
    Serial.printf("Mapper  : %s   Bank: %d\n",
                  _mapper == SMSMapper::NONE        ? "NONE"        :
                  _mapper == SMSMapper::SEGA        ? "SEGA"        :
                  _mapper == SMSMapper::CODEMASTERS ? "CODEMASTERS" : "KOREAN",
                  _numBanks);
    Serial.printf("Header  : %s\n", _hasHeader ? "sì" : "no");
    Serial.printf("File    : %s\n", _fileOpen  ? "aperto (swap attivo)" : "chiuso");
    Serial.printf("CartRAM : enabled=%d mapped=%d bank=%d\n",
                  _cartRamEnabled, _cartRamMapped, _cartRamBank);
    for (int s = 0; s < SMS_SLOTS; s++) {
        Serial.printf("  Slot %d ($%04X-$%04X) → bank %d\n",
                      s, s*0x4000, s*0x4000+0x3FFF, _bankReg[s]);
    }
    Serial.println("========================\n");
}

bool SMSLoader::fileExists(const char* path) {
    if (_currentFS == FSType::FS_SD)       return SD.exists(path);
    if (_currentFS == FSType::FS_LITTLEFS) return LittleFS.exists(path);
    return false;
}

// ─────────────────────────────────────────────────
//  listROMs — elenco file ROM sulla sorgente attiva
//  Identico a MSXLoader::elencaFileROM()
// ─────────────────────────────────────────────────
void SMSLoader::listROMs(const char* folder) {
    if (!_fsReady) { Serial.println("[Loader] Filesystem non pronto"); return; }

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
        if (!root) { Serial.println("  ERRORE: cartella non trovata"); return; }
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
