#pragma once
/*
 * SMSLoader.h — Loader ROM per SMS Emulator (RP2040)
 *
 * Parte del progetto SMS Emulator per Raspberry Pi Pico.
 * Copyright (C) 2026 Ivan Vettori
 *
 * Rilasciato sotto GNU General Public License v3.0.
 *
 * Basato su MSXLoader del progetto PiMSX.
 *
 * ============================================================
 * Architettura memoria SMS (vista Z80, 64 KB)
 * ============================================================
 *
 *   $0000-$3FFF   ROM slot 0  (16 KB, bank selezionato da reg $FFFD)
 *   $4000-$7FFF   ROM slot 1  (16 KB, bank selezionato da reg $FFFE)
 *   $8000-$BFFF   ROM slot 2  (16 KB, bank selezionato da reg $FFFF)
 *                             oppure RAM cartuccia (reg $FFFC bit 2)
 *   $C000-$DFFF   RAM sistema  (8 KB)
 *   $E000-$FFFF   Mirror RAM  + registri mapper ($FFFC-$FFFF)
 *
 * ============================================================
 * Strategia di storage (vincoli Pico 264 KB RAM)
 * ============================================================
 *
 *   ROM ≤ 48 KB (≤ 3 bank):
 *     Caricata interamente in _romCache[] (48 KB fissi in RAM)
 *     Nessun accesso SD/flash durante l'emulazione
 *     Mapper: NONE
 *
 *   ROM > 48 KB → swap da SD o flash:
 *     Solo 3 bank attivi in RAM (3 × 16 KB = 48 KB)
 *     Bank switch ricarica il bank dalla sorgente
 *     Mapper: SEGA / CODEMASTERS / KOREAN
 *
 *   Sorgente ROM (in ordine di priorità):
 *     1. Flash interna Pico (LittleFS) — zero latenza SD
 *     2. SD card — file system FAT
 *
 * ============================================================
 * Primo KB fisso (quirk hardware SMS)
 * ============================================================
 *
 *   I byte $0000-$03FF sono sempre letti dal bank fisico 0,
 *   indipendentemente da quale bank è mappato nello slot 0.
 *   Il loader mantiene una copia separata _bank0Fixed[1024]
 *   che non viene mai sovrascritta dai bank switch.
 *
 * ============================================================
 * RAM cartuccia
 * ============================================================
 *
 *   Supporto opzionale per RAM cartuccia (es. Phantasy Star).
 *   Registro $FFFC:
 *     bit 3: abilita RAM cartuccia
 *     bit 2: mappa RAM su slot 2 invece di ROM
 *     bit 1: seleziona bank RAM (0 o 1, 16 KB ciascuno)
 *   Dimensione massima: 32 KB (_cartRam[32768])
 *
 * ============================================================
 * Uso da memRead / memWrite (PiSMS.ino)
 * ============================================================
 *
 *   uint8_t smsMemRead(uint16_t addr)  { return loader.read(addr); }
 *   void    smsMemWrite(uint16_t addr, uint8_t v) { loader.write(addr, v); }
 */

#include <Arduino.h>
#include <SD.h>
#include <LittleFS.h>

// ── Filesystem ────────────────────────────────────
enum class FSType {
    FS_NONE,
    FS_SD,
    FS_LITTLEFS
};

// ============================================================================
// Pin SD
// ============================================================================

#ifndef SD_CS
  #define SD_CS    13
  #define SD_MISO  12
  #define SD_MOSI  11
  #define SD_SCLK  10
#endif

// ── Mapper ────────────────────────────────────────
enum class SMSMapper : uint8_t {
    NONE,           // ROM ≤ 48 KB, flat (no banking)
    SEGA,           // Mapper Sega standard (95% dei titoli)
    CODEMASTERS,    // Mapper Codemasters
    KOREAN          // Mapper coreano (scrittura a $A000)
};

// ── Costanti ──────────────────────────────────────
static constexpr uint32_t SMS_BANK_SIZE        = 0x4000;  // 16 KB
static constexpr uint32_t SMS_CACHE_SIZE       = 0xC000;  // 48 KB (3 bank in RAM)
static constexpr uint32_t SMS_RAM_SIZE_L       = 0x2000;  // 8 KB RAM sistema
static constexpr uint32_t SMS_CART_RAM_SIZE    = 0x8000;  // 32 KB RAM cartuccia
static constexpr uint32_t SMS_FIXED_KB         = 0x0400;  // 1 KB fisso da bank 0
static constexpr uint8_t  SMS_MAX_BANKS        = 64;       // 64 × 16 KB = 1 MB max
static constexpr uint8_t  SMS_SLOTS            = 3;        // slot 0, 1, 2

// ─────────────────────────────────────────────────
//  Classe SMSLoader
// ─────────────────────────────────────────────────
class SMSLoader {
public:
    SMSLoader();
    ~SMSLoader() { unloadROM(); }

    // ── Filesystem ───────────────────────────────
    bool        initFilesystem(FSType preferred = FSType::FS_SD,
                               int sdCsPin = 5);
    FSType   currentFS()      const { return _currentFS; }
    bool        fsReady()        const { return _fsReady; }
    void        closeFilesystem();

    // ── Caricamento ROM ───────────────────────────
    // path: percorso file (es. "/roms/sonic.sms")
    // Ritorna true se la ROM è stata caricata correttamente.
    bool        loadROM(const char* path);
    void        unloadROM();
    bool        isLoaded()       const { return _loaded; }

    // ── Interfaccia CPU Z80 ───────────────────────
    // Chiamate da smsMemRead() / smsMemWrite() in PiSMS.ino
    // Inline per massima performance (identico a MSXLoader)
    inline uint8_t read(uint16_t addr);
    inline void    write(uint16_t addr, uint8_t value);

    // ── Info ROM ──────────────────────────────────
    uint32_t    romSize()        const { return _romSize; }
    uint8_t     numBanks()       const { return _numBanks; }
    SMSMapper   mapperType()     const { return _mapper; }
    const char* romName()        const { return _romName; }
    uint8_t     slotBank(int s)  const { return (s<3) ? _bankReg[s] : 0; }

    // ── Debug ─────────────────────────────────────
    void        debugStatus()    const;
    void        listROMs(const char* folder = "/roms");
    bool        fileExists(const char* path); 
    void        setSelectedROM(const char* path);
    File        openFile(const char* path, const char* mode);
    FSType      getCurrentFilesystem() const { return _currentFS; }
    const char* getSelectedROM() const { return _romPath; }

private:
    // ── Cache ROM (3 bank attivi in RAM) ──────────
    // per 3 slot SMS da 16 KB ciascuno
    uint8_t  _romCache[SMS_CACHE_SIZE];     // 48 KB: slot0 + slot1 + slot2

    // ── Primo KB fisso (quirk hardware SMS) ───────
    // Copia immutabile dei byte $0000-$03FF del bank fisico 0.
    // Restituita sempre per letture in quel range,
    // anche se lo slot 0 è stato switchato ad altro bank.
    uint8_t  _bank0Fixed[SMS_FIXED_KB];

    // ── RAM sistema (8 KB) ────────────────────────
    uint8_t  _ram[SMS_RAM_SIZE_L];

    // ── RAM cartuccia (opzionale, lazy alloc) ───────
    // Allocata su heap SOLO quando il gioco la richiede ($FFFC bit 3)
    // Libera la pressione in RAM per i giochi che non ne hanno bisogno.
    // Giochi che la usano: Phantasy Star, Ys, ecc. (< 5% dei titoli)
    uint8_t* _cartRam;          // nullptr finché non allocata
    bool     _cartRamAllocated; // true se malloc riuscito

    // ── Registri mapper ───────────────────────────
    // Identici a MemoryMapper ma semplificati:
    // $FFFC: controllo RAM cartuccia
    // $FFFD: bank slot 0
    // $FFFE: bank slot 1
    // $FFFF: bank slot 2
    uint8_t  _bankReg[SMS_SLOTS];    // bank attivo per slot 0/1/2
    uint8_t  _regFFFC;               // registro controllo RAM cartuccia
    bool     _cartRamEnabled;
    bool     _cartRamMapped;
    uint8_t  _cartRamBank;

    // ── Stato ROM ─────────────────────────────────
    bool      _loaded;
    uint32_t  _romSize;
    uint8_t   _numBanks;
    SMSMapper _mapper;
    char      _romName[64];
    char      _romPath[128];
    bool      _hasHeader;            // true: file aveva header 512 byte

    // ── Filesystem ────────────────────────────────
    FSType _currentFS = FSType::FS_NONE;
    bool      _fsReady;
    int       _sdCsPin;
    File      _romFile;
    bool      _fileOpen;

    bool initSD();

    // ── Helpers privati ──────────────────────────
    bool     openROMFile();
    void     closeROMFile();

    // Carica un bank fisico nel slot (0/1/2) della cache
    // Stesso pattern di MSXLoader::caricaBanco()
    bool     loadBank(uint8_t slot, uint8_t bank);

    // Legge N byte dal file ROM a partire da offset fisico
    bool     readFromFile(uint32_t offset, uint8_t* dst, uint32_t len);

    // Gestione write mapper Sega ($FFFC-$FFFF in mirror RAM)
    void     writeSegaMapper(uint16_t addr, uint8_t value);
    void     writeCodemastersMapper(uint16_t addr, uint8_t value);
    void     writeKoreanMapper(uint16_t addr, uint8_t value);

    // Detection mapper e rimozione header
    SMSMapper detectMapper();
    bool      detectHeader();        // ritorna true se file ha header 512B
    void      extractName(const char* path);

    // Puntatore base del slot nella cache (slot 0/1/2 → offset 0/16K/32K)
    inline uint8_t* slotPtr(uint8_t slot) {
        return _romCache + (uint32_t)slot * SMS_BANK_SIZE;
    }
};

// ─────────────────────────────────────────────────
//  read — inline (hot path)
//  Identico a MSXLoader::readCart() ma con
//  gestione completa della mappa SMS
// ─────────────────────────────────────────────────
inline uint8_t SMSLoader::read(uint16_t addr) {

    // ── Primo KB fisso (sempre bank 0 fisico) ─────
    // Quirk hardware: $0000-$03FF non è soggetto a bank switch
    if (addr < SMS_FIXED_KB) {
        return _bank0Fixed[addr];
    }

    // ── ROM slot 0 ($0000-$3FFF) ──────────────────
    if (addr < 0x4000) {
        return _romCache[addr];              // slot 0, offset diretto
    }

    // ── ROM slot 1 ($4000-$7FFF) ──────────────────
    if (addr < 0x8000) {
        return _romCache[addr];              // slot 1 a offset +16K
    }

    // ── ROM slot 2 / RAM cartuccia ($8000-$BFFF) ──
    if (addr < 0xC000) {
        if (_cartRamMapped && _cartRam) {
            uint32_t cartAddr = (addr - 0x8000U)
                              + (uint32_t)_cartRamBank * SMS_BANK_SIZE;
            return _cartRam[cartAddr & (SMS_CART_RAM_SIZE - 1)];
        }
        return _romCache[addr];              // slot 2 a offset +32K
    }

    // ── RAM sistema ($C000-$DFFF) ─────────────────
    if (addr < 0xE000) {
        return _ram[addr - 0xC000U];
    }

    // ── Mirror RAM ($E000-$FFFF) ──────────────────
    return _ram[addr - 0xE000U];
}

// ─────────────────────────────────────────────────
//  write — inline (hot path)
// ─────────────────────────────────────────────────
inline void SMSLoader::write(uint16_t addr, uint8_t value) {

    // ── Slot ROM 0/1: solo Codemasters usa scritture qui ──
    if (addr < 0x8000) {
        if (_mapper == SMSMapper::CODEMASTERS) {
            writeCodemastersMapper(addr, value);
        }
        return;
    }

    // ── Slot 2: RAM cartuccia o ROM (non scrivibile) ──
    if (addr < 0xC000) {
        if (_cartRamMapped && _cartRam) {
            uint32_t cartAddr = (addr - 0x8000U)
                              + (uint32_t)_cartRamBank * SMS_BANK_SIZE;
            _cartRam[cartAddr & (SMS_CART_RAM_SIZE - 1)] = value;
        }
        return;
    }

    // ── RAM sistema ($C000-$DFFF) ─────────────────
    if (addr < 0xE000) {
        _ram[addr - 0xC000U] = value;
        if (_mapper == SMSMapper::KOREAN) {
            writeKoreanMapper(addr, value);
        }
        return;
    }

    // ── Mirror RAM + registri mapper ($E000-$FFFF) ─
    _ram[addr - 0xE000U] = value;

    if (_mapper == SMSMapper::SEGA || _mapper == SMSMapper::NONE) {
        writeSegaMapper(addr, value);
    }
}
