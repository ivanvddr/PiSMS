#pragma once
/*
 * SMSLoader.h — Loader ROM per SMS Emulator (RP2040)
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 * Basato su MSXLoader del progetto PiMSX.
 *
 *
 *  1) POOL DI BANK IN RAM (SMS_POOL_BANKS × 16 KB, default 6 = 96 KB)
 *     Il framebuffer da 96 KB non esiste più (il rendering va a blocchi
 *     su Core1), quindi la RAM liberata ospita più bank di ROM:
 *       - ROM ≤ 128 KB: caricata INTERAMENTE all'avvio, file SD chiuso,
 *         nessun accesso SD durante il gioco (prima ogni bank switch
 *         = lettura SD da 16 KB ≈ 15-20 ms di stallo).
 *       - ROM più grandi: cache LRU dei bank; si legge dalla SD solo
 *         per i bank non presenti nel pool.
 *
 *  2) TABELLE DI PAGINA (64 pagine da 1 KB)
 *     read()/write() diventano 1 lookup + 1 accesso, senza catena di
 *     if. Gli slot ROM puntano a un buffer qualsiasi del pool, quindi
 *     il bank switch è un cambio di puntatori. Anche il Z80 le usa
 *     direttamente (vedi Z80::setPages): niente chiamata a funzione
 *     per ogni accesso in memoria.
 *
 * ============================================================
 *
 *   $0000-$03FF   sempre dal bank fisico 0 (quirk hardware)
 *   $0400-$3FFF   slot 0   ($FFFD)
 *   $4000-$7FFF   slot 1   ($FFFE)
 *   $8000-$BFFF   slot 2   ($FFFF) oppure RAM cartuccia ($FFFC)
 *   $C000-$DFFF   RAM sistema 8 KB;  $E000-$FFFF mirror + registri mapper
 */

#include <Arduino.h>
#include <SD.h>
#include <LittleFS.h>

#define LOADER_RAMFUNC __attribute__((section(".time_critical.loader")))

enum class FSType {
    FS_NONE,
    FS_SD,
    FS_LITTLEFS
};

#ifndef SD_CS
  #define SD_CS    13
  #define SD_MISO  12
  #define SD_MOSI  11
  #define SD_SCLK  10
#endif

enum class SMSMapper : uint8_t {
    NONE,
    SEGA,
    CODEMASTERS,
    KOREAN
};

static constexpr uint32_t SMS_BANK_SIZE        = 0x4000;
static constexpr uint32_t SMS_CACHE_SIZE       = 0xC000;   // soglia mapper NONE (48 KB)
static constexpr uint32_t SMS_RAM_SIZE_L       = 0x2000;
static constexpr uint32_t SMS_CART_RAM_SIZE    = 0x8000;
static constexpr uint32_t SMS_FIXED_KB         = 0x0400;
static constexpr uint8_t  SMS_MAX_BANKS        = 64;
static constexpr uint8_t  SMS_SLOTS            = 3;
static constexpr uint8_t  SMS_POOL_BANKS       = 6;        // 6 × 16 KB = 96 KB (lascia ca 32 KB per la CartRAM)

class SMSLoader {
public:
    SMSLoader();
    ~SMSLoader() { unloadROM(); }

    // ── Filesystem ───────────────────────────────
    bool    initFilesystem(FSType preferred = FSType::FS_SD, int sdCsPin = 5);
    FSType  currentFS() const { return _currentFS; }
    bool    fsReady()   const { return _fsReady; }
    void    closeFilesystem();

    // ── ROM ───────────────────────────────────────
    bool    loadROM(const char* path);
    void    unloadROM();
    bool    isLoaded() const { return _loaded; }

    // ── Accesso memoria ───────────────────────────
    inline uint8_t read(uint16_t addr) const {
        return _rd[addr >> 10][addr & 0x3FF];
    }
    inline void write(uint16_t addr, uint8_t value) {
        uint8_t* p = _wr[addr >> 10];
        if (p) p[addr & 0x3FF] = value;
        else   writeSlow(addr, value);
    }
    
    // Percorso lento: registri mapper, Codemasters, pagina $FC00-$FFFF
    void writeSlow(uint16_t addr, uint8_t value);

    // Tabelle di pagina per Z80::setPages()
    const uint8_t* const* readPages()  const { return _rd; }
    uint8_t* const*       writePages()       { return _wr; }

    // ── Info ──────────────────────────────────────
    uint32_t    romSize()    const { return _romSize; }
    uint8_t     numBanks()   const { return _numBanks; }
    SMSMapper   mapperType() const { return _mapper; }
    const char* romName()    const { return _romName; }
    uint8_t     slotBank(int s) const { return (s < 3) ? _bankReg[s] : 0; }

    // ── Debug / utilità ───────────────────────────
    void        debugStatus() const;
    void        listROMs(const char* folder = "/roms");
    bool        fileExists(const char* path);
    void        setSelectedROM(const char* path);
    File        openFile(const char* path, const char* mode);
    FSType      getCurrentFilesystem() const { return _currentFS; }
    const char* getSelectedROM() const { return _romPath; }

private:
    // ── Pool bank ROM ─────────────────────────────
    uint8_t  _pool[SMS_POOL_BANKS][SMS_BANK_SIZE];
    uint8_t  _poolBank[SMS_POOL_BANKS];    // bank contenuto (0xFF = vuoto)
    uint32_t _poolStamp[SMS_POOL_BANKS];   // per LRU
    uint32_t _stamp;
    uint8_t  _slotPool[SMS_SLOTS];         // indice pool mappato su ogni slot

    uint8_t  _bank0Fixed[SMS_FIXED_KB];
    uint8_t  _ram[SMS_RAM_SIZE_L];
    uint8_t  _sink[1024];                  // destinazione scritture su ROM

    // ── Tabelle di pagina (64 × 1 KB) ─────────────
    const uint8_t* _rd[64];
    uint8_t*       _wr[64];               // nullptr = percorso lento

    // ── RAM cartuccia (lazy) ──────────────────────
    uint8_t* _cartRam;
    bool     _cartRamAllocated;

    // ── Mapper ────────────────────────────────────
    uint8_t  _bankReg[SMS_SLOTS];
    uint8_t  _regFFFC;
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
    bool      _hasHeader;

    // ── Filesystem ────────────────────────────────
    FSType _currentFS = FSType::FS_NONE;
    bool   _fsReady;
    int    _sdCsPin;
    File   _romFile;
    bool   _fileOpen;

    bool initSD();
    bool openROMFile();
    void closeROMFile();

    bool loadBank(uint8_t slot, uint8_t bank);
    bool readBank(uint8_t bank, uint8_t* dst);
    bool readFromFile(uint32_t offset, uint8_t* dst, uint32_t len);

    void mapSlot(uint8_t slot);
    void mapRam();
    void rebuildPages();

    void writeSegaMapper(uint16_t addr, uint8_t value);
    void writeCodemastersMapper(uint16_t addr, uint8_t value);

    SMSMapper detectMapper();
    bool      detectHeader();
    void      extractName(const char* path);
};