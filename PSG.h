#pragma once
/*
 * PSG SN76489 — Architettura DMA-driven (v2 — DSP edge-accurate)
 *
 * Sega Master System / Game Gear
 *
 * ─────────────────────────────────────────────────────────────────
 *  Pipeline audio (nessun busy-wait per il pacing dei campioni):
 *
 *   Core0                              Hardware DMA (autonomo)
 *   ──────────────────────────────     ──────────────────────────────
 *   cpu.step(N cicli)                  DMA timer @ 44100 Hz (DREQ)
 *     └→ psg.addCycles(N)                └→ legge 1 campione da dmaBuf[active]
 *           │                                └→ scrive in PWM CC reg
 *           │  _cycleNum += N × SR
 *           │  while ≥ PSG_CLOCK_HZ
 *           │    renderSample()
 *           └→ _ring[_writeIdx++]      Core1 (IRQ, non busy-wait)
 *                                        └→ onDmaIrq() ogni 128 campioni:
 *                                             1. retrigger su altro buffer
 *                                             2. fillBuffer() dal ring
 * ─────────────────────────────────────────────────────────────────
 *
 *  NOVITÀ v2 rispetto alla v1 (confronto con smsplus/esp_8_bit):
 *
 *  1) SINTESI TONO/NOISE EDGE-ACCURATE
 *     La v1 usava un semplice phase-accumulator che, quando più
 *     transizioni capitavano nello stesso campione audio (note
 *     acute, N piccolo), guardava solo la PARITÀ del numero di
 *     transizioni — causando aliasing/distorsione pesante.
 *     La v2 calcola, per ogni campione, l'esatta frazione di tempo
 *     che l'onda quadra ha trascorso a livello alto (stesso schema
 *     di MAME/smsplus: Period/Count invece di phaseInc/phaseAcc),
 *     gestendo correttamente toggle multipli dentro un campione.
 *     Per questo NON serve più "congelare" il canale per N piccoli
 *     (rimosso TONE_FREEZE_MIN_N): col nuovo schema anche N=1 dà
 *     al massimo ~5 iterazioni per campione, trascurabili.
 *
 *  2) DC-BLOCKING FILTER
 *     smsplus/esp_8_bit applica un filtro passa-alto a 1 polo dopo
 *     il mix per eliminare i "pop"/click quando volume o tono
 *     cambiano bruscamente (cosa che un offset statico non fa).
 *     Aggiunto qui come _dcLP nel renderSample().
 *
 *  3) VOLUME MASTER MODULABILE (AUDIO_GAIN)
 *     AUDIO_GAIN nella v1 era una costante MAI USATA. Qui diventa
 *     un guadagno master lineare in Q8 (256 = 1.0x, comportamento
 *     "storico"), impostabile a runtime con setGain()/setVolume(),
 *     applicato dopo mix e DC-blocking, prima della conversione a
 *     8 bit per il PWM.
 *
 *  Core0 non viene MAI interrotto per l'audio: lo Z80 e il VDP
 *   girano col loro timing originale, indisturbati.
 * ─────────────────────────────────────────────────────────────────
 *
 * Copyright (C) 2025 Ivan Vettori — GPLv3
 */

#include <Arduino.h>
#include <stdint.h>
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

// ── Clock SN76489 ─────────────────────────────────────
static constexpr uint32_t PSG_CLOCK_HZ   = 3579545UL;   // clock Z80/SMS

// ── Sample rate nominale ──────────────────────────────
// Il DMA timer approssima questo valore con un Y intero (X=1, rate
// reale = sysHz/Y). Errore tipico < 0.01%, inudibile.
static constexpr uint32_t PSG_SAMPLE_RATE = 11025;

// ── PWM: wrap=255 ──────────────────────────────────────
// Campioni già a 8 bit (0-255): con wrap=255 il DMA scrive
// direttamente nel registro CC (1 byte, zero conversioni in IRQ).
static constexpr uint16_t PWM_WRAP   = 255;
static constexpr uint8_t  PWM_CENTER = 128;   // silenzio

// ── Ring buffer di produzione (Core0 → refill IRQ) ────
static constexpr int RING_BITS    = 12;
static constexpr int RING_SIZE    = 1 << RING_BITS;
static constexpr int RING_MASK    = RING_SIZE - 1;
static constexpr int RING_PREFILL = PSG_SAMPLE_RATE / 100; // ~10ms

// ── Buffer DMA (double buffer, refill a blocchi) ──────
static constexpr int AUDIO_DMA_SAMPLES = 128;

// ── Volume master (AUDIO_GAIN) ────────────────────────
// Guadagno lineare in fixed-point Q8: 256 = 1.0x (volume "storico",
// quello con cui la tabella attenuazioni del chip è stata pensata).
// Aumentare oltre 256 alza il volume percepito (più rischio di
// hard-clip ai picchi), diminuire lo abbassa. 0 = muto.
static constexpr uint16_t AUDIO_GAIN_UNITY   = 256;
static constexpr uint16_t AUDIO_GAIN_DEFAULT = AUDIO_GAIN_UNITY;

// ── Shift finale di mixing ─────────────────────────────
// Margine fra il volume massimo teorico (4 canali a fondo scala,
// gain=AUDIO_GAIN_UNITY) e il range 0-255 del PWM.
//
// NOTA: con la v2 (sintesi edge-accurate, bug-fix di tickTone),
// l'ampiezza prodotta è quella REALMENTE corretta — il bug della
// v1 la sottostimava in molte situazioni, motivo per cui lo shift
// a 9 "sembrava" andare bene allora. Con l'ampiezza corretta, 9
// satura quasi sempre a 8 bit (hard-clip, distorsione a banda
// larga). Con 11 il margine torna realistico: anche a piena scala
// su tutti i canali si resta sotto al limite del PWM.
static constexpr int AUDIO_MIX_SHIFT = 11;

// ── Volume table SN76489 (curva reale del chip, -2dB/step) ───
static const uint16_t PSG_VOL_TABLE[16] = {
    32767, 26028, 20675, 16422,
    13045, 10362,  8231,  6538,
     5193,  4125,  3276,  2602,
     2067,  1642,  1304,     0
};

// ─────────────────────────────────────────────────────
//  Strutture canali — ora basate su Period/Count (edge-accurate)
//  invece di phaseInc/phaseAcc.
// ─────────────────────────────────────────────────────
struct PSGToneChannel {
    uint16_t toneReg;
    uint8_t  volReg;
    uint32_t period;   // durata di mezzo periodo, in unità STEP (0x10000/campione)
    int32_t  count;    // contatore discendente verso il prossimo toggle
    uint8_t  output;
};

struct PSGNoiseChannel {
    uint8_t  noiseReg;
    uint8_t  volReg;
    uint32_t lfsr;
    uint32_t period;
    int32_t  count;
    uint8_t  output;
};

// ─────────────────────────────────────────────────────
//  Classe PSG
// ─────────────────────────────────────────────────────
class PSG {
public:
    PSG();

    void reset();

    // ── Setup hardware (chiamato UNA VOLTA da Core0, in SMSsetup) ──
    void initPWMDMA(uint8_t audioPin);

    // ── Avvio effettivo (chiamato da Core1, all'inizio di core1_main) ──
    void startAudioOnThisCore();

    // ── Core0: interfaccia Z80 ────────────────────────
    void write(uint8_t value);          // porta 0x7E/0x7F write-only
    void addCycles(int cycles);         // dopo ogni cpu.step()

    // ── Volume master, modulabile a runtime ───────────
    // gainQ8: 256 = 1.0x (riferimento), 0 = muto. Nessun limite
    // superiore imposto: oltre un certo punto si sentirà il clip.
    void setGainQ8(uint16_t gainQ8) { _gainQ8 = gainQ8; }
    uint16_t gainQ8() const { return _gainQ8; }

    // Comodo alias 0.0f-1.0f con curva quadratica (percezione più
    // "naturale" su uno slider): v=1.0 → 2x il volume di riferimento,
    // v≈0.5 → volume di riferimento (1.0x), v=0 → muto.
    void setVolume(float v01) {
        if (v01 < 0.0f) v01 = 0.0f;
        if (v01 > 1.0f) v01 = 1.0f;
        _gainQ8 = (uint16_t)(v01 * v01 * (AUDIO_GAIN_UNITY * 2) + 0.5f);
    }

    // ── Diagnostica ────────────────────────────────────
    uint32_t underruns() const { return _underruns; }
    uint32_t writeCount() const { return _writeCount; }
    int      ringUsed()  const { return (_writeIdx - _readIdx) & RING_MASK; }
    int      ringFree()  const { return RING_SIZE - 1 - ringUsed(); }
    uint8_t  volumeReg(int ch) const {
        return (ch < 3) ? _tone[ch].volReg : _noise.volReg;
    }

private:
    // ── Canali DSP ─────────────────────────────────────
    PSGToneChannel  _tone[3];
    PSGNoiseChannel _noise;

    // ── Stato protocollo write SN76489 ────────────────
    uint8_t _latchedChannel;
    bool    _latchedIsVolume;
    uint8_t _latchedFreqLo;

    // ── Ring buffer SPSC (produzione Core0 → refill IRQ) ──
    uint8_t           _ring[RING_SIZE];
    volatile uint16_t _writeIdx;
    volatile uint16_t _readIdx;
    uint32_t          _underruns;
    uint32_t          _writeCount;
    volatile bool     _started;
    uint32_t          _cycleNum;      // accumulatore razionale cicli→campioni

    // ── Volume master e DC-blocking ────────────────────
    uint16_t _gainQ8;   // fixed-point Q8, 256 = 1.0x
    int32_t  _dcLP;     // stato del filtro passa-alto a 1 polo

    // ── Hardware audio: PWM + DMA ─────────────────────
    uint8_t  _audioPin;
    uint     _sliceNum;
    uint8_t  _pwmByteOffset;
    int      _dmaChan;
    int      _dmaTimer;
    uint16_t _dmaBuf[2][AUDIO_DMA_SAMPLES];
    volatile uint8_t _activeBuf;

    // ── Trampolino IRQ ─────────────────────────────────
    static PSG* s_instance;
    static void irqTrampoline();
    void onDmaIrq();
    void fillBuffer(uint8_t bufIdx);

    // ── Helpers DSP ────────────────────────────────────
    void recalcToneInc(int ch);
    void recalcNoiseInc();
    uint8_t renderSample();
    inline int32_t tickTone(int ch);
    inline int32_t tickNoise();
    inline void    stepLFSR();
};