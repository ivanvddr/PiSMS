/*
 * PSG SN76489 — Implementazione DMA-driven (v2 — DSP edge-accurate)
 *
 * Copyright (C) 2026 Ivan Vettori — GPLv3
 *
 * Flusso completo:
 *
 *   SETUP (Core0, una volta, in SMSsetup):
 *     psg.initPWMDMA(PIN_AUDIO)
 *   AVVIO (Core1, all'inizio di core1_main):
 *     psg.startAudioOnThisCore()
 *   RUNTIME:
 *     Core0: cpu.step() → psg.addCycles() → produce campioni nel ring
 *     DMA timer (hardware, ~44100 Hz): legge dal buffer attivo, scrive
 *       nel registro CC del PWM. Zero intervento CPU.
 *     Ogni 128 campioni: DMA_IRQ_1 scatta SU CORE1 → onDmaIrq():
 *       retrigger immediato + fillBuffer() dal ring.
 */

#include "PSG.h"

// ── Costanti di sintesi ────────────────────────────────
// STEP = larghezza di un campione audio nello stesso spazio a
// virgola fissa usato per i "period" dei canali (16.16).
static constexpr uint32_t STEP      = 0x10000;
static constexpr uint32_t LFSR_INIT = 0x8000;

// PSG_UPDATE_STEP: durata (in unità STEP) di UN ciclo di clock del
// PSG dopo il prescaler interno /16. Period[ch] = PSG_UPDATE_STEP * N.
// calcolato una sola volta perché sample rate e clock sono fissi a compile-time.
static const uint32_t PSG_UPDATE_STEP =
    (uint32_t)(((uint64_t)STEP * PSG_SAMPLE_RATE * 16) / PSG_CLOCK_HZ);

static inline uint32_t periodFromN(uint16_t N) {
    uint32_t p = PSG_UPDATE_STEP * (uint32_t)N;
    return p ? p : PSG_UPDATE_STEP;   // N=0 si comporta come N=1
}

PSG* PSG::s_instance = nullptr;

// ─────────────────────────────────────────────────────
//  Costruttore
// ─────────────────────────────────────────────────────
PSG::PSG()
    : _latchedChannel(0)
    , _latchedIsVolume(false)
    , _latchedFreqLo(0)
    , _writeIdx(0)
    , _readIdx(0)
    , _underruns(0)
    , _writeCount(0)
    , _started(false)
    , _cycleNum(0)
    , _gainQ8(AUDIO_GAIN_DEFAULT)
    , _dcLP(0)
    , _audioPin(0)
    , _sliceNum(0)
    , _pwmByteOffset(0)
    , _dmaChan(-1)
    , _dmaTimer(-1)
    , _activeBuf(0)
{
    reset();
}

// ─────────────────────────────────────────────────────
//  reset — stato DSP. Può essere richiamato in qualsiasi momento
//  da Core0 senza toccare l'hardware DMA/PWM già configurato.
//  NOTA: non resetta _gainQ8 (il volume impostato dall'utente
//  deve sopravvivere a un reset/cambio ROM).
// ─────────────────────────────────────────────────────
void PSG::reset() {
    for (int i = 0; i < 3; i++) {
        _tone[i].toneReg = 0;
        _tone[i].volReg  = 15;            // silenzio
        _tone[i].period  = PSG_UPDATE_STEP;
        _tone[i].count   = (int32_t)PSG_UPDATE_STEP;
        _tone[i].output  = 0;
    }

    _noise.noiseReg = 0;
    _noise.volReg   = 15;
    _noise.lfsr     = LFSR_INIT;
    _noise.period   = PSG_UPDATE_STEP << 5;   // rate 0 di default
    _noise.count    = (int32_t)_noise.period;
    _noise.output   = (uint8_t)(_noise.lfsr & 1);

    _latchedChannel  = 0;
    _latchedIsVolume = false;
    _latchedFreqLo   = 0;

    _writeIdx  = 0;
    _readIdx   = 0;
    _underruns = 0;
    _started   = false;
    _cycleNum  = 0;
    _dcLP      = 0;
}

// ─────────────────────────────────────────────────────
//  initPWMDMA — setup hardware, identico alla v1.
// ─────────────────────────────────────────────────────
void PSG::initPWMDMA(uint8_t audioPin) {
    s_instance = this;
    _audioPin  = audioPin;

    gpio_set_function(audioPin, GPIO_FUNC_PWM);
    _sliceNum = pwm_gpio_to_slice_num(audioPin);

    uint chan = pwm_gpio_to_channel(audioPin);
    _pwmByteOffset = (chan == PWM_CHAN_B) ? 2 : 0;

    pwm_set_wrap(_sliceNum, PWM_WRAP);
    pwm_set_clkdiv(_sliceNum, 1.0f);
    pwm_set_gpio_level(audioPin, PWM_CENTER);
    pwm_set_enabled(_sliceNum, true);

    // ── Riempi i due buffer iniziali (ring vuoto → silenzio) ──
    fillBuffer(0);
    fillBuffer(1);
    _activeBuf = 0;

    // ── Claim dinamico: NIENTE numeri hardcoded ───────────────
    _dmaChan  = dma_claim_unused_channel(true);
    _dmaTimer = dma_claim_unused_timer(true);

    uint32_t sysHz = clock_get_hz(clk_sys);
    uint32_t Y = sysHz / PSG_SAMPLE_RATE;
    if (Y == 0)      Y = 1;
    if (Y > 0xFFFF)  Y = 0xFFFF;
    dma_timer_set_fraction(_dmaTimer, 1, (uint16_t)Y);
    uint32_t actualRate = sysHz / Y;
    uint     dreq       = dma_get_timer_dreq(_dmaTimer);

    dma_channel_config c = dma_channel_get_default_config(_dmaChan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, dreq);

    volatile uint16_t* ccHalfWord =
        (volatile uint16_t*)(((volatile uint8_t*)&pwm_hw->slice[_sliceNum].cc)
                              + _pwmByteOffset);

    dma_channel_configure(
        _dmaChan, &c,
        (void*)ccHalfWord,
        _dmaBuf[_activeBuf],
        AUDIO_DMA_SAMPLES,
        false
    );

    dma_channel_set_irq1_enabled(_dmaChan, true);

    Serial.printf("[PSG] PWM GPIO%d slice=%u chan=%c  carrier=%lu Hz\n",
        audioPin, _sliceNum, (chan == PWM_CHAN_B) ? 'B' : 'A',
        (unsigned long)(sysHz / (PWM_WRAP + 1)));
    Serial.printf("[PSG] DMA chan=%d timer=%d dreq=%u  rate reale=%lu Hz (target %lu)  size=16bit\n",
        _dmaChan, _dmaTimer, dreq,
        (unsigned long)actualRate, (unsigned long)PSG_SAMPLE_RATE);
    Serial.printf("[PSG] Buffer: 2 x %d campioni (~%.1fms latenza totale)\n",
        AUDIO_DMA_SAMPLES, 2000.0f * AUDIO_DMA_SAMPLES / PSG_SAMPLE_RATE);
    Serial.printf("[PSG] DSP: edge-accurate tone/noise, DC-block on, gainQ8=%u\n",
        _gainQ8);
}

void PSG::startAudioOnThisCore() {
    irq_add_shared_handler(DMA_IRQ_1, irqTrampoline,
                            PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_1, true);

    dma_channel_start(_dmaChan);

    Serial.println("[PSG] Audio DMA avviato su Core1");
}

// ─────────────────────────────────────────────────────
//  write — protocollo SN76489, identico alla v1. Cambia solo cosa
//  fanno recalcToneInc/recalcNoiseInc "sotto" (Period invece di
//  phaseInc).
// ─────────────────────────────────────────────────────
void PSG::write(uint8_t value) {
    _writeCount++;

    if (value & 0x80) {
        _latchedChannel  = (value >> 5) & 0x03;
        _latchedIsVolume = (value >> 4) & 0x01;
        _latchedFreqLo   = value & 0x0F;

        if (_latchedIsVolume) {
            uint8_t vol = value & 0x0F;
            if (_latchedChannel < 3) _tone[_latchedChannel].volReg = vol;
            else                     _noise.volReg = vol;

        } else if (_latchedChannel == 3) {
            _noise.noiseReg = value & 0x0F;
            _noise.lfsr     = LFSR_INIT;
            recalcNoiseInc();

        } else {
            uint16_t &reg = _tone[_latchedChannel].toneReg;
            reg = (reg & 0x3F0) | _latchedFreqLo;
            recalcToneInc(_latchedChannel);
        }

    } else {
        if (!_latchedIsVolume && _latchedChannel < 3) {
            uint16_t &reg = _tone[_latchedChannel].toneReg;
            reg = (reg & 0x00F) | ((uint16_t)(value & 0x3F) << 4);
            recalcToneInc(_latchedChannel);
        }
    }
}

void PSG::addCycles(int cycles) {
    _cycleNum += (uint32_t)cycles * PSG_SAMPLE_RATE;

    while (_cycleNum >= PSG_CLOCK_HZ) {
        _cycleNum -= PSG_CLOCK_HZ;

        uint16_t nextWrite = (_writeIdx + 1) & RING_MASK;
        if (nextWrite != _readIdx) {
            _ring[_writeIdx] = renderSample();
            _writeIdx = nextWrite;
        }
    }

    if (!_started && ringUsed() >= RING_PREFILL) {
        _started = true;
    }
}

void PSG::irqTrampoline() {
    if (s_instance) s_instance->onDmaIrq();
}

void PSG::onDmaIrq() {
    if (!dma_channel_get_irq1_status(_dmaChan)) return;
    dma_channel_acknowledge_irq1(_dmaChan);

    uint8_t justFinished = _activeBuf;
    uint8_t next         = 1 - _activeBuf;

    dma_channel_set_read_addr(_dmaChan, _dmaBuf[next], true);
    _activeBuf = next;

    fillBuffer(justFinished);
}

void PSG::fillBuffer(uint8_t bufIdx) {
    uint16_t* buf = _dmaBuf[bufIdx];
    static uint16_t lastSample = 128;

    #ifdef PSG_DEBUG_TEST_TONE
        static uint32_t phase = 0;
        constexpr uint32_t HALF_PERIOD = PSG_SAMPLE_RATE / 440 / 2;
        for (int i = 0; i < AUDIO_DMA_SAMPLES; i++) {
            buf[i] = ((phase / HALF_PERIOD) & 1) ? 200 : 56;
            phase++;
        }
        return;
    #endif

    for (int i = 0; i < AUDIO_DMA_SAMPLES; i++) {
        if (_readIdx != _writeIdx) {
            lastSample = _ring[_readIdx];
            buf[i]   = lastSample;
            _readIdx = (_readIdx + 1) & RING_MASK;
        } else {
            buf[i] = lastSample; // niente salti bruschi sull'underrun
            if (_started) _underruns++;
        }
    }
}

// ─────────────────────────────────────────────────────
//  Ricalcolo periodi — Period/Count invece di phaseInc/phaseAcc.
//  Nessun "freeze" per N piccoli: il loop edge-accurate in
//  tickTone()/tickNoise() gestisce correttamente anche N=1.
// ─────────────────────────────────────────────────────
void PSG::recalcToneInc(int ch) {
    _tone[ch].period = periodFromN(_tone[ch].toneReg);

    if (ch == 2 && (_noise.noiseReg & 0x03) == 3) {
        recalcNoiseInc();
    }
}

void PSG::recalcNoiseInc() {
    uint8_t rate = _noise.noiseReg & 0x03;

    switch (rate) {
        case 0: _noise.period = PSG_UPDATE_STEP << 5; break;  // N/32
        case 1: _noise.period = PSG_UPDATE_STEP << 6; break;  // N/64
        case 2: _noise.period = PSG_UPDATE_STEP << 7; break;  // N/128
        case 3: _noise.period = 2 * _tone[2].period;  break;  // segue tono 2
        default: _noise.period = PSG_UPDATE_STEP << 5; break;
    }
    if (_noise.period == 0) _noise.period = PSG_UPDATE_STEP;
}

inline void PSG::stepLFSR() {
    bool white = (_noise.noiseReg & 0x04) != 0;

    // Sega Master System: i tap sono bit 0 e bit 3.
    uint32_t fb = white
        ? ((_noise.lfsr & 0x0001) ^ ((_noise.lfsr >> 3) & 0x0001))
        : (_noise.lfsr & 0x0001);

    // Registro a 16 bit: il feedback si inietta nel bit 15.
    _noise.lfsr   = (_noise.lfsr >> 1) | (fb << 15);
    _noise.output = (uint8_t)(_noise.lfsr & 0x0001);
}

// ─────────────────────────────────────────────────────
//  tickTone — EDGE-ACCURATE (era phase-accumulator nella v1).
//  Calcola l'esatta frazione di tempo (in unità STEP) che il
//  canale ha trascorso a livello "alto" durante QUESTO campione,
//  gestendo correttamente toggle multipli quando period < STEP
//  (note acute / N piccolo). 
// ─────────────────────────────────────────────────────
inline int32_t PSG::tickTone(int ch) {
    PSGToneChannel& t = _tone[ch];

    // Avanza esattamente STEP unità di tempo "a segmenti": ogni
    // segmento dura fino al MINIMO fra (a) il tempo rimasto nella
    // finestra del campione e (b) il tempo rimasto prima del
    // prossimo toggle. Se l'output è alto durante il segmento, lo
    // si accumula in "hi". Niente assunzioni su quante transizioni
    // cadono nella finestra: ne gestisce correttamente 0, 1 o N,
    // pari o dispari — a differenza della versione precedente, che
    // "saltava" le transizioni a coppie assumendo un numero pari
    // sbagliando lo stato finale quando il numero era dispari.

    uint32_t hi        = 0;
    int32_t  remaining = (int32_t)STEP;

    while (remaining > 0) {
        int32_t segment = (t.count < remaining) ? t.count : remaining;
        if (segment > 0) {
            if (t.output) hi += (uint32_t)segment;
            t.count   -= segment;
            remaining -= segment;
        }
        if (t.count <= 0) {
            t.output ^= 1;
            t.count  += (int32_t)t.period;
        }
    }

    if (t.volReg == 15) return 0;

    uint32_t vol = PSG_VOL_TABLE[t.volReg] >> 1;   // max 16383
    // hi è in [0, STEP]: mappa linearmente a [-vol, +vol].
    // Tutto in 32 bit: hi*2*vol arriva al massimo a ~2.15e9, entra
    // in un uint32_t. STEP è una potenza di 2 (0x10000): la divisione
    // diventa uno shift, niente divisione 64-bit (costosa su M0+,
    // che non ha hardware per moltiplicazioni/divisioni a 64 bit).
    uint32_t scaled = (hi * (vol << 1)) >> 16;   // equivalente a /STEP
    return (int32_t)scaled - (int32_t)vol;
}

// ─────────────────────────────────────────────────────
//  tickNoise — avanza l'LFSR esattamente il numero di volte
//  necessario a "recuperare" il tempo trascorso in questo campione
//  (anche qui, niente più "parità dei flip"), poi campiona il
//  livello corrente. A differenza del tono non peso per area:
//  il rumore è già a banda larga, il punto critico per l'udibilità
//  erano i canali tonali.
// ─────────────────────────────────────────────────────
inline int32_t PSG::tickNoise() {
    _noise.count -= (int32_t)STEP;
    while (_noise.count <= 0) {
        _noise.count += (int32_t)_noise.period;
        stepLFSR();
    }

    if (_noise.volReg == 15) return 0;

    int32_t vol = PSG_VOL_TABLE[_noise.volReg] >> 1;
    return _noise.output ? vol : -vol;
}

// ─────────────────────────────────────────────────────
//  renderSample — mix → DC-blocking → guadagno master → 8 bit.
// ─────────────────────────────────────────────────────
uint8_t PSG::renderSample() {
    int32_t mix = tickTone(0) + tickTone(1) + tickTone(2) + tickNoise();

    // DC-blocking a 1 polo. Coefficiente 255/256 (shift 8): cutoff
    // reale ≈27 Hz — rimuove solo la deriva DC vera, non il basso
    // udibile. Con shift 5 (31/32, versione precedente) il cutoff
    // era ≈220 Hz: tagliava via anche le note basse legittime
    _dcLP = (_dcLP * 255 + mix) >> 8;
    mix  -= _dcLP;

    // Guadagno master modulabile (AUDIO_GAIN). 256 = 1.0x.
    mix = (mix * (int32_t)_gainQ8) >> 8;

    int32_t s = PWM_CENTER + (mix >> AUDIO_MIX_SHIFT);

    if (s < 0)   s = 0;
    if (s > 255) s = 255;

    return (uint8_t)s;
}