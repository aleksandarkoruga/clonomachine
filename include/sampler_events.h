/* Control events. The ESP32-S3 (or host/osc_bridge.py) turns OSC / MIDI
 * into these and sends them SLIP-framed to the FPGA soft core:
 *
 *   frame   = SLIP( event[0] .. event[n-1], crc8 )      n >= 1
 *   event   = type u8 | chan u8 | a u8 | b u8 | value i32 (little endian)   (8 bytes)
 *   crc8    = CRC-8, poly 0x07, init 0x00, over all event bytes of the frame
 *   SLIP    : END 0xC0 (also sent before the frame), ESC 0xDB, ESC_END 0xDC, ESC_ESC 0xDD
 *
 * Links: USB console UART (115200, mixed with the text shell) and the ESP32-S3 UART on
 * the header (3 Mbaud).
 */
#ifndef SAMPLER_EVENTS_H
#define SAMPLER_EVENTS_H

#include <stdint.h>

enum {
    EV_NOTE_ON   = 0,   /* chan, a = note, b = velocity                                  */
    EV_NOTE_OFF  = 1,   /* chan, a = note                                                */
    EV_CC        = 2,   /* chan, a = cc, b = value                                       */
    EV_PC        = 3,   /* chan, a = program                                             */
    EV_BEND      = 4,   /* chan, value = 0..16383                                        */
    EV_BPM       = 5,   /* value = bpm * 1000                                            */
    EV_CLOCK     = 6,   /* MIDI clock tick (24 ppq)                                      */
    EV_START     = 7,
    EV_STOP      = 8,
    EV_RAW_PARAM = 9,   /* a = slot, b = param id (P_*), value = x * 2^frac (see below)  */
    EV_LOAD_BUF  = 10   /* a = slot, b = buffer index in the slot's bank                 */
};

struct __attribute__((packed)) smp_event {
    uint8_t type, chan, a, b;
    int32_t value;
};

/* RAW_PARAM ids: OSC /slot/<n>/<name> [f]. Names follow BufGrainPlay's arguments.
 * Value fixed point: x * 2^frac. One X() per line: host/osc_bridge.py parses this list.
 *   X(ID, oscName, frac)   meaning / range                                              */
#define SMP_PARAMS(X)                                                                        \
    X(TRIG,        trig,        0)  /* strobe: restart at pos, gate + stutter             */ \
    X(BUFNUM,      bufNum,      0)  /* buffer index inside the slot's bank (= CC1)        */ \
    X(BANK,        bank,        0)  /* bank of the slot's folder (= program change)       */ \
    X(POS,         pos,         30) /* 0..1: >= 0 re-pins every sample, < 0 runs free     */ \
    X(RATE,        rate,        16) /* playhead speed at the global tempo, -8..8          */ \
    X(PITCH,       pitch,       16) /* grain read speed, 0.03..8                          */ \
    X(GRAINDUR,    grainDur,    16) /* seconds, 0.001..1                                  */ \
    X(OVERLAP,     overlap,     16) /* 0..16 (0: no new grains)                           */ \
    X(PROB,        prob,        16) /* 0..1 onset probability                             */ \
    X(SEPARATION,  separation,  16) /* 0..1 onset jitter                                  */ \
    X(RANDPOS,     randPos,     16) /* 0..1 of buffer, random read offset                 */ \
    X(RANDOMPITCH, randomPitch, 16) /* octaves, -4..4                                     */ \
    X(TEMPOLOCK,   tempoLock,   0)  /* 0/1: stretch to the global bpm; 0 = own speed      */ \
    X(BGADVANCE,   bgAdvance,   0)  /* 0/1: position = pos + background playhead          */ \
    X(MAXGRAINS,   maxGrains,   0)  /* 1..32                                              */ \
    X(SEED,        seed,        0)  /* random seed                                        */ \
    X(ENVDUR,      envDur,      16) /* gate length in seconds                             */ \
    X(ENVBARS,     envBars,     16) /* gate length in bars at the global bpm (default 1)  */ \
    X(ENVHOLD,     envHold,     0)  /* 0/1: gate open until note-off / gateOff            */ \
    X(GATEOFF,     gateOff,     0)  /* strobe                                             */ \
    X(REPEATS,     repeats,     0)  /* stutter retriggers 0..126                          */ \
    X(INTERVAL,    interval,    16) /* stutter period, seconds, <= 0.25                   */ \
    X(AMP,         amp,         16) /* 0..16                                              */ \
    X(ALGOAMP,     algoAmp,     16) /* 0..1                                               */ \
    X(ALGO,        algo,        0)  /* 0 grains (BufGrainPlay), 1 phase vocoder (Bungee)  */ \
    X(PVTRANSIENT, pvTransient, 0)  /* 0/1: phase vocoder transient handling (default 1)  */ \
    X(NOTEMODE,    noteMode,    0)  /* note -> pos: 0 equal 16ths, 1 transients, 2 snapped */ \
    X(KILL,        kill,        0)  /* strobe: silence all grains                         */ \
    X(ENABLE,      enable,      0)  /* 0/1                                                */

#define SMP_PARAM_ENUM(id, name, frac) P_##id,
enum { SMP_PARAMS(SMP_PARAM_ENUM) P_NUM_PARAMS };
#undef SMP_PARAM_ENUM

static inline uint8_t smp_crc8(const uint8_t* p, unsigned n) {
    uint8_t c = 0;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; ++i)
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}

#endif
