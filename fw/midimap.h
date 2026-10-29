/* MIDI pages: which MIDI channel drives which voice and what its controls do. Read from
 * /midimap.json on the SD card (written there with the defaults when missing), so the
 * mapping can be edited without a firmware build. No hardware here (host-testable:
 * host/test_midimap.c). Example of the file:
 *
 *   {
 *     "clockFollow": true, "cutterChannel": 7,
 *     "pages": [
 *       { "channel": 5, "voice": 0, "cc": 16, "rate": [0, 4],
 *         "knobs": ["bufNum", "bank", "rate", "pitch", "gate", "interval", "repeats", "fxSend", ...],
 *         "velocity": "volume", "noteOff": true, "mw": "algo", "at": "grainDur", "pb": "cutter" },
 *       ...
 *     ]
 *   }
 *
 * channel: 1..16 as the instrument shows it. voice: 0 drums, 1 melody, 2 third voice.
 * cc: the first CC of the page; the knobs follow on consecutive CCs. rate: the rate knob's
 * range (63 = 1). velocity / mw / at / pb: what a note's velocity, the mod wheel, channel
 * aftertouch and pitch bend do (pitch bend: its coarse 7 bits, all the Machinedrum sends).
 */
#ifndef MIDIMAP_H
#define MIDIMAP_H

#include <stdint.h>

#define MM_MAX_PAGES 8
#define MM_MAX_KNOBS 24

/* what a control can drive (names as in the JSON file) */
enum {
    MT_NONE, MT_BUFNUM, MT_BANK, MT_RATE, MT_PITCH, MT_GATE, MT_INTERVAL, MT_AMP, MT_OVERLAP, MT_RANDPOS,
    MT_RANDOMPITCH, MT_PROB, MT_SEPARATION, MT_MAXGRAINS, MT_TEMPOLOCK, MT_BGADVANCE, MT_ENVHOLD,
    MT_PVTRANSIENT, MT_ALGO, MT_GRAINDUR, MT_NOTEMODE, MT_OUT, MT_CUTTER, MT_REPEATS, MT_FXSEND, MT_VOLUME, MT_NUM
};
extern const char* const mm_target_names[MT_NUM];

struct mm_page {
    uint8_t chan;               /* 0-based MIDI channel                    */
    uint8_t voice, cc, nknobs;
    uint8_t mw, at, pb, vel;    /* MT_* targets (vel: note-on velocity)     */
    uint8_t note_off;           /* 1: a note-off ends the gate              */
    uint8_t knobs[MM_MAX_KNOBS];
    int32_t rate_lo, rate_hi;   /* Q16                                      */
};

struct midimap {
    int npages, clock_follow;
    int cutter_chan;            /* 1..16: MIDI channel forwarded to the ESP's cutter, 0 none */
    struct mm_page page[MM_MAX_PAGES];
};

void midimap_defaults(struct midimap* m);   /* the Machinedrum setup */
/* 0 on success; on failure m is left at the defaults and err says why */
int midimap_parse(struct midimap* m, const char* buf, int len, char* err, int errn);
int midimap_write(const struct midimap* m, char* buf, int n); /* JSON text, returns its length */
int mm_target_of(const char* name);                          /* -1 if unknown */

#endif
