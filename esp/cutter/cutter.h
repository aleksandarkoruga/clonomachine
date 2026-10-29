/* The break cutter: a procedural engine that cuts voice 0's loop. Portable C (no allocation,
 * floats), shared by the ESP32 firmware and the PC stand-in (host/cutter_live.py).
 *
 * The idea (from a family of SuperCollider break cutters, abstracted): leave the loop alone where
 * the meter is strong and move away from it where the meter is weak and tension is high.
 *
 *   metric weight w   metric stability generalised: sum over the units containing the step
 *                     (8th, beat, bar, phrase) of how early in the unit it sits, weighted by the
 *                     unit's length; 1 on the phrase's first step, small just before a downbeat
 *   tension T         rises over the phrase and the section (build: early / linear / late)
 *   deviation d       energy^0.7 * sqrt(T * (1 - w)^(2 anchor)): 0 plays the loop where it is
 *   move              from a palette ordered by disruption (reorder, repeat, mangle, texture,
 *                     silence: character weights the families), its size scaled by d
 *   release           the end of each phrase lets the loop run, fills, or drops out
 *   memory            a phrase's dice can be replayed, mutated with the tension
 *   conductor (auto)  moves the knobs through groove / build / peak / break sections
 *
 * Hits fall on 16ths from a pattern per bar: density hits, placed in metric order by the bits
 * of an "intensity" number; syncopation pushes them to the weak positions.
 *
 * The FPGA sends a tick every 32nd (EV_TICK), the bpm, start/stop, who owns voice 0 and the
 * sample's length in 16ths, and the MIDI messages of the cutter channel (CC 16.. = the knobs,
 * CC 25 phrase bars 1..16, CC 26 phrases per section 1..8, notes = presets, pitch bend =
 * conductor auto / manual). The cutter answers with frames of EV_RAW_PARAM events for slot 0,
 * some of them delayed (ratchets, tape stops). Gate length and volume stay MIDI's (the drum
 * page): the cutter's staccato only closes a gate early.
 */
#ifndef CUTTER_H
#define CUTTER_H

#include <stdint.h>

#include "../../include/sampler_events.h"

enum cut_knob { /* 0..1 each */
    CK_ENERGY,      /* how far the cuts move away from the loop                        */
    CK_DENSITY,     /* hits per bar                                                     */
    CK_SYNCOPATION, /* 0: hits on the strong positions .. 1: on the weak ones           */
    CK_ANCHOR,      /* how much a strong position protects the loop                     */
    CK_BUILD,       /* tension over the phrase: 0 early, 0.5 linear, 1 late surge       */
    CK_CHARACTER,   /* palette: 0 edits (reorder) .. 0.33 rolls .. 0.66 tape .. 1 texture */
    CK_RELEASE,     /* length of the phrase's release, 0 .. half the phrase             */
    CK_MEMORY,      /* 0 new dice every phrase .. 1 the same phrase again               */
    CK_SURPRISE,    /* wider choice of moves, abrupt conductor changes                  */
    CK_NUM
};

enum cut_family { CF_REORDER, CF_REPEAT, CF_MANGLE, CF_TEXTURE, CF_SILENCE, CF_NUM };
enum cut_release { CR_RUN, CR_FILL, CR_DROP };

struct cut_params {
    float knob[CK_NUM];
    float palette[CF_NUM];  /* family weights; set from CK_CHARACTER unless a preset fixes them */
    int palette_fixed;
    int release_mode;       /* enum cut_release */
    float accel;            /* ratchets: 0 even .. 1 accelerating like repeatDur / (1 + k) */
    int beats_per_bar, phrase_bars, section_phrases;
    int base_buf;           /* sample in voice 0's bank the cutter plays */
};

#define CUT_MAX_EV 12      /* events per frame */
#define CUT_QUEUE 64       /* frames waiting for their time */

struct cut_frame {
    double due;             /* seconds, on the host's clock (cut_clock / cut_pop) */
    uint32_t seq;           /* order among frames due together */
    int n;
    struct smp_event ev[CUT_MAX_EV];
};

struct cutter {
    struct cut_params p;    /* the knobs as set                                   */
    struct cut_params e;    /* what plays: the knobs through the conductor        */
    int conductor;          /* 1 auto */
    int preset;
    /* from the FPGA */
    float bpm;
    int owner, playing, n16;
    uint32_t step, have_step;
    /* phrase state */
    uint32_t phrase_seed, rng_fresh, rng_phrase;
    uint32_t phrase_idx;
    uint16_t bar_pattern[16];  /* hits per bar of the phrase (bit k = 16th k) */
    int cond_state, cond_next;
    double busy_until;         /* a ratchet / tape stop owns the voice until then */
    int last_buf;
    float last_rate;
    int stats_hits[16];        /* moves taken, for the console / tests */
    /* output */
    double now;                /* seconds */
    uint32_t seq;
    struct cut_frame q[CUT_QUEUE];
    int nq;
};

void cut_init(struct cutter* c, uint32_t seed);
void cut_preset(struct cutter* c, int n);              /* n % cut_npresets()          */
int cut_npresets(void);
const char* cut_preset_name(int n);
void cut_set_knob(struct cutter* c, int k, float v);
void cut_set_conductor(struct cutter* c, int on);
void cut_set_phrase(struct cutter* c, int bars, int sections); /* 0 keeps; from the next phrase */
/* the host's clock in seconds: set it before cut_rx, which schedules from it */
void cut_clock(struct cutter* c, double t);
/* an event from the FPGA (EV_TICK, EV_BPM, EV_START, EV_STOP, EV_OWNER, EV_LOAD_BUF, the
   cutter channel's EV_CC / EV_NOTE_ON / EV_BEND) */
void cut_rx(struct cutter* c, const struct smp_event* e);
/* the clock moves to t; returns 1 with a frame whose time has come (copied to out), 0 when
   none is due. Call until it returns 0. */
int cut_pop(struct cutter* c, double t, struct cut_frame* out);
const char* cut_move_name(int m);
enum { CUT_NMOVES = 15 };

#endif
