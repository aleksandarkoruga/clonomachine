/* The harmony generator: functional harmony for the synth's voices, on the ESP32 beside the break
 * cutter. Portable C (no allocation), shared by the ESP firmware and the PC test (host/
 * test_harmony.c). After the user's SuperCollider HarmonyGenerator:
 *
 *   functions   a chain of scale degrees 1..7 with weighted steps: 1 3 6 the tonic group
 *               (resolution), 2 4 preparation, 5 7 tension; it moves forward through the groups
 *               (1 3 6 -> 2 4 -> 5 7 -> back to the tonic only from 5 / 7, jumps forward allowed,
 *               3 never back to 1, 4 never back to 2, 6 never back to 1 or 3), and often stays
 *   melody      a seed of degrees grown by inversion, retrograde, transposition, shuffle and
 *               rotation of pieces of itself; each step it transposes the chord
 *   chords      thirds stacked on the function's degree (+ the melody), in four octaves of the
 *               scale; voiced as the arrangement of the new notes with the least movement from the
 *               last chord that has no parallel fifths or octaves
 *   cadences    the last steps of a phrase lead to a cadence: authentic (5 or 7 -> 1), deceptive
 *               (5 -> 6) or half (ending on 5); never preparation straight to the tonic
 *
 * Time: the FPGA's 32nd ticks (EV_TICK, while the transport plays); a chord step every
 * harmBeats beats. Out: EV_SYN_NOTE (scale degrees: the synth maps them through its scale and
 * tuning) for the voices whose note changes. Parameters: EV_SYN_PARAM with the harm* ids of
 * include/synth_params.h (and scale, for its size). */
#ifndef HARMONY_H
#define HARMONY_H

#include <stdint.h>

#include "../../include/sampler_events.h"

#define HARM_VOICES 3
#define HARM_MAX_MELODY 512
#define HARM_MAX_EV 12

struct harm_frame {
    int n;
    struct smp_event ev[HARM_MAX_EV];
};

struct harmony {
    /* parameters */
    int on, beats, phrase, octave, voices, melody_on, vel, seed;
    float gate;
    int scale_n;                    /* degrees per octave of the synth's scale */
    /* state */
    uint32_t rng;
    int playing, have_step;
    uint32_t step;                  /* the last tick's step (32nds since start) */
    uint32_t next_at, off_at;       /* the next chord step / gate end, in 32nds */
    int func;                       /* the current function degree 1..7 */
    int pos;                        /* chord step within the phrase */
    int chord[HARM_VOICES];         /* degrees (0 .. 4 octaves) per voice */
    int sounding[HARM_VOICES];      /* the note each voice plays (-1 none) */
    int16_t melody[HARM_MAX_MELODY];
    int mlen, midx;
    int fresh;                      /* the first chord after a start */
    uint32_t chords, cadences;      /* counters (status, tests) */
    /* output: frames due now */
    struct harm_frame out[4];
    int nout;
};

void harm_init(struct harmony* h, uint32_t seed);
void harm_rx(struct harmony* h, const struct smp_event* e);  /* ticks, transport, parameters */
int harm_pop(struct harmony* h, struct harm_frame* f);        /* 1 while frames are waiting */
void harm_new_melody(struct harmony* h, uint32_t seed);
/* one chord step by hand (tests): the next function and chord */
void harm_step(struct harmony* h);

#endif
