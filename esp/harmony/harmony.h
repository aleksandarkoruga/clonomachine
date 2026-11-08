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
 * Modes (harmony): 1 play, EV_SYN_NOTE (scale degrees: the synth maps them through its scale
 * and tuning) for the voices whose note changes; 2 offset, the chord's root (in degrees, within
 * half an octave of the key's root, + harmOctave) goes to the synth as EV_SYN_HOFS and is added
 * to its MIDI notes like noteOffset. The next chord's offset is sent a step ahead and the FPGA
 * applies it on the step itself, so the notes of that step already get it.
 *
 * Time: harmTrig 0, the FPGA's 32nd ticks (EV_TICK, while the transport plays), a chord step
 * every harmBeats beats; harmTrig 1..16, a step on each EV_SYN_TRIG (the FPGA counts the note-ons
 * on that channel, every harmBeats-th is one), the ticks only time harmGate (of the last trig
 * interval). Parameters: EV_SYN_PARAM with the harm* ids of include/synth_params.h (and scale, for
 * its size); the FPGA repeats them every second, so they act only when they change. */
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
    int mode, beats, phrase, octave, voices, melody_on, vel, seed; /* mode 0 off, 1 play, 2 offset */
    int trig;                       /* 0 the beat, 1..16 the FPGA's trigs step it */
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
    int root;                       /* the chord's root degree (function + melody) */
    int ahead;                      /* offset mode: the next chord's offset is out */
    uint32_t last_trig, trig_len;   /* trig mode: the last trig's step, the interval (32nds) */
    int have_trig;
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
int harm_offset(const struct harmony* h);                     /* offset mode: the chord's offset */
void harm_new_melody(struct harmony* h, uint32_t seed);
/* one chord step by hand (tests): the next function and chord */
void harm_step(struct harmony* h);

#endif
