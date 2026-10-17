/* Sampler slots: one BufGrainPlay parameter set per slot (no modes: OLA time-stretch
 * and granular are just parameter values), the SC drum voice's gate/stutter contract,
 * tempo logic and the conversion to engine registers. */
#ifndef SLOT_H
#define SLOT_H

#include <stdint.h>
#include <generated/soc.h>
#ifndef SAMPLER_N_SLOTS
#define SAMPLER_N_SLOTS 2
#endif

#define NUM_SLOTS SAMPLER_N_SLOTS

struct slot {
    int lib;                 /* library = folder (0 /drums, 1 /melody, ...)             */
    int bank, buf;           /* playing: bank, index in bank (-1: none)                 */
    int want_bank, want_buf; /* requested; switches as soon as the sample is in SDRAM   */
    int cc1;                 /* last CC1 (buffer index inside the bank)                 */
    /* BufGrainPlay arguments */
    uint32_t pos_q30;        /* 0..1 of buffer: start position latched on trig          */
    int pos_pin;             /* pos >= 0 given via /pos: re-pin every sample (UGen)     */
    int32_t rate_q16, pitch_q16;
    int32_t graindur_q16;    /* seconds                                                  */
    int32_t overlap_q16, prob_q16, separation_q16, randpos_q16, randompitch_q16;
    /* engine extras */
    int tempo_lock, max_grains, enabled;
    int bg_advance;          /* trig/pin position = pos + background phasor (keeps playing) */
    int algo;                /* 0 grains, 1 phase vocoder (switches live, crossfaded)    */
    int pv_transient;        /* phase vocoder transient handling on                     */
    int note_mode;           /* note -> pos: 0 equal 16ths, 1 k-th transient, 2 16ths snapped to transients */
    int out;                 /* output: 0 left, 1 right, 2 both                          */
    uint32_t seed;
    /* drum voice gate / stutter / output */
    int env_abs;             /* 0: env_bars at the global tempo, 1: env_sec             */
    int32_t env_bars_q16, env_sec_q16;
    int env_hold;
    int repeats;
    int32_t interval_q16;    /* stutter period, seconds                                  */
    int32_t amp_q16, algo_q16;
};

extern struct slot g_slots[NUM_SLOTS];
extern uint32_t g_bpm_milli;           /* global tempo, bpm * 1000 */

void slots_init(void);
void slot_select(int s, int idx);      /* buffer inside the current/wanted bank (CC1, bufNum) */
void slot_select_bank(int s, int bank);/* bank of the slot's library (PC, /bank)              */
void slots_service(void);              /* main loop: switch to loaded buffers, prefetch      */

void slot_apply(int s);                /* every engine register of the slot */
void slot_apply_tempo(int s);          /* bpm, tempo lock, rate, pitch, gate length */
void slot_apply_grain(int s);
void slot_apply_output(int s);
void slot_apply_flags(int s);
void slot_apply_algo(int s);

void slot_set_pos(int s, int32_t pos_q30); /* UGen pos: >= 0 pins, < 0 runs free */
void slot_trig(int s);                 /* POS + stutter params + TRIG strobe */
void slot_note_on(int s, int note, int vel);
void slot_note_off(int s);             /* ends the gate */
void set_global_bpm(uint32_t bpm_milli);

uint32_t slot_tempo_ratio_q24(int s);
uint32_t slot_sixteenths(int s);
uint64_t bar_samples_q16(void);

#endif
