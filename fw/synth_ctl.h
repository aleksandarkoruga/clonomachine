/* The wavetable synth's control (rtl/synth_top.v): notes and voices, envelopes, LFOs, the mod
 * matrix, and every SY_TICK samples the registers each voice needs (oscillator increments,
 * table positions and levels, warps, filter coefficients, mixer weights and gains). Integer
 * arithmetic per tick (no FPU); tables are built in double at init and when a parameter
 * changes. No hardware here: the register writes go through a callback (fw/synth.c writes the
 * CSRs, sim/tb_synth_fw.cpp feeds them to the RTL in Verilator). */
#ifndef SYNTH_CTL_H
#define SYNTH_CTL_H

#include <stdint.h>

#include "../include/synth_params.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SY_VOICES     3
#define SY_TICK       64      /* samples per control tick (1.33 ms) */
#define SY_MAX_TABLES 32
#define SY_LEVELS     10      /* mip levels 1..9 are stored (level 0, 2048 samples, isn't played) */

/* a wavetable in SDRAM (fw/synth_wt.h builds them) */
struct sy_table {
    char name[28];
    uint16_t frames;               /* 1..256 */
    uint32_t lvl[SY_LEVELS];       /* SDRAM word address of level k's frame 0, k = 1..9 */
};

typedef void (*sy_write_fn)(uint32_t addr, uint32_t data);

struct sy_env {
    uint8_t st;
    int32_t v;                     /* Q24 */
    uint32_t t;                    /* ticks in the stage */
};
struct sy_lfo {
    uint32_t ph, age;
    int32_t held;                  /* S&H, Q16 */
    uint8_t done;
};
struct sy_voice {
    uint8_t active, gate, degree;  /* degree: note is a scale degree + 64 (the harmony generator) */
    int note;
    int32_t vel, rnd;              /* Q16 */
    uint32_t age;                  /* ticks since the note started (stealing) */
    int32_t pitch, target;         /* semitones Q16 */
    struct sy_env env[3];
    struct sy_lfo lfo[4];
    int lev[2];                    /* mip level per oscillator (0: choose afresh) */
    int idle_written;
    int32_t fkey[4][4];            /* each filter slot's last inputs (coefficients only when they move) */
};

struct sy_envp { int32_t atk_step, sus; uint32_t hold, dec_ticks, rel_ticks; int32_t dec_k, rel_k; };

struct sy_state {
    int32_t knob[SP_NUM];          /* Q16, 0..65536 */
    struct sy_voice v[SY_VOICES];
    struct sy_table tab[SY_MAX_TABLES]; /* all resident (sy_set_tables): aTable / bTable index these */
    int ntab;
    const struct sy_table* osc_tab[2];  /* what oscillators A and B play (0: silent) */
    uint32_t fs_q16, bpm_milli, rng;
    int32_t bend;                  /* -8192 .. 8191 */
    sy_write_fn wr;
    /* derived (sy_param_changed) */
    struct sy_envp envp[3];
    uint32_t lfo_inc[4], lfo_rise[4], lfo_delay[4];   /* per tick / ticks */
    int32_t porta_k;               /* Q30, 0: no glide */
    uint8_t dst_param[24];         /* mod destination index -> parameter */
    int ndst;
    int slot_model[4];             /* what the filter threads run */
    uint32_t inc_a4;               /* phase increment of 440 Hz per sub-sample */
    uint32_t limit_inc;            /* the mip level limit (43.2 kHz) as an increment */
    int32_t ticks;
    /* register shadow: written only when the value changes */
    uint32_t shadow[8192];
    uint8_t valid[8192 / 8];
    uint32_t writes;
};

void sy_init(struct sy_state* s, uint32_t fs_q16, sy_write_fn wr);
void sy_hw_reset(struct sy_state* s);              /* constants, programs, everything rewritten */
void sy_set_tables(struct sy_state* s, const struct sy_table* t, int n); /* every table resident */
void sy_use_table(struct sy_state* s, int osc, const struct sy_table* t); /* or one at a time (fw/synth.c) */
void sy_set_knob(struct sy_state* s, int p, int32_t knob_q16);
void sy_set_cc(struct sy_state* s, int p, int v127);
void sy_set_natural(struct sy_state* s, int p, int32_t x_q16);   /* OSC: natural units x 2^16 */
int32_t sy_natural_q16(const struct sy_state* s, int p);
void sy_set_bpm(struct sy_state* s, uint32_t bpm_milli);
void sy_note_on(struct sy_state* s, int note, int vel127, int degree);
void sy_note_off(struct sy_state* s, int note, int degree);
void sy_all_off(struct sy_state* s);
void sy_bend(struct sy_state* s, int v14);
void sy_tick(struct sy_state* s);                  /* one control tick */
int sy_param_of(const char* name);                 /* -1 if unknown (case-insensitive) */
const char* sy_param_name(int p);
const char* sy_source_name(int src);
int sy_active_voices(const struct sy_state* s);
/* the patch at power-up: a reese bass (every parameter but the tables: A a saw stack, B a table
   to drift through; fw/synth.c picks them by name) */
void sy_patch_reese(struct sy_state* s);

/* fw/synth_filt_ctl.c (AGPL: the coefficient formulas of the quad morph filter's models) */
struct sy_fslot { int model, type; int32_t g_q24, res_q16, drive_q16; };
int sy_filt_coefs(const struct sy_fslot* f, uint32_t fs2_q16, uint8_t* reg, int32_t* val); /* -> count */
int sy_filt_state_regs(int model, uint8_t* reg, int32_t* val);                             /* reset values */
int sy_filt_start(int model);
void sy_filt_constants(uint32_t fs2_q16, uint8_t* reg, int32_t* val, int* n);

#ifdef __cplusplus
}
#endif

#endif
