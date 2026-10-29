/* The plate reverb's control (the PMWave UGen as a 2x2 plate, rtl/reverb_unit.v). Every
 * 256 samples the soft core works out, for each of the 16 rails, the length it glides to by the
 * end of the next block (glide and wander as in the UGen, in log2 of samples) and its loop filter
 * (gain, pole) at that length. The unit ramps the lengths linearly over the block and computes the
 * interpolation taps itself. Integer arithmetic per block (no FPU); the tables are rebuilt in
 * double when the parameters change. No hardware here: model/reverb_model + host/test_reverb
 * run it on the PC. */
#ifndef REVERB_CTL_H
#define REVERB_CTL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RV_RAILS 16
#define RV_BLOCK_LOG2 8 /* 256 samples per control block */

struct rv_params {          /* the UGen's inputs, Q16 */
    int32_t decay;          /* T60 at DC, s                         */
    int32_t time;           /* size 0..1: 0 = 40 ms rails, 1 = 0.1 ms */
    int32_t glide;          /* s                                    */
    int32_t hf_decay;       /* T60 at Nyquist, s                    */
    int32_t rand1;          /* rail length spread, ratio            */
    int32_t rand2;          /* glide time spread, ratio             */
    int32_t wander;         /* slow length modulation depth, ratio  */
    int glide_mode;         /* 0 lag, 1 constant rate               */
};

struct rv_regs {            /* what the unit takes for the next block */
    uint32_t target[RV_RAILS]; /* rail length at the block's end, samples Q20 */
    int32_t gain[RV_RAILS];    /* loop filter y = gain x + pole y[n-1], Q1.17 */
    int32_t pole[RV_RAILS];
};

struct rv_ctl {
    uint32_t fs, rng;
    struct rv_params p;
    int32_t log_min, log_max;                 /* log2(samples), Q16 */
    int32_t rand1[RV_RAILS], rand2[RV_RAILS]; /* -0.5 .. 0.5, Q16  */
    int32_t log_target[RV_RAILS], remain[RV_RAILS];
    int32_t lag[RV_RAILS];                    /* per block: lag mode remaining fraction Q30, rate mode octaves Q16 */
    uint32_t w_phase[RV_RAILS], w_inc[RV_RAILS];
    int32_t w_from[RV_RAILS], w_to[RV_RAILS]; /* -1 .. 1, Q16 */
    int32_t spread1, wander_oct;              /* Q16 */
    uint32_t len[RV_RAILS];                   /* last target, Q20 */
    int32_t gain_tab[257], pole_tab[257];     /* over log2 length, Q1.17 */
    uint32_t exp_tab[257];                    /* 2^(k/256), Q30 */
};

void rv_defaults(struct rv_params* p); /* the UGen's defaults */
void rv_init(struct rv_ctl* c, uint32_t fs, uint32_t seed, const struct rv_params* p);
void rv_set(struct rv_ctl* c, const struct rv_params* p);
void rv_block(struct rv_ctl* c, struct rv_regs* out); /* advances one block */

#ifdef __cplusplus
}
#endif

#endif
