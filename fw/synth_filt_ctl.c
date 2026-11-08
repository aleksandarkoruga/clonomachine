// SPDX-License-Identifier: AGPL-3.0-or-later
/* The filter engine's coefficients (rtl/synth_filt.v running scripts/synth_asm.py's programs) from
 * cutoff, resonance and drive. The formulas follow model/synth_qmf.h, derived from QuadMorphFilter
 * (https://github.com/OTODESK4193/QuadMorphFilter) by OTODESK4193, GNU AGPL v3; this file is under
 * the AGPL-3.0 as well (the rest of this repository is MIT, see LICENSE), and so is a firmware
 * binary built with it. Fixed point Q8.24 like the engine's registers; no hardware here. */
#include "synth_ctl.h"
#include "synth_ucode.h"
#include "dmath.h"

#define ONE (1 << 24)

static int32_t mul24(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> 24); }
static int32_t div24(int32_t a, int32_t b) { return b ? (int32_t)(((int64_t)a << 24) / b) : 0x7FFFFFFF; }
static int32_t clamp(int32_t x, int32_t lo, int32_t hi) { return x < lo ? lo : x > hi ? hi : x; }

int sy_filt_start(int model) {
    return model == 1 ? SYU_MOOG_START : model == 2 ? SYU_TB303_START : SYU_SVF_START;
}

/* the ladders' bypass of their gain control near self-oscillation */
static int32_t bypass(int32_t kn) { return kn > ONE * 85 / 100 ? clamp((kn - ONE * 85 / 100) * 10, 0, ONE) : 0; }

int sy_filt_coefs(const struct sy_fslot* f, uint32_t fs2_q16, uint8_t* reg, int32_t* val) {
    const int32_t g = f->g_q24, res = f->res_q16 << 8, drive = f->drive_q16 << 8;
    int n = 0;
#define PUT(r, v) (reg[n] = (uint8_t)(r), val[n] = (v), ++n)
    if (f->model == 0) { /* clean SVF; the type as output coefficients */
        const int32_t R = div24(ONE, 2 * res);
        PUT(SYU_SVF_comp, mul24(ONE + res / 10, drive));
        PUT(SYU_SVF_k1, 2 * R + g);
        PUT(SYU_SVF_g, g);
        PUT(SYU_SVF_h, div24(ONE, ONE + 2 * mul24(R, g) + mul24(g, g)));
        PUT(SYU_SVF_cl, (f->type == 0 || f->type == 3) ? ONE : 0);
        PUT(SYU_SVF_cb, f->type == 1 ? ONE : 0);
        PUT(SYU_SVF_ch, (f->type == 2 || f->type == 3) ? ONE : 0);
        return n;
    }
    const int32_t G = div24(g, ONE + g), G2 = mul24(G, G), G4 = mul24(G2, G2);
    const int32_t r01 = res - ONE / 10;
    if (f->model == 2) { /* TB-303 (diode ladder) */
        const int32_t k = clamp(mul24(r01, (int32_t)(ONE * (4.2 / 9.9))), 0, (int32_t)(ONE * 4.2));
        const int32_t kn = clamp(mul24(r01, (int32_t)(ONE / 9.9)), 0, ONE);
        /* the high-pass in the feedback at 8 Hz: tan(pi 8 / fs) ~ pi 8 / fs */
        const int32_t hgt = (int32_t)(((int64_t)(3.14159265 * 8 * 65536) << 24) / fs2_q16);
        PUT(SYU_TB303_comp, mul24(ONE + k / 5, drive));
        PUT(SYU_TB303_inv1pg, div24(ONE, ONE + g));
        PUT(SYU_TB303_G, G);
        PUT(SYU_TB303_G4, G4);
        PUT(SYU_TB303_k, k);
        PUT(SYU_TB303_invn, div24(ONE, ONE + mul24(k, G4)));
        PUT(SYU_TB303_hg, div24(hgt, ONE + hgt));
        PUT(SYU_TB303_byp, bypass(kn));
        return n;
    }
    { /* Moog ladder */
        const int32_t k = mul24(r01, (int32_t)(ONE * (4.5 / 9.9)));
        const int32_t inv = div24(ONE, ONE + mul24(k, G4)), kg4 = mul24(k, G4);
        PUT(SYU_MOOG_comp, mul24(ONE + k / 2, drive));
        PUT(SYU_MOOG_inv1pg, div24(ONE, ONE + g));
        PUT(SYU_MOOG_G, G);
        PUT(SYU_MOOG_G2, G2);
        PUT(SYU_MOOG_G3, mul24(G2, G));
        PUT(SYU_MOOG_k, k);
        PUT(SYU_MOOG_inv, inv);
        PUT(SYU_MOOG_kg4, kg4);
        PUT(SYU_MOOG_kg4i, mul24(kg4, inv));
        PUT(SYU_MOOG_byp, bypass(mul24(k, (int32_t)(ONE / 4.5))));
        return n;
    }
#undef PUT
}

/* a slot's state at a note's start / a model change: zero, the ladders' gain control at 1 */
int sy_filt_state_regs(int model, uint8_t* reg, int32_t* val) {
    const int first = model == 0 ? SYU_SVF_ch : model == 1 ? SYU_MOOG_byp : SYU_TB303_byp;
    int n = 0;
    for (int r = 0; r < first; ++r, ++n) {
        reg[n] = (uint8_t)r;
        val[n] = 0;
    }
    if (model == 1)
        val[SYU_MOOG_agc] = ONE;
    if (model == 2)
        val[SYU_TB303_agc] = ONE;
    return n;
}

/* the shared constants (r32..r63): the gain control's meter / smoothing coefficients at the
   filter rate fs2 (twice the sample rate) */
void sy_filt_constants(uint32_t fs2_q16, uint8_t* reg, int32_t* val, int* n) {
    const double fs2 = fs2_q16 / 65536.0;
    const double cm = 1 - d_exp(-1 / (0.0045 * fs2)), ca = 1 - d_exp(-1 / (0.045 * fs2));
    const struct { int r; double v; } c[] = {
        {SYU_C_ZERO, 0}, {SYU_C_ONE, 1}, {SYU_C_TWO, 2}, {SYU_C_HALF, 0.5}, {SYU_C_EPS, 1e-6},
        {SYU_C_GMIN, 0.1}, {SYU_C_GMAX, 15}, {SYU_C_CM, cm}, {SYU_C_CA, ca}, {SYU_C_C95, 0.95}};
    *n = 0;
    for (unsigned i = 0; i < sizeof(c) / sizeof(c[0]); ++i) {
        reg[*n] = (uint8_t)c[i].r;
        val[*n] = (int32_t)(c[i].v * ONE + (c[i].v >= 0 ? 0.5 : -0.5));
        ++*n;
    }
}
