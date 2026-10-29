/* The plate reverb's control, see reverb_ctl.h. Follows PMWave.hpp / PMWave.cpp step by step,
 * with the per-sample glide and wander advanced a whole block at a time. */
#include "reverb_ctl.h"

#include <string.h>

#include "dmath.h"

#define BLOCK (1 << RV_BLOCK_LOG2)
#define Q16(x) ((int32_t)((x) * 65536.0 + ((x) >= 0 ? 0.5 : -0.5)))
#define LN1000 6.907755278982137 /* 60 dB */

static double d_log2(double x) { return d_log(x) / 0.69314718055994530942; }
static double d_exp2(double x) { return d_exp(x * 0.69314718055994530942); }
static double q16d(int32_t v) { return v / 65536.0; }

static uint32_t rng(struct rv_ctl* c) {
    uint32_t x = c->rng ? c->rng : 1;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return c->rng = x;
}
static int32_t frand(struct rv_ctl* c) { return (int32_t)(rng(c) >> 16); }               /* 0 .. 1, Q16 */
static int32_t frand2(struct rv_ctl* c) { return (int32_t)(rng(c) >> 15) - 65536; }      /* -1 .. 1, Q16 */
static int32_t clampi(int32_t x, int32_t lo, int32_t hi) { return x < lo ? lo : x > hi ? hi : x; }

void rv_defaults(struct rv_params* p) {
    p->decay = Q16(4.0);
    p->time = Q16(0.3);
    p->glide = Q16(0.2);
    p->hf_decay = Q16(1.0);
    p->rand1 = Q16(0.1);
    p->rand2 = Q16(0.2);
    p->wander = 0;
    p->glide_mode = 0;
}

/* 2^x, x log2(samples) Q16 -> samples Q20 */
static uint32_t exp2_q20(const struct rv_ctl* c, int32_t x) {
    const int32_t i = x >> 16;
    const uint32_t f = (uint32_t)x & 0xFFFF, k = f >> 8, fr = f & 0xFF;
    const uint32_t m = c->exp_tab[k] + (uint32_t)(((uint64_t)(c->exp_tab[k + 1] - c->exp_tab[k]) * fr) >> 8); /* Q30 */
    return i <= 10 ? m >> (10 - i) : m << (i - 10);
}

static void set_targets(struct rv_ctl* c) {
    const int32_t log_time = c->log_max + (int32_t)(((int64_t)clampi(c->p.time, 0, 65536) * (c->log_min - c->log_max)) >> 16);
    for (int r = 0; r < RV_RAILS; ++r) {
        const int32_t t = clampi(log_time + (int32_t)(((int64_t)c->spread1 * c->rand1[r]) >> 16), c->log_min, c->log_max);
        c->remain[r] += t - c->log_target[r]; /* the lagged length stays where it is */
        c->log_target[r] = t;
    }
}

static void set_lag(struct rv_ctl* c) {
    const double spread = 2 * d_log2(1 + (c->p.rand2 > 0 ? q16d(c->p.rand2) : 0));
    const double glide = c->p.glide > 0 ? q16d(c->p.glide) : 0;
    for (int r = 0; r < RV_RAILS; ++r) {
        const double seconds = glide * d_exp2(spread * q16d(c->rand2[r]));
        if (c->p.glide_mode) /* octaves per block */
            c->lag[r] = seconds > 0 ? Q16(BLOCK / (seconds * c->fs)) : 0x7FFFFFFF;
        else                 /* what's left of the distance after a block: (1 - coef)^256 */
            c->lag[r] = seconds > 0 ? (int32_t)(d_exp(-LN1000 * BLOCK / (seconds * c->fs)) * 1073741824.0) : 0;
    }
}

/* Jot: a rail D samples long gets g(1-b)/(1-b z^-1) with |H(0)| = 10^(-3D/(fs decay)) and
   |H(pi)| = 10^(-3D/(fs hfDecay)), so T60 doesn't depend on the plate's size */
static void set_loss(struct rv_ctl* c) {
    const double dec = q16d(c->p.decay), hf = q16d(c->p.hf_decay);
    const double kLo = -LN1000 / (c->fs * (dec < 0.01 ? 0.01 : dec > 1000 ? 1000 : dec));
    const double kHi = -LN1000 / (c->fs * (hf < 0.001 ? 0.001 : hf > 1000 ? 1000 : hf));
    const double lmin = q16d(c->log_min), lmax = q16d(c->log_max);
    for (int i = 0; i <= 256; ++i) {
        const double D = d_exp2(lmin + i * (lmax - lmin) / 256);
        const double rr = d_exp((kHi - kLo) * D), b = (1 - rr) / (1 + rr);
        c->gain_tab[i] = (int32_t)(d_exp(kLo * D) * (1 - b) * 131072.0 + 0.5);
        c->pole_tab[i] = (int32_t)(b * 131072.0 + 0.5);
    }
}

void rv_set(struct rv_ctl* c, const struct rv_params* p) {
    const struct rv_params old = c->p;
    c->p = *p;
    c->spread1 = Q16(2 * d_log2(1 + (p->rand1 > 0 ? q16d(p->rand1) : 0)));
    c->wander_oct = Q16(d_log2(1 + (p->wander > 0 ? q16d(p->wander) : 0)));
    if (p->time != old.time || p->rand1 != old.rand1)
        set_targets(c);
    if (p->glide != old.glide || p->rand2 != old.rand2 || p->glide_mode != old.glide_mode)
        set_lag(c);
    if (p->decay != old.decay || p->hf_decay != old.hf_decay)
        set_loss(c);
}

static int32_t wander_value(const struct rv_ctl* c, int r) {
    const int64_t ph = c->w_phase[r] >> 16; /* Q16 */
    const int64_t s = (ph * ph >> 16) * (3 * 65536 - 2 * ph) >> 16;
    return c->w_from[r] + (int32_t)(((int64_t)(c->w_to[r] - c->w_from[r]) * s) >> 16);
}

void rv_init(struct rv_ctl* c, uint32_t fs, uint32_t seed, const struct rv_params* p) {
    memset(c, 0, sizeof(*c));
    c->fs = fs;
    c->rng = seed ? seed : 1;
    for (int k = 0; k <= 256; ++k)
        c->exp_tab[k] = (uint32_t)(d_exp2(k / 256.0) * 1073741824.0 + 0.5);
    const double longest = 0.040 * fs, shortest = 0.0001 * fs < 4 ? 4 : 0.0001 * fs;
    c->log_max = Q16(d_log2(longest));
    c->log_min = Q16(d_log2(shortest));
    for (int r = 0; r < RV_RAILS; ++r) {
        c->rand1[r] = frand(c) - 32768;
        c->rand2[r] = frand(c) - 32768;
        c->w_phase[r] = rng(c);
        const double rate = 0.1 + 0.3 * (frand(c) / 65536.0); /* Hz */
        c->w_inc[r] = (uint32_t)(rate * BLOCK / fs * 4294967296.0);
        c->w_from[r] = frand2(c);
        c->w_to[r] = frand2(c);
    }
    c->p = *p;
    c->p.time = ~p->time; /* everything counts as changed */
    c->p.glide = ~p->glide;
    c->p.decay = ~p->decay;
    rv_set(c, p);
    for (int r = 0; r < RV_RAILS; ++r) { /* the plate starts at its size */
        c->remain[r] = 0;
        c->len[r] = exp2_q20(c, clampi(c->log_target[r] + (int32_t)(((int64_t)c->wander_oct * wander_value(c, r)) >> 16),
                                       c->log_min, c->log_max));
    }
}

void rv_block(struct rv_ctl* c, struct rv_regs* out) {
    const uint32_t lo = exp2_q20(c, c->log_min), hi = exp2_q20(c, c->log_max);
    const uint32_t slew = (uint32_t)BLOCK << 20; /* a rail moves at most a sample per sample */
    for (int r = 0; r < RV_RAILS; ++r) {
        int32_t rem = c->remain[r];
        if (rem) {
            if (c->p.glide_mode) {
                const int32_t st = c->lag[r];
                rem = rem > st ? rem - st : rem < -st ? rem + st : 0;
            } else {
                rem = (int32_t)(((int64_t)rem * c->lag[r]) >> 30);
                if (rem > -2 && rem < 2)
                    rem = 0;
            }
            c->remain[r] = rem;
        }
        int32_t offset = -rem;
        if (c->wander_oct > 0) {
            const uint32_t ph = c->w_phase[r] + c->w_inc[r];
            if (ph < c->w_phase[r]) { /* a new random target */
                c->w_from[r] = c->w_to[r];
                c->w_to[r] = frand2(c);
            }
            c->w_phase[r] = ph;
            offset += (int32_t)(((int64_t)c->wander_oct * wander_value(c, r)) >> 16);
        }
        const int32_t lr = clampi(c->log_target[r] + offset, c->log_min, c->log_max);
        uint32_t d = exp2_q20(c, lr);
        d = d < lo ? lo : d > hi ? hi : d;
        const uint32_t prev = c->len[r];
        if (d > prev + slew)
            d = prev + slew;
        else if (prev > slew && d < prev - slew)
            d = prev - slew;
        c->len[r] = out->target[r] = d;
        /* loss for the length read: table over log2 length, interpolated */
        const int64_t x = ((int64_t)(lr - c->log_min) << 8) * 65536 / (c->log_max - c->log_min); /* Q16 index */
        int32_t i = (int32_t)(x >> 16);
        int32_t f = (int32_t)(x & 0xFFFF);
        if (i >= 256) {
            i = 255;
            f = 65536;
        }
        out->gain[r] = c->gain_tab[i] + (int32_t)(((int64_t)(c->gain_tab[i + 1] - c->gain_tab[i]) * f) >> 16);
        out->pole[r] = c->pole_tab[i] + (int32_t)(((int64_t)(c->pole_tab[i + 1] - c->pole_tab[i]) * f) >> 16);
    }
}
