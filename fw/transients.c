#include "transients.h"

struct cand {
    uint32_t block;
    int32_t rise, level;
};

static int32_t log2q8(uint32_t v) {
    int e = 31;
    while (e > 0 && !(v >> e))
        --e;
    const uint32_t m = e >= 8 ? (v >> (e - 8)) & 255u : (v << (8 - e)) & 255u;
    return (int32_t)(((uint32_t)e << 8) | m);
}

static int16_t absv(int16_t v) { return v < 0 ? (v == -32768 ? 32767 : (int16_t)-v) : v; }

/* first sample of the attack around block b, moved back to a zero crossing */
static uint32_t refine(tr_get_fn get, void* ctx, uint32_t frames, uint32_t b) {
    const uint32_t s0 = b * TR_BLOCK, s1 = s0 + TR_BLOCK < frames ? s0 + TR_BLOCK : frames;
    int16_t pk = 1;
    for (uint32_t i = s0; i < s1; ++i) {
        const int16_t a = absv(get(ctx, i));
        if (a > pk)
            pk = a;
    }
    uint32_t i = s0 >= TR_BLOCK ? s0 - TR_BLOCK : 0;
    while (i < s1 && absv(get(ctx, i)) < pk / 4)
        ++i;
    for (uint32_t k = 0; k < 2 * TR_BLOCK && i > 0; ++k, --i) {
        const int16_t a = get(ctx, i), p = get(ctx, i - 1);
        if (a == 0 || (a < 0) != (p < 0))
            break;
    }
    return i;
}

int tr_analyse(tr_get_fn get, void* ctx, uint32_t frames, uint32_t* pos, void (*idle)(void)) {
    struct cand c[TR_MAX * 2];
    int nc = 0;
    const uint32_t nb = frames / TR_BLOCK;
    if (nb < 4)
        return 0;
    int32_t hist[3] = {0, 0, 0}, lmax = 0, l0 = 0;
    int16_t prev = get(ctx, 0);
    uint32_t last = 0;
    int have_last = 0;
    for (uint32_t b = 0; b < nb; ++b) {
        if (idle && !(b & 127))
            idle();
        uint32_t e = 0;
        for (uint32_t i = b * TR_BLOCK; i < (b + 1) * TR_BLOCK; ++i) {
            const int16_t v = get(ctx, i);
            const int32_t d = (int32_t)v - prev;
            e += (uint32_t)(d < 0 ? -d : d);
            prev = v;
        }
        const int32_t l = log2q8(e + 16);
        if (b == 0)
            hist[0] = hist[1] = hist[2] = l0 = l;
        if (l > lmax)
            lmax = l;
        int32_t mn = hist[0];
        if (hist[1] < mn) mn = hist[1];
        if (hist[2] < mn) mn = hist[2];
        const int32_t rise = l - mn;
        hist[b % 3] = l;
        if (b == 0 || rise < TR_RISE || b + 8 >= nb)   /* not the loop wrapping onto its start */
            continue;
        if (have_last && b - last < TR_REFRACT) {
            /* same event: keep the stronger rise */
            if (rise > c[nc - 1].rise) {
                c[nc - 1].block = b;
                c[nc - 1].rise = rise;
                c[nc - 1].level = l;
                last = b;
            }
            continue;
        }
        if (nc == TR_MAX * 2) {
            /* full: replace the weakest candidate (order restored below) */
            int w = 0;
            for (int k = 1; k < nc; ++k)
                if (c[k].rise < c[w].rise)
                    w = k;
            if (rise <= c[w].rise)
                continue;
            for (int k = w; k < nc - 1; ++k)
                c[k] = c[k + 1];
            --nc;
        }
        c[nc].block = b;
        c[nc].rise = rise;
        c[nc].level = l;
        ++nc;
        last = b;
        have_last = 1;
    }
    /* level filter, then the TR_MAX strongest */
    int n = 0;
    for (int k = 0; k < nc; ++k)
        if (c[k].level >= lmax - TR_RANGE)
            c[n++] = c[k];
    while (n > TR_MAX) {
        int w = 0;
        for (int k = 1; k < n; ++k)
            if (c[k].rise < c[w].rise)
                w = k;
        for (int k = w; k < n - 1; ++k)
            c[k] = c[k + 1];
        --n;
    }
    int out = 0;
    /* the start counts when it begins loud (most loops start on a hit) */
    if (l0 >= lmax - TR_RANGE && (n == 0 || c[0].block > TR_REFRACT))
        pos[out++] = 0;
    for (int k = 0; k < n && out < TR_MAX; ++k) {
        const uint32_t p = refine(get, ctx, frames, c[k].block);
        if (out == 0 || p > pos[out - 1])
            pos[out++] = p;
    }
    return out;
}
