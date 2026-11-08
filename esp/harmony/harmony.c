/* The harmony generator, see harmony.h. */
#include "harmony.h"

#include <string.h>

#include "../../fw/scales.h"
#include "../../include/synth_params.h"

/* the function chain: weights of the steps from each degree (the SC version's Pfsm, as counts) */
static const uint8_t W[8][8] = {
    {0},
    /*        1   2   3   4   5   6  7 */
    [1] = {0, 18, 4,  3,  6,  2,  3, 1},
    [2] = {0, 0,  20, 0,  6,  4,  0, 1},
    [3] = {0, 0,  6,  17, 5,  3,  3, 2},
    [4] = {0, 0,  0,  0,  17, 5,  0, 1},
    [5] = {0, 12, 0,  0,  0,  25, 5, 9},
    [6] = {0, 0,  10, 0,  11, 1,  13, 1},
    [7] = {0, 18, 0,  4,  0,  0,  4, 0},
};

static uint32_t rnd(struct harmony* h) {
    uint32_t x = h->rng;
    x ^= x << 13, x ^= x >> 17, x ^= x << 5;
    return h->rng = x;
}
static int rndn(struct harmony* h, int n) { return n > 0 ? (int)(rnd(h) % (uint32_t)n) : 0; }

/* ---- melody --------------------------------------------------------------------------------- */
/* a piece of a[0..n) transformed, appended at a[n..]; returns the new length */
static int grow(struct harmony* h, int16_t* a, int n, int maxd) {
    if (n < 2)
        return n;
    int i0 = 1 + rndn(h, n - 1), i1 = (i0 + 1 + rndn(h, n - 1)) % n;
    i0 %= n;
    if (i0 > i1) {
        const int t = i0;
        i0 = i1;
        i1 = t;
    }
    const int len = i1 - i0 + 1;
    if (n + len > HARM_MAX_MELODY)
        return n;
    int16_t* s = a + i0;
    int16_t* r = a + n;
    switch (rndn(h, 7)) {
    case 0: /* inversion around the first note */
        for (int k = 0; k < len; ++k)
            r[k] = (int16_t)(((2 * s[0] - s[k]) % maxd + maxd) % maxd);
        break;
    case 1: { /* the second differences inverted (differentiate twice, negate, integrate twice) */
        int i1 = 0, i2 = 0, pd1 = 0;
        for (int k = 0; k < len; ++k) {
            const int d1 = k ? s[k] - s[k - 1] : s[0];
            const int d2 = k ? d1 - pd1 : d1;
            pd1 = d1;
            i1 += k ? -d2 : d2;
            i2 += i1;
            r[k] = (int16_t)((i2 % maxd + maxd) % maxd);
        }
        break;
    }
    case 2: { /* transposed */
        const int t = 1 + rndn(h, maxd - 1);
        for (int k = 0; k < len; ++k)
            r[k] = (int16_t)((s[k] + t) % maxd);
        break;
    }
    case 3: /* retrograde */
        for (int k = 0; k < len; ++k)
            r[k] = s[len - 1 - k];
        break;
    case 4: { /* perfect shuffle: the halves interleaved */
        const int half = (len + 1) / 2;
        for (int k = 0, a2 = 0, b2 = half; k < len; ++k)
            r[k] = (k & 1) ? s[b2++] : s[a2++];
        break;
    }
    case 5: { /* rotated */
        const int rot = rndn(h, len);
        for (int k = 0; k < len; ++k)
            r[k] = s[(k + rot) % len];
        break;
    }
    default: memmove(r, s, (size_t)len * sizeof(*s)); break;
    }
    return n + len;
}

void harm_new_melody(struct harmony* h, uint32_t seed) {
    h->rng = seed ? seed : 0x2545F491u;
    for (int k = 0; k < 7; ++k) /* a seed of seven degrees */
        h->melody[k] = (int16_t)rndn(h, h->scale_n);
    int n = 7;
    for (int it = 0; it < 10; ++it)
        n = grow(h, h->melody, n, 4 * h->scale_n);
    h->mlen = n;
    h->midx = 0;
}

/* ---- functions and chords ------------------------------------------------------------------- */
static int pick(struct harmony* h, int from) {
    int sum = 0;
    for (int t = 1; t <= 7; ++t)
        sum += W[from][t];
    int r = rndn(h, sum);
    for (int t = 1; t <= 7; ++t) {
        if (r < W[from][t])
            return t;
        r -= W[from][t];
    }
    return from;
}

/* the next function: the chain, steered into a cadence at the end of the phrase */
static int next_function(struct harmony* h) {
    const int prev = h->func, P = h->phrase, at = h->pos;
    if (h->fresh)
        return 1;
    if (P >= 3 && at == P - 2 && W[prev][5]) /* the penultimate: the dominant */
        return 5;
    if (P >= 2 && at == P - 1) {             /* the last step */
        if (prev == 5 || prev == 7) {
            const int r = rndn(h, 100);
            ++h->cadences;
            if (r < 20 && W[prev][6])
                return 6;                    /* deceptive */
            if (r < 35 && prev == 5)
                return 5;                    /* half: rest on the dominant */
            return 1;                        /* authentic */
        }
        if (W[prev][5]) {
            ++h->cadences;
            return 5;                        /* half cadence */
        }
    }
    return pick(h, prev);
}

static void voice_chord(struct harmony* h, int func) {
    const int n = h->voices, maxd = 4 * h->scale_n, sn = h->scale_n;
    const int base = (func - 1) + (h->melody_on && h->mlen ? h->melody[h->midx] : 0);
    int nc[HARM_VOICES];
    for (int i = 0; i < n; ++i)
        nc[i] = (2 * i + base) % maxd;
    if (h->fresh) {
        memcpy(h->chord, nc, sizeof(nc));
        return;
    }
    /* every order of the new notes over the voices: the least movement without parallels */
    static const uint8_t P3[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
    static const uint8_t P2[2][3] = {{0, 1, 0}, {1, 0, 0}};
    const int np = n == 3 ? 6 : n == 2 ? 2 : 1;
    int best = -1, bestd = 1 << 30, any = 0, anyd = 1 << 30;
    for (int p = 0; p < np; ++p) {
        int perm[HARM_VOICES], d = 0;
        for (int i = 0; i < n; ++i) {
            perm[i] = nc[n == 3 ? P3[p][i] : n == 2 ? P2[p][i] : 0];
            const int m = h->chord[i] - perm[i];
            d += m < 0 ? -m : m;
        }
        int par = 0;
        for (int i = 0; i < n - 1; ++i)
            for (int j = i + 1; j < n; ++j) {
                const int a = (h->chord[i] - h->chord[j] + 10 * sn) % sn, b = (perm[i] - perm[j] + 10 * sn) % sn;
                if ((a == 4 && b == 4) || (a == 0 && b == 0))
                    par = 1;
            }
        if (d < anyd)
            anyd = d, any = p;
        if (!par && d < bestd)
            bestd = d, best = p;
    }
    if (best < 0)
        best = any;
    for (int i = 0; i < n; ++i)
        h->chord[i] = nc[n == 3 ? P3[best][i] : n == 2 ? P2[best][i] : 0];
}

void harm_step(struct harmony* h) {
    const int f = next_function(h);
    voice_chord(h, f);
    h->func = f;
    h->fresh = 0;
    h->pos = (h->pos + 1) % (h->phrase > 0 ? h->phrase : 1);
    if (h->mlen)
        h->midx = (h->midx + 1) % h->mlen;
    ++h->chords;
}

/* ---- output -------------------------------------------------------------------------------- */
static struct harm_frame* frame(struct harmony* h) {
    if (h->nout >= 4)
        return 0;
    struct harm_frame* f = &h->out[h->nout++];
    f->n = 0;
    return f;
}
static void put(struct harm_frame* f, int degree_note, int vel) {
    if (!f || f->n >= HARM_MAX_EV)
        return;
    struct smp_event* e = &f->ev[f->n++];
    e->type = EV_SYN_NOTE;
    e->chan = 0;
    e->a = (uint8_t)(degree_note < 0 ? 0 : degree_note > 127 ? 127 : degree_note);
    e->b = (uint8_t)vel;
    e->value = 0;
}
static void all_off(struct harmony* h) {
    struct harm_frame* f = 0;
    for (int i = 0; i < HARM_VOICES; ++i)
        if (h->sounding[i] >= 0) {
            if (!f)
                f = frame(h);
            put(f, h->sounding[i], 0);
            h->sounding[i] = -1;
        }
}
/* the chord's degrees as the synth's degree notes (64 = the root): an octave below the root
   upwards, moved by harmOctave */
static int note_of(const struct harmony* h, int c) { return 64 + c - h->scale_n + h->octave * h->scale_n; }

static void play_chord(struct harmony* h) {
    struct harm_frame* f = frame(h);
    for (int i = 0; i < HARM_VOICES; ++i) { /* offs first: the voices come free */
        const int want = i < h->voices ? note_of(h, h->chord[i]) : -1;
        if (h->sounding[i] >= 0 && h->sounding[i] != want) {
            put(f, h->sounding[i], 0);
            h->sounding[i] = -1;
        }
    }
    for (int i = 0; i < h->voices; ++i) {
        const int want = note_of(h, h->chord[i]);
        if (h->sounding[i] != want) {
            put(f, want, h->vel);
            h->sounding[i] = want;
        }
    }
}

static uint32_t step_len(const struct harmony* h) { return (uint32_t)(h->beats > 0 ? h->beats : 1) * 8; }

static void restart(struct harmony* h) {
    h->fresh = 1;
    h->pos = 0;
    h->func = 1;
    h->have_step = 0;
    for (int i = 0; i < HARM_VOICES; ++i)
        h->chord[i] = 2 * i;
}

void harm_init(struct harmony* h, uint32_t seed) {
    memset(h, 0, sizeof(*h));
    h->beats = 4;
    h->phrase = 8;
    h->voices = 3;
    h->melody_on = 1;
    h->vel = 100;
    h->gate = 1.0f;
    h->scale_n = 7;
    h->seed = (int)(seed & 127);
    for (int i = 0; i < HARM_VOICES; ++i)
        h->sounding[i] = -1;
    harm_new_melody(h, seed);
    restart(h);
}

static void tick(struct harmony* h, uint32_t step, int playing) {
    h->step = step;
    if (!h->on || !playing) {
        if (h->playing || !h->on)
            all_off(h);
        h->playing = playing;
        return;
    }
    h->playing = 1;
    const uint32_t L = step_len(h);
    if (!h->have_step) { /* the first chord on the next step boundary (now if on one) */
        h->have_step = 1;
        h->next_at = (step + L - 1) / L * L;
        h->off_at = 0xFFFFFFFFu;
    }
    if (h->off_at != 0xFFFFFFFFu && (int32_t)(step - h->off_at) >= 0) {
        all_off(h);
        h->off_at = 0xFFFFFFFFu;
    }
    if ((int32_t)(step - h->next_at) >= 0) {
        harm_step(h);
        if (h->gate < 0.999f)
            all_off(h); /* a gate: every chord struck again */
        play_chord(h);
        if (h->gate < 0.999f) {
            uint32_t g = (uint32_t)(h->gate * (float)L);
            h->off_at = h->next_at + (g < 1 ? 1 : g);
        }
        h->next_at += L;
        if ((int32_t)(step - h->next_at) >= 0) /* ticks lost: the grid from here */
            h->next_at = (step / L + 1) * L;
    }
}

static int nat_int(int32_t v) { return (v + (v >= 0 ? 32768 : -32768)) >> 16; }

void harm_rx(struct harmony* h, const struct smp_event* e) {
    switch (e->type) {
    case EV_TICK: tick(h, (uint32_t)e->value, e->a & 1); break;
    case EV_START: restart(h); break;
    case EV_STOP:
        all_off(h);
        h->playing = 0;
        restart(h);
        break;
    case EV_SYN_PARAM: {
        const int v = nat_int(e->value);
        switch (e->a) {
        case SP_HARMONY:
            if (!v)
                all_off(h);
            else if (!h->on)
                restart(h);
            h->on = v != 0;
            break;
        case SP_HARMBEATS: h->beats = v < 1 ? 1 : v; break;
        case SP_HARMPHRASE:
            h->phrase = v < 1 ? 1 : v;
            h->pos %= h->phrase;
            break;
        case SP_HARMOCTAVE: h->octave = v; break;
        case SP_HARMVOICES:
            h->voices = v < 1 ? 1 : v > HARM_VOICES ? HARM_VOICES : v;
            break;
        case SP_HARMGATE: h->gate = (float)e->value / 65536.0f; break;
        case SP_HARMVEL: h->vel = v < 1 ? 1 : v > 127 ? 127 : v; break;
        case SP_HARMMELODY: h->melody_on = v != 0; break;
        case SP_HARMSEED:
            h->seed = v;
            harm_new_melody(h, 0x9E3779B9u * (uint32_t)(v + 1));
            break;
        case SP_SCALE: {
            const int n = scale_size(v);
            if (n != h->scale_n && n > 0) {
                h->scale_n = n;
                harm_new_melody(h, 0x9E3779B9u * (uint32_t)(h->seed + 1));
            }
            break;
        }
        default: break;
        }
        break;
    }
    default: break;
    }
}

int harm_pop(struct harmony* h, struct harm_frame* f) {
    if (!h->nout)
        return 0;
    *f = h->out[0];
    memmove(h->out, h->out + 1, (size_t)(h->nout - 1) * sizeof(h->out[0]));
    --h->nout;
    return 1;
}
