/* The break cutter, see cutter.h. */
#include "cutter.h"

#include <math.h>
#include <string.h>

/* ---- random ------------------------------------------------------------------------------ */
static uint32_t xs(uint32_t* s) {
    uint32_t x = *s ? *s : 0x9E3779B9u;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return *s = x;
}
static float uni(uint32_t* s) { return (float)(xs(s) >> 8) * (1.0f / 16777216.0f); }
static float clamp01(float x) { return x < 0 ? 0 : x > 1 ? 1 : x; }

/* every 16th takes the same number of draws from the phrase's stream, so a replayed phrase
   (memory) lines up draw for draw; each can be mutated from the fresh stream */
#define NDRAW 12

/* ---- moves -------------------------------------------------------------------------------- */
enum {
    M_STRAIGHT, M_NUDGE, M_JUMP, M_SCAN, M_REPEAT, M_STAB, M_PITCH, M_SWAP, M_RATCHET, M_HALF,
    M_REVERSE, M_FREEZE, M_TAPESTOP, M_DROP, M_ONE /* the phrase's first step: back on the one */
};
static const struct { const char* name; float dis; int fam; } k_move[CUT_NMOVES] = {
    {"straight", 0.00f, -1},        {"nudge", 0.15f, CF_REORDER},  {"jump", 0.30f, CF_REORDER},
    {"scan", 0.40f, CF_REORDER},    {"repeat", 0.45f, CF_REPEAT},  {"stab", 0.50f, CF_TEXTURE},
    {"pitch", 0.55f, CF_MANGLE},    {"swap", 0.58f, CF_TEXTURE},   {"ratchet", 0.62f, CF_REPEAT},
    {"half", 0.68f, CF_MANGLE},     {"reverse", 0.72f, CF_MANGLE}, {"freeze", 0.82f, CF_TEXTURE},
    {"tapestop", 0.88f, CF_MANGLE}, {"drop", 0.95f, CF_SILENCE},   {"one", 0.00f, -2},
};
const char* cut_move_name(int m) { return m >= 0 && m < CUT_NMOVES ? k_move[m].name : "?"; }

/* character 0 .. 1 walks these four palettes: edits, rolls, tape, texture */
static const float k_corner[4][CF_NUM] = {
    {1.0f, 0.3f, 0.1f, 0.2f, 0.1f},
    {0.4f, 1.0f, 0.3f, 0.2f, 0.1f},
    {0.3f, 0.5f, 1.0f, 0.3f, 0.2f},
    {0.2f, 0.3f, 0.4f, 1.0f, 0.4f},
};
static void palette_of(float x, float* out) {
    const float u = clamp01(x) * 3;
    const int i = u >= 3 ? 2 : (int)u;
    const float f = u - (float)i;
    for (int k = 0; k < CF_NUM; ++k)
        out[k] = k_corner[i][k] * (1 - f) + k_corner[i + 1][k] * f;
}

/* ---- presets ------------------------------------------------------------------------------ */
struct preset {
    const char* name;
    float knob[CK_NUM]; /* energy density syncopation anchor build character release memory surprise */
    int release_mode;
    float accel;
};
static const struct preset k_preset[] = {
    {"straight", {0.15f, 0.25f, 0.30f, 0.80f, 0.50f, 0.00f, 0.00f, 0.80f, 0.10f}, CR_RUN, 0.5f},
    {"edits", {0.45f, 0.40f, 0.50f, 0.60f, 0.50f, 0.05f, 0.50f, 0.60f, 0.20f}, CR_RUN, 0.5f},
    {"jungle", {0.60f, 0.55f, 0.60f, 0.50f, 0.60f, 0.30f, 0.50f, 0.50f, 0.30f}, CR_FILL, 0.8f},
    {"drill", {0.70f, 0.35f, 0.70f, 0.40f, 0.80f, 0.36f, 0.40f, 0.30f, 0.20f}, CR_RUN, 1.0f},
    {"tape", {0.75f, 0.30f, 0.50f, 0.50f, 0.70f, 0.66f, 0.60f, 0.40f, 0.30f}, CR_DROP, 0.6f},
    {"v13", {0.70f, 0.20f, 0.85f, 0.60f, 0.50f, 0.40f, 0.50f, 0.00f, 0.30f}, CR_RUN, 1.0f},
    {"texture", {0.60f, 0.30f, 0.60f, 0.50f, 0.40f, 1.00f, 0.60f, 0.50f, 0.40f}, CR_DROP, 0.5f},
};
#define NPRESETS ((int)(sizeof(k_preset) / sizeof(k_preset[0])))
int cut_npresets(void) { return NPRESETS; }
const char* cut_preset_name(int n) { return k_preset[((n % NPRESETS) + NPRESETS) % NPRESETS].name; }

static void params_default(struct cut_params* p) {
    memset(p, 0, sizeof(*p));
    p->beats_per_bar = 4;
    p->phrase_bars = 4;
    p->section_phrases = 4;
    p->accel = 1.0f;
}

void cut_preset(struct cutter* c, int n) {
    const struct preset* s = &k_preset[((n % NPRESETS) + NPRESETS) % NPRESETS];
    c->preset = ((n % NPRESETS) + NPRESETS) % NPRESETS;
    memcpy(c->p.knob, s->knob, sizeof(s->knob));
    c->p.release_mode = s->release_mode;
    c->p.accel = s->accel;
    c->p.palette_fixed = 0;
}

void cut_set_knob(struct cutter* c, int k, float v) {
    if (k >= 0 && k < CK_NUM)
        c->p.knob[k] = clamp01(v);
}

void cut_set_conductor(struct cutter* c, int on) { c->conductor = !!on; }

void cut_init(struct cutter* c, uint32_t seed) {
    memset(c, 0, sizeof(*c));
    params_default(&c->p);
    cut_preset(c, 1);
    c->e = c->p;
    c->rng_fresh = seed ? seed : 1;
    c->bpm = 120;
    c->n16 = 16;
    c->last_buf = -1;
    c->last_rate = 1;
}

/* ---- the conductor ------------------------------------------------------------------------ */
enum { CS_GROOVE, CS_BUILD, CS_PEAK, CS_BREAK, CS_NUM };
static const float k_trans[CS_NUM][CS_NUM] = {
    {0.50f, 0.40f, 0.00f, 0.10f}, /* groove -> */
    {0.20f, 0.00f, 0.70f, 0.10f}, /* build  -> */
    {0.50f, 0.00f, 0.10f, 0.40f}, /* peak   -> */
    {0.50f, 0.50f, 0.00f, 0.00f}, /* break  -> */
};

static void conductor_next(struct cutter* c) {
    const float s = c->p.knob[CK_SURPRISE];
    float r = uni(&c->rng_fresh), acc = 0;
    int next = c->cond_state;
    for (int k = 0; k < CS_NUM; ++k) {
        acc += (1 - s) * k_trans[c->cond_state][k] + s * (1.0f / CS_NUM);
        if (r < acc) {
            next = k;
            break;
        }
    }
    c->cond_state = next;
}

/* what plays: the knobs, through the conductor's section state (q: progress in the section) */
static void effective(struct cutter* c, float q) {
    c->e = c->p;
    if (!c->p.palette_fixed)
        palette_of(c->p.knob[CK_CHARACTER], c->e.palette);
    if (!c->conductor)
        return;
    float* k = c->e.knob;
    switch (c->cond_state) {
    case CS_GROOVE:
        k[CK_ENERGY] *= 0.5f;
        k[CK_DENSITY] *= 0.8f;
        break;
    case CS_BUILD:
        k[CK_ENERGY] *= 0.4f + 0.9f * q;
        k[CK_DENSITY] *= 0.8f + 0.6f * q;
        k[CK_BUILD] = 0.8f;
        break;
    case CS_PEAK:
        k[CK_ENERGY] *= 1.3f;
        k[CK_DENSITY] *= 1.2f;
        k[CK_CHARACTER] += 0.25f;
        break;
    case CS_BREAK:
        k[CK_ENERGY] *= 0.6f;
        k[CK_DENSITY] *= 0.4f;
        k[CK_CHARACTER] = 0.5f * k[CK_CHARACTER] + 0.5f;
        c->e.release_mode = CR_DROP;
        break;
    }
    for (int i = 0; i < CK_NUM; ++i)
        k[i] = clamp01(k[i]);
    if (!c->p.palette_fixed)
        palette_of(k[CK_CHARACTER], c->e.palette);
}

/* ---- meter --------------------------------------------------------------------------------- */
static int bar16(const struct cutter* c) { return 4 * c->e.beats_per_bar; }
static int phrase16(const struct cutter* c) { return bar16(c) * c->e.phrase_bars; }

/* Metric stability: sum over the 8th, beat, bar and phrase containing 16th n of how early in the
   unit it sits, weighted by the unit's length; levels: 3 = up to the bar, 4 = up to the phrase */
static float metric(const struct cutter* c, int n, int levels) {
    const int per[4] = {2, 4, bar16(c), phrase16(c)};
    float num = 0, den = 0;
    for (int i = 0; i < levels; ++i) {
        num += (float)(per[i] - n % per[i]);
        den += (float)per[i];
    }
    return den > 0 ? num / den : 1;
}

/* a bar's hits: density hits, the bits of an intensity number placed in metric
   order (the most salient position takes the top bit), fixed up from the least salient end */
static uint16_t bar_pattern(struct cutter* c, float density, float sync) {
    const int n = bar16(c) > 16 ? 16 : bar16(c);
    int order[16];
    for (int i = 0; i < n; ++i)
        order[i] = i;
    for (int i = 1; i < n; ++i) { /* by metric weight, strongest first (insertion sort) */
        const int v = order[i];
        int j = i;
        for (; j > 0 && metric(c, order[j - 1], 3) < metric(c, v, 3); --j)
            order[j] = order[j - 1];
        order[j] = v;
    }
    const float d = clamp01(density * (0.6f + 0.8f * uni(&c->rng_phrase)));
    const int k = (int)(d * (float)n + 0.5f);
    const float inten = clamp01(1 - sync + (uni(&c->rng_phrase) - 0.5f) * 0.3f);
    const uint32_t I = (uint32_t)(inten * (float)((1u << n) - 1));
    int bits[16], count = 0;
    for (int j = 0; j < n; ++j)
        count += bits[j] = (int)((I >> (n - 1 - j)) & 1);
    for (int j = n - 1; j >= 0 && count != k; --j)
        if (bits[j] == (count > k)) {
            bits[j] ^= 1;
            count += bits[j] ? 1 : -1;
        }
    uint16_t pat = 0;
    for (int j = 0; j < n; ++j)
        if (bits[j])
            pat |= (uint16_t)(1u << order[j]);
    return pat;
}

static void new_phrase(struct cutter* c) {
    const int sp = c->e.section_phrases > 0 ? c->e.section_phrases : 4;
    const int section_start = c->phrase_idx % (uint32_t)sp == 0;
    if (section_start && c->conductor)
        conductor_next(c);
    effective(c, 0);
    const float m = c->e.knob[CK_MEMORY];
    const float keep = section_start ? m * m : m;
    if (!c->phrase_seed || uni(&c->rng_fresh) >= keep)
        c->phrase_seed = xs(&c->rng_fresh) | 1;
    c->rng_phrase = c->phrase_seed;
    for (int b = 0; b < c->e.phrase_bars && b < 16; ++b)
        c->bar_pattern[b] = bar_pattern(c, c->e.knob[CK_DENSITY], c->e.knob[CK_SYNCOPATION]);
}

/* ---- output -------------------------------------------------------------------------------- */
static struct cut_frame* frame_at(struct cutter* c, float delay) {
    if (c->nq >= CUT_QUEUE)
        return 0;
    struct cut_frame* f = &c->q[c->nq++];
    f->due = c->now + delay;
    f->seq = c->seq++;
    f->n = 0;
    return f;
}
static void ev(struct cut_frame* f, int pid, int32_t v) {
    if (f && f->n < CUT_MAX_EV) {
        const struct smp_event e = {EV_RAW_PARAM, 0, 0, (uint8_t)pid, v};
        f->ev[f->n++] = e;
    }
}
static int32_t q16(float x) { return (int32_t)lrintf(x * 65536.0f); }
static float wrap01(float x) { return x - floorf(x); }
static void trig_at(struct cut_frame* f, float pos) {
    ev(f, P_POS, (int32_t)(wrap01(pos) * 1073741824.0f));
    ev(f, P_TRIG, 0);
    ev(f, P_POS, -1); /* Latch(pos, trig): the playhead runs on from there */
}

#define AMP 0.89f  /* -1 dB */
#define GATE_LONG 16.0f

/* one hit: buffer, volume, speeds, gate, stutter, then the position + trigger */
static struct cut_frame* hit(struct cutter* c, float delay, int buf, float amp, float rate, float pitch, float gate,
                             int repeats, float interval, float pos) {
    struct cut_frame* f = frame_at(c, delay);
    if (!f)
        return 0;
    if (buf != c->last_buf) {
        ev(f, P_BUFNUM, buf);
        c->last_buf = buf;
    }
    ev(f, P_AMP, q16(amp));
    ev(f, P_RATE, q16(rate));
    ev(f, P_PITCH, q16(pitch));
    ev(f, P_ENVDUR, q16(gate));
    ev(f, P_REPEATS, repeats);
    if (repeats)
        ev(f, P_INTERVAL, q16(interval > 0.25f ? 0.25f : interval));
    trig_at(f, pos);
    c->last_rate = rate;
    return f;
}

static int choose_move(const struct cutter* c, float d, float r) {
    const float sigma = 0.15f + 0.20f * c->e.knob[CK_SURPRISE];
    float w[CUT_NMOVES], sum = 0;
    for (int m = 0; m < M_ONE; ++m) {
        const float x = (k_move[m].dis - d) / sigma;
        const float fam = k_move[m].fam < 0 ? 1.0f : c->e.palette[k_move[m].fam];
        sum += w[m] = fam * expf(-x * x);
    }
    float acc = 0;
    for (int m = 0; m < M_ONE; ++m) {
        acc += w[m];
        if (r * sum < acc)
            return m;
    }
    return M_STRAIGHT;
}

/* the 16th n of the phrase (with its 32nd step for timing) */
static void sixteenth(struct cutter* c, int n, int emit) {
    float r[NDRAW];
    const float mut = 0.15f * (1 - c->e.knob[CK_MEMORY]);
    const int P = phrase16(c);
    const float p = (float)n / (float)P;
    const int sp = c->e.section_phrases > 0 ? c->e.section_phrases : 4;
    const float q = ((float)(c->phrase_idx % (uint32_t)sp) + p) / (float)sp;
    effective(c, q);
    const float ex = powf(2.0f, (c->e.knob[CK_BUILD] - 0.5f) * 4.0f);
    const float T = clamp01(0.75f * powf(p, ex) + 0.25f * powf(q, ex));
    for (int i = 0; i < NDRAW; ++i) { /* same count every 16th */
        float v = uni(&c->rng_phrase);
        if (uni(&c->rng_fresh) < mut * (0.5f + T))
            v = uni(&c->rng_fresh);
        r[i] = v;
    }
    if (!emit || !c->owner || !c->playing)
        return;
    const float beat = 60.0f / (c->bpm > 1 ? c->bpm : 120);
    const float s16 = beat / 4;
    const int L = c->n16 > 0 ? c->n16 : 16;
    const float natural = (float)(n % L) / (float)L;
    const float g16 = 1.0f / (float)L;
    const float w = metric(c, n, 4);
    const float rel = c->e.knob[CK_RELEASE] * 0.5f;
    const int in_release = rel > 0 && p >= 1 - rel;
    const int release_start = in_release && (n == 0 || (float)(n - 1) / (float)P < 1 - rel);
    int buf = c->e.base_buf;

    if (n == 0) { /* back on the one: the loop where it belongs, everything reset */
        hit(c, 0, buf, AMP, 1, 1, GATE_LONG, 0, 0, natural);
        ++c->stats_hits[M_ONE];
        c->busy_until = 0;
        return;
    }
    if (c->now < c->busy_until)
        return;
    if (in_release) {
        if (c->e.release_mode == CR_DROP && release_start) {
            struct cut_frame* f = frame_at(c, 0);
            ev(f, P_GATEOFF, 0);
            ++c->stats_hits[M_DROP];
        }
        if (c->e.release_mode != CR_FILL)
            return; /* run: the loop plays on; drop: silence till the one */
        buf += 1;   /* fill: the next sample, rolled */
    }
    const int bar = (n / bar16(c)) % 16;
    const int in_bar = n % bar16(c);
    int is_hit = (c->bar_pattern[bar] >> (in_bar & 15)) & 1;
    if (in_release && c->e.release_mode == CR_FILL)
        is_hit = is_hit || r[1] < 0.5f;
    if (!is_hit)
        return;

    /* tension and metric weakness both in [0, 1] and multiplied: the square root keeps a weak
       position late in the phrase in the upper half; anchor 0.5 = (1 - w) as it is */
    float d = powf(c->e.knob[CK_ENERGY], 0.7f) * sqrtf(T * powf(1 - w, c->e.knob[CK_ANCHOR] * 2.0f));
    if (in_release && c->e.release_mode == CR_FILL && d < 0.5f)
        d = 0.5f;
    d = clamp01(d);
    int m = choose_move(c, d, r[2]);
    if (in_release && c->e.release_mode == CR_FILL && (m < M_REPEAT || m == M_DROP))
        m = r[3] < 0.5f ? M_REPEAT : M_RATCHET;
    ++c->stats_hits[m];
    float pos = natural, rate = 1, pitch = 1, gate = GATE_LONG;

    switch (m) {
    case M_STRAIGHT:
        break;
    case M_NUDGE: {
        const int k = 1 + (int)(r[4] * 2);
        pos = natural + (float)(r[5] < 0.5f ? -k : k) * g16;
        break;
    }
    case M_JUMP: {
        const int grid = w > 0.5f ? 4 : w > 0.25f ? 2 : 1;
        const int span = 1 + (int)(d * (float)L / (float)(2 * grid));
        const int k = 1 + (int)(r[4] * (float)span);
        pos = natural + (float)((r[5] < 0.5f ? -k : k) * grid) * g16;
        break;
    }
    case M_SCAN: /* the later SC cutters: phrase progress + bar offset + stability, on 16ths */
        pos = floorf(wrap01(p + floorf((float)c->phrase_idx / 8.0f * 4.0f + 0.5f) / 4.0f + w) * (float)L) * g16;
        break;
    case M_REPEAT: {
        static const float divs[] = {4, 6, 8, 3, 12};
        const float interval = beat / divs[(int)(r[4] * 5) % 5];
        const int reps = 1 + (int)(d * 6);
        hit(c, 0, buf, AMP, 1, 1, GATE_LONG, reps, interval, natural);
        c->busy_until = c->now + (float)reps * interval;
        return;
    }
    case M_STAB:
        gate = s16 * (0.5f + r[4]);
        break;
    case M_PITCH: {
        static const float ratios[] = {1.335f, 0.75f, 1.5f, 0.667f, 2.0f, 0.5f};
        const int span = 1 + (int)(d * 5);
        pitch = ratios[(int)(r[4] * (float)span) % 6];
        break;
    }
    case M_SWAP:
        buf += (r[4] < 0.5f ? -1 : 1) * (1 + (int)(r[5] * 2));
        if (buf < 0)
            buf = -buf;
        break;
    case M_RATCHET: { /* the SC cutters: n = 6..36 retriggers at repeatDur / (1 + k), tape tail */
        static const float divs[] = {18, 24, 16, 16, 8, 6, 3, 9, 12};
        const float rd = beat / divs[(int)(r[4] * 9) % 9];
        int n_rep = 6 * (1 + (int)(r[5] * (1 + d * 5)));
        float t = 0;
        const float slow = r[6] < (1 - w) * 0.6f ? (r[7] < 0.5f ? 0.1f : 0.01f + 0.05f * r[8]) : 1;
        for (int k = 0; k < n_rep && t < 2 * beat; ++k) {
            const float a = AMP * (1 - 0.5f * (float)k / (float)n_rep);
            hit(c, t, buf, a, k > 10 ? slow : 1, 1, GATE_LONG, 0, 0, natural);
            t += rd / (1 + c->e.accel * (float)k);
        }
        c->busy_until = c->now + t;
        return;
    }
    case M_HALF:
        rate = 0.5f;
        break;
    case M_REVERSE:
        rate = -1;
        pos = natural + g16;
        break;
    case M_FREEZE:
    case M_TAPESTOP: {
        const int len = 1 + (int)(r[4] * (1 + d * 3)); /* 16ths */
        if (m == M_FREEZE)
            hit(c, 0, buf, AMP, 0, 1, GATE_LONG, 0, 0, natural);
        else
            for (int k = 0; k < 8; ++k) {
                const float x = 1 - (float)k / 8;
                struct cut_frame* f = k ? frame_at(c, (float)k * (float)len * s16 / 8) : 0;
                if (!k)
                    hit(c, 0, buf, AMP, 1, 1, GATE_LONG, 0, 0, natural);
                else
                    ev(f, P_RATE, q16(0.02f + 0.98f * x * x));
            }
        /* back where the loop would be */
        hit(c, (float)len * s16, buf, AMP, 1, 1, GATE_LONG, 0, 0, natural + (float)len * g16);
        c->busy_until = c->now + (float)len * s16;
        return;
    }
    case M_DROP: {
        struct cut_frame* f = frame_at(c, 0);
        ev(f, P_GATEOFF, 0);
        return;
    }
    }
    hit(c, 0, buf, AMP, rate, pitch, gate, 0, 0, pos);
}

static void step32(struct cutter* c, uint32_t s, int emit) {
    const uint32_t P32 = (uint32_t)phrase16(c) * 2;
    if (s % P32 == 0 || !c->have_step) {
        c->phrase_idx = s / P32;
        new_phrase(c);
    }
    c->have_step = 1;
    if (s & 1)
        return;
    sixteenth(c, (int)((s % P32) / 2), emit);
}

/* ---- from the FPGA ------------------------------------------------------------------------- */
static void owner_base(struct cutter* c) {
    struct cut_frame* f = frame_at(c, 0);
    ev(f, P_ENVHOLD, 0);
    ev(f, P_BGADVANCE, 0);
    ev(f, P_REPEATS, 0);
    c->last_buf = -1;
}

void cut_rx(struct cutter* c, const struct smp_event* e) {
    switch (e->type) {
    case EV_TICK: {
        const uint32_t s = (uint32_t)e->value;
        c->playing = e->a & 1;
        if (c->have_step && s > c->step + 1 && s - c->step < 64) /* lost ticks: catch up quietly */
            for (uint32_t k = c->step + 1; k < s; ++k)
                step32(c, k, 0);
        else if (c->have_step && (s <= c->step || s - c->step >= 64))
            c->have_step = 0; /* restarted */
        c->step = s;
        step32(c, s, 1);
        break;
    }
    case EV_BPM:
        if (e->value > 1000)
            c->bpm = (float)e->value / 1000.0f;
        break;
    case EV_START:
        c->have_step = 0;
        c->playing = 1;
        c->busy_until = 0;
        c->nq = 0;
        break;
    case EV_STOP:
        c->playing = 0;
        c->nq = 0;
        break;
    case EV_OWNER:
        if (e->a == 0) {
            if (e->value > 0)
                c->n16 = e->value;
            if (e->b && !c->owner)
                owner_base(c);
            if (!e->b)
                c->nq = 0;
            c->owner = e->b;
        }
        break;
    case EV_LOAD_BUF: /* the drum page's sample knob: the cutter's base sample */
        if (e->a == 0)
            c->p.base_buf = e->b;
        break;
    case EV_CC: /* the cutter channel: CC 16.. = the knobs, mod wheel = release */
        if (e->a >= 16 && e->a < 16 + CK_NUM)
            cut_set_knob(c, e->a - 16, (float)e->b / 127.0f);
        else if (e->a == 1)
            cut_set_knob(c, CK_RELEASE, (float)e->b / 127.0f);
        break;
    case EV_NOTE_ON: /* a note picks a preset */
        if (e->b)
            cut_preset(c, e->a);
        break;
    case EV_BEND: /* above the centre: the conductor plays the form */
        cut_set_conductor(c, e->value > 8192 + 512);
        break;
    default:
        break;
    }
}

void cut_clock(struct cutter* c, double t) { c->now = t; }

int cut_pop(struct cutter* c, double t, struct cut_frame* out) {
    c->now = t;
    int best = -1;
    for (int i = 0; i < c->nq; ++i)
        if (c->q[i].due <= t && (best < 0 || c->q[i].due < c->q[best].due ||
                                 (c->q[i].due == c->q[best].due && c->q[i].seq < c->q[best].seq)))
            best = i;
    if (best < 0)
        return 0;
    *out = c->q[best];
    c->q[best] = c->q[--c->nq];
    return 1;
}
