/* The wavetable synth's control, see synth_ctl.h. Register map: rtl/synth_top.v (mixer),
 * rtl/synth_osc.v (oscillators, sub, noise), rtl/synth_filt.v (filter slots). */
#include "synth_ctl.h"

#include <string.h>

#include "dmath.h"
#include "scales.h"

/* ---- parameters ------------------------------------------------------------------------- */
struct pdef {
    const char* name;
    uint8_t kind, mod;
    float lo, hi, def;
};
#define SY_DEF(id, name, kind, lo, hi, def, mod) {#name, kind, mod, (float)(lo), (float)(hi), (float)(def)},
static const struct pdef P[SP_NUM] = {SYNTH_PARAMS(SY_DEF)};
#undef SY_DEF
#define SYS_NAME(id, name) #name,
static const char* const g_src_names[SYS_NUM] = {SYNTH_SOURCES(SYS_NAME)};
#undef SYS_NAME

#define OSTRIDE (SP_B_ON - SP_A_ON)
#define ONE24 (1 << 24)
#define TRIM 49152 /* voice gain at full volume, Q16: the synth's level beside the samples */

static int name_eq(const char* a, const char* b) {
    for (; *a && *b; ++a, ++b)
        if ((*a | 32) != (*b | 32))
            return 0;
    return *a == *b;
}
int sy_param_of(const char* name) {
    for (int p = 0; p < SP_NUM; ++p)
        if (name_eq(name, P[p].name))
            return p;
    return -1;
}
const char* sy_param_name(int p) { return (p >= 0 && p < SP_NUM) ? P[p].name : "?"; }
const char* sy_source_name(int src) { return (src >= 0 && src < SYS_NUM) ? g_src_names[src] : "?"; }

/* ---- math ------------------------------------------------------------------------------- */
static uint32_t g_exp2[257];  /* 2^(i/256), Q30 */
static int32_t g_sinq[257];   /* sin(i/256 pi/2), Q16 */
static int32_t g_gtab[257];   /* g = tan(pi fc / fs2) at fc = 20 Hz 2^(i/16 - 4), Q24 (clamped at 0.45 fs2) */
#define GTAB_OCT0 4

static double d_sin(double x) { /* |x| <= ~2 */
    double t = x, s = x;
    for (int i = 1; i < 14; ++i) {
        t *= -x * x / ((2 * i) * (2 * i + 1));
        s += t;
    }
    return s;
}
static double d_cos(double x) {
    double t = 1, s = 1;
    for (int i = 1; i < 14; ++i) {
        t *= -x * x / ((2 * i - 1) * (2 * i));
        s += t;
    }
    return s;
}
static double nat_of(int p, double k) {
    const double lo = P[p].lo, hi = P[p].hi;
    switch (P[p].kind) {
    case SK_EXP: return lo * d_exp(d_log(hi / lo) * k);
    case SK_CUBE: return lo + (hi - lo) * k * k * k;
    default: return lo + (hi - lo) * k;
    }
}
static int32_t clampi(int32_t x, int32_t lo, int32_t hi) { return x < lo ? lo : x > hi ? hi : x; }
static int32_t mul16(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> 16); }

/* 2^x for x Q16 (-16 .. 15), result Q16 */
static uint32_t exp2_q16(int32_t x) {
    const int32_t ip = x >> 16;
    const uint32_t fr = (uint32_t)x & 0xFFFF, i = fr >> 8, f = fr & 255;
    const uint32_t e = g_exp2[i] + (uint32_t)(((uint64_t)(g_exp2[i + 1] - g_exp2[i]) * f) >> 8); /* Q30 */
    const int sh = 14 - ip;                                                                     /* Q30 -> Q16 */
    if (sh >= 32) return 0;
    if (sh <= -2) return 0xFFFFFFFFu;
    return sh >= 0 ? e >> sh : e << -sh;
}
/* phase increment per sub-sample for a pitch in semitones Q16 (69 = 440 Hz) */
static uint32_t pitch_inc(const struct sy_state* s, int32_t semis) {
    const int32_t oct = (int32_t)(((int64_t)semis - (69 << 16)) / 12);
    const int32_t ip = oct >> 16;
    const uint32_t fr = (uint32_t)oct & 0xFFFF, i = fr >> 8, f = fr & 255;
    const uint32_t e = g_exp2[i] + (uint32_t)(((uint64_t)(g_exp2[i + 1] - g_exp2[i]) * f) >> 8);
    uint64_t inc = ((uint64_t)s->inc_a4 * e) >> 30;
    if (ip >= 0) inc <<= ip; else inc >>= -ip;
    return inc > 0x7FFFFFFFu ? 0x7FFFFFFFu : (uint32_t)inc;
}
static int32_t sinq(int32_t x) { /* sin(x pi/2), x Q16 in 0..1 */
    x = clampi(x, 0, 65536);
    const int i = x >> 8, f = x & 255;
    return g_sinq[i] + (((g_sinq[i + (i < 256)] - g_sinq[i]) * f) >> 8);
}
static int32_t gtab(int32_t oct_q16) { /* oct: octaves above 20 Hz */
    int32_t x = (oct_q16 + (GTAB_OCT0 << 16)) * 16; /* index Q16 */
    x = clampi(x, 0, 256 << 16);
    const int i = x >> 16, f = (x >> 8) & 255;
    if (i >= 256) return g_gtab[256];
    return g_gtab[i] + (int32_t)(((int64_t)(g_gtab[i + 1] - g_gtab[i]) * f) >> 8);
}
static uint32_t rnd32(struct sy_state* s) {
    uint32_t r = s->rng;
    r ^= r << 13, r ^= r >> 17, r ^= r << 5;
    return s->rng = r;
}

/* ---- registers ---------------------------------------------------------------------------- */
#define A_OSC(o, r) ((uint32_t)(((o) << 5) | (r)))
#define A_VOI(v, r) ((uint32_t)(0x100 | ((v) << 4) | (r)))
#define A_FIN(t, l, r) ((uint32_t)((1 << 11) | ((t) << 7) | ((l) << 5) | (r)))
#define A_FCON(r) ((uint32_t)((1 << 11) | (1 << 9) | ((r) - 32)))
#define A_FPROG(t) ((uint32_t)((1 << 11) | (2 << 9) | (t)))
#define A_W(v, t) ((uint32_t)((2 << 11) | ((v) * 4 + (t))))
#define A_AMP(v) ((uint32_t)((2 << 11) | 16 | (v)))

static void W(struct sy_state* s, uint32_t a, uint32_t d) {
    a &= 8191;
    if ((s->valid[a >> 3] >> (a & 7)) & 1 && s->shadow[a] == d)
        return;
    s->shadow[a] = d;
    s->valid[a >> 3] |= (uint8_t)(1 << (a & 7));
    ++s->writes;
    s->wr(a, d);
}

/* ---- parameter values ----------------------------------------------------------------------- */
static int ival(const struct sy_state* s, int p) { /* an SK_INT parameter */
    const int lo = (int)P[p].lo, hi = (int)P[p].hi;
    return lo + (int)(((int64_t)s->knob[p] * (hi - lo) + 32768) >> 16);
}
static int32_t lin_q16(int p, int32_t k) { /* an SK_LIN parameter at knob k, natural Q16 */
    const int32_t lo = (int32_t)(P[p].lo * 65536.0f), span = (int32_t)((P[p].hi - P[p].lo) * 65536.0f);
    return lo + (int32_t)(((int64_t)span * k) >> 16);
}

static void env_derive(struct sy_state* s, int e) {
    const double dt = (double)SY_TICK * 65536.0 / s->fs_q16;
    const int b = SP_ENV1A + 5 * e;
    const double a = nat_of(b, s->knob[b] / 65536.0), h = nat_of(b + 1, s->knob[b + 1] / 65536.0),
                 d = nat_of(b + 2, s->knob[b + 2] / 65536.0), su = nat_of(b + 3, s->knob[b + 3] / 65536.0),
                 r = nat_of(b + 4, s->knob[b + 4] / 65536.0);
    struct sy_envp* E = &s->envp[e];
    E->atk_step = a > dt ? (int32_t)(dt / a * ONE24) : ONE24;
    if (E->atk_step < 1) E->atk_step = 1;
    E->hold = (uint32_t)(h / dt);
    E->dec_ticks = (uint32_t)(d / dt);
    E->dec_k = d > 0 ? (int32_t)(d_exp(-6.9 * dt / d) * 1073741824.0) : 0;
    E->sus = (int32_t)(su * ONE24);
    E->rel_ticks = (uint32_t)(r / dt);
    E->rel_k = r > 0 ? (int32_t)(d_exp(-6.9 * dt / r) * 1073741824.0) : 0;
}
static void lfo_derive(struct sy_state* s, int l) {
    const int b = SP_LFO1SHAPE + 7 * l;
    double rate = nat_of(b + 1, s->knob[b + 1] / 65536.0);
    if (ival(s, b + 2)) { /* synced: cycles per beat, the nearest power of two */
        const int e = (int)(d_log(rate) / 0.69314718 + (rate >= 1 ? 0.5 : -0.5));
        rate = 1;
        for (int i = 0; i < (e < 0 ? -e : e); ++i)
            rate = e < 0 ? rate * 0.5 : rate * 2;
        rate *= s->bpm_milli / 60000.0;
    }
    s->lfo_inc[l] = (uint32_t)(rate * SY_TICK * 65536.0 / s->fs_q16 * 4294967296.0);
    const double dt = (double)SY_TICK * 65536.0 / s->fs_q16;
    s->lfo_rise[l] = (uint32_t)(nat_of(b + 4, s->knob[b + 4] / 65536.0) / dt);
    s->lfo_delay[l] = (uint32_t)(nat_of(b + 5, s->knob[b + 5] / 65536.0) / dt);
}
static void porta_derive(struct sy_state* s) {
    const double t = nat_of(SP_PORTA, s->knob[SP_PORTA] / 65536.0), dt = (double)SY_TICK * 65536.0 / s->fs_q16;
    s->porta_k = t > 0.0005 ? (int32_t)(d_exp(-4.6 * dt / t) * 1073741824.0) : 0;
}

/* the pitch a voice's note stands for (semitones Q16): MIDI as is, or through the scale */
static int32_t note_pitch(const struct sy_state* s, const struct sy_voice* v) {
    const int off = ival(s, SP_NOTEOFFSET);
    if (v->degree)
        return scale_pitch_q16(ival(s, SP_SCALE), ival(s, SP_TUNING), ival(s, SP_ROOT), v->note - 64 + off);
    const int n = v->note + off;
    if (ival(s, SP_SCALEMAP))
        return scale_pitch_q16(ival(s, SP_SCALE), ival(s, SP_TUNING), ival(s, SP_ROOT), n - 60);
    return n << 16;
}

static void env_on(struct sy_env* e) { e->st = 1, e->t = 0; }
static void env_off(struct sy_env* e) {
    if (e->st) e->st = 5, e->t = 0;
}

static void retarget(struct sy_state* s, int retrig) {
    for (int i = 0; i < SY_VOICES; ++i) {
        struct sy_voice* v = &s->v[i];
        if (!v->active || !v->gate)
            continue;
        v->target = note_pitch(s, v);
        if (!s->porta_k)
            v->pitch = v->target;
        if (retrig)
            for (int e = 0; e < 3; ++e)
                env_on(&v->env[e]);
    }
}

static void slot_models(struct sy_state* s);

static void param_changed(struct sy_state* s, int p) {
    if (p >= SP_ENV1A && p <= SP_ENV3R)
        env_derive(s, (p - SP_ENV1A) / 5);
    else if (p >= SP_LFO1SHAPE && p <= SP_LFO4PHASE)
        lfo_derive(s, (p - SP_LFO1SHAPE) / 7);
    else if (p == SP_PORTA)
        porta_derive(s);
    else if (p == SP_SCALE || p == SP_TUNING || p == SP_ROOT || p == SP_SCALEMAP)
        retarget(s, 0);
    else if (p == SP_NOTEOFFSET)
        retarget(s, ival(s, SP_ARPRETRIG));
    else if (p >= SP_SLOT1 && p <= SP_SLOT4)
        slot_models(s);
    else if ((p == SP_A_TABLE || p == SP_B_TABLE) && s->ntab) /* all resident: by index */
        s->osc_tab[p == SP_B_TABLE] = &s->tab[clampi(ival(s, p), 0, s->ntab - 1)];
}

void sy_set_knob(struct sy_state* s, int p, int32_t k) {
    if (p < 0 || p >= SP_NUM)
        return;
    k = clampi(k, 0, 65536);
    const int changed_int = P[p].kind == SK_INT ? 1 : 0;
    const int before = changed_int ? ival(s, p) : 0;
    s->knob[p] = k;
    if (p == SP_NOTEOFFSET && ival(s, p) == before)
        return; /* the same step: no retrigger */
    param_changed(s, p);
}
void sy_set_cc(struct sy_state* s, int p, int v) { sy_set_knob(s, p, (v & 127) * 65536 / 127); }

void sy_set_natural(struct sy_state* s, int p, int32_t xq) {
    if (p < 0 || p >= SP_NUM)
        return;
    const double x = xq / 65536.0, lo = P[p].lo, hi = P[p].hi;
    double k;
    switch (P[p].kind) {
    case SK_EXP: k = x <= lo ? 0 : d_log(x / lo) / d_log(hi / lo); break;
    case SK_CUBE: {
        const double y = (x - lo) / (hi - lo);
        k = y <= 0 ? 0 : d_exp(d_log(y) / 3);
        break;
    }
    default: k = (x - lo) / (hi - lo); break;
    }
    if (P[p].kind == SK_INT) { /* exactly the step asked for */
        const double steps = hi - lo;
        const int st = (int)(k * steps + 0.5);
        sy_set_knob(s, p, (int32_t)((st * 65536.0) / steps + 0.5));
        return;
    }
    sy_set_knob(s, p, (int32_t)(k * 65536.0 + 0.5));
}
int32_t sy_natural_q16(const struct sy_state* s, int p) {
    if (P[p].kind == SK_INT)
        return ival(s, p) << 16;
    return (int32_t)(nat_of(p, s->knob[p] / 65536.0) * 65536.0);
}

void sy_set_bpm(struct sy_state* s, uint32_t bpm_milli) {
    s->bpm_milli = bpm_milli ? bpm_milli : 120000;
    for (int l = 0; l < 4; ++l)
        lfo_derive(s, l);
}

/* ---- filter slots ------------------------------------------------------------------------- */
static void lane_reset(struct sy_state* s, int lane) {
    uint8_t reg[32];
    int32_t val[32];
    for (int t = 0; t < 4; ++t) {
        const int n = sy_filt_state_regs(s->slot_model[t], reg, val);
        for (int i = 0; i < n; ++i) {
            const uint32_t a = A_FIN(t, lane, reg[i]);
            s->valid[a >> 3] &= (uint8_t)~(1 << (a & 7)); /* states move: always write */
            W(s, a, (uint32_t)val[i]);
        }
        for (int i = 0; i < 4; ++i)
            s->v[lane].fkey[t][i] = -1;
    }
}
static void slot_models(struct sy_state* s) {
    for (int t = 0; t < 4; ++t) {
        const int m = ival(s, SP_SLOT1 + t);
        if (m == s->slot_model[t])
            continue;
        s->slot_model[t] = m;
        W(s, A_FPROG(t), (uint32_t)sy_filt_start(m));
        for (int l = 0; l < SY_VOICES; ++l) /* the other program's registers mean other things */
            lane_reset(s, l);
    }
}

/* ---- init ------------------------------------------------------------------------------------ */
void sy_init(struct sy_state* s, uint32_t fs_q16, sy_write_fn wr) {
    memset(s, 0, sizeof(*s));
    s->fs_q16 = fs_q16;
    s->wr = wr;
    s->rng = 0x9E3779B9u;
    s->bpm_milli = 120000;
    for (int i = 0; i <= 256; ++i) {
        g_exp2[i] = (uint32_t)(d_exp(0.69314718055994530942 * i / 256.0) * 1073741824.0 + 0.5);
        g_sinq[i] = (int32_t)(d_sin(1.5707963267948966 * i / 256.0) * 65536.0 + 0.5);
        const double fs2 = 2.0 * fs_q16 / 65536.0;
        double fc = 20.0 * d_exp(0.69314718055994530942 * (i / 16.0 - GTAB_OCT0));
        if (fc > 0.45 * fs2)
            fc = 0.45 * fs2;
        const double x = 3.14159265358979 * fc / fs2;
        g_gtab[i] = (int32_t)(d_sin(x) / d_cos(x) * ONE24 + 0.5);
    }
    s->inc_a4 = (uint32_t)(((uint64_t)440 << 47) / fs_q16);
    s->limit_inc = (uint32_t)((43200.0 * 65536.0 / (2.0 * fs_q16)) * 4294967296.0);
    for (int p = 0; p < SP_NUM; ++p) {
        double k;
        const double lo = P[p].lo, hi = P[p].hi, d = P[p].def;
        switch (P[p].kind) {
        case SK_EXP: k = d_log(d / lo) / d_log(hi / lo); break;
        case SK_CUBE: k = d <= lo ? 0 : d_exp(d_log((d - lo) / (hi - lo)) / 3); break;
        default: k = (d - lo) / (hi - lo); break;
        }
        s->knob[p] = clampi((int32_t)(k * 65536.0 + 0.5), 0, 65536);
        if (P[p].mod)
            s->dst_param[++s->ndst] = (uint8_t)p;
    }
    for (int e = 0; e < 3; ++e)
        env_derive(s, e);
    for (int l = 0; l < 4; ++l)
        lfo_derive(s, l);
    porta_derive(s);
    for (int t = 0; t < 4; ++t)
        s->slot_model[t] = -1;
}

void sy_hw_reset(struct sy_state* s) {
    memset(s->valid, 0, sizeof(s->valid));
    uint8_t reg[16];
    int32_t val[16];
    int n;
    sy_filt_constants(2 * s->fs_q16, reg, val, &n);
    for (int i = 0; i < n; ++i)
        W(s, A_FCON(reg[i]), (uint32_t)val[i]);
    for (int t = 0; t < 4; ++t)
        s->slot_model[t] = -1;
    slot_models(s);
    for (int v = 0; v < SY_VOICES; ++v) {
        s->v[v].idle_written = 0;
        s->v[v].lev[0] = s->v[v].lev[1] = 0;
        W(s, A_AMP(v), 0);
        for (int t = 0; t < 4; ++t)
            W(s, A_W(v, t), 0);
    }
}

void sy_set_tables(struct sy_state* s, const struct sy_table* t, int n) {
    if (n > SY_MAX_TABLES) n = SY_MAX_TABLES;
    memcpy(s->tab, t, sizeof(*t) * (unsigned)n);
    s->ntab = n;
    for (int o = 0; o < 2; ++o)
        s->osc_tab[o] = n ? &s->tab[clampi(ival(s, SP_A_TABLE + o * OSTRIDE), 0, n - 1)] : 0;
}
void sy_use_table(struct sy_state* s, int osc, const struct sy_table* t) { s->osc_tab[osc & 1] = t; }

/* ---- notes ------------------------------------------------------------------------------------ */
static void voice_update(struct sy_state* s, int vi, int note_on);

void sy_note_on(struct sy_state* s, int note, int vel, int degree) {
    const int mono = ival(s, SP_MONO);
    struct sy_voice* v = 0;
    if (mono)
        v = &s->v[0];
    else {
        for (int i = 0; i < SY_VOICES && !v; ++i)
            if (!s->v[i].active)
                v = &s->v[i];
        if (!v) { /* steal: a released voice first, else the oldest */
            uint32_t best = 0;
            for (int i = 0; i < SY_VOICES; ++i) {
                const uint32_t score = s->v[i].age + (s->v[i].gate ? 0 : 0x40000000u);
                if (!v || score > best)
                    best = score, v = &s->v[i];
            }
        }
    }
    const int vi = (int)(v - s->v);
    const int legato = mono && ival(s, SP_LEGATO) && v->active && v->gate;
    const int glide = s->porta_k && v->active && (legato || ival(s, SP_PORTAALWAYS) || mono);
    v->note = note;
    v->degree = (uint8_t)(degree != 0);
    v->vel = vel * 65536 / 127;
    v->target = note_pitch(s, v);
    if (!glide)
        v->pitch = v->target;
    if (legato) {
        v->gate = 1;
        return;
    }
    const int fresh = !v->active;
    v->active = v->gate = 1;
    v->age = 0;
    v->rnd = (int32_t)(rnd32(s) >> 16);
    for (int e = 0; e < 3; ++e)
        env_on(&v->env[e]);
    for (int l = 0; l < 4; ++l) {
        const int b = SP_LFO1SHAPE + 7 * l;
        if (ival(s, b + 3) != 2) { /* not free running: from the start phase */
            v->lfo[l].ph = (uint32_t)s->knob[b + 6] << 16;
            v->lfo[l].age = 0;
            v->lfo[l].done = 0;
        }
    }
    if (fresh)
        lane_reset(s, vi);
    v->lev[0] = v->lev[1] = 0;
    v->idle_written = 0;
    voice_update(s, vi, 1);
}

void sy_note_off(struct sy_state* s, int note, int degree) {
    for (int i = 0; i < SY_VOICES; ++i) {
        struct sy_voice* v = &s->v[i];
        if (v->active && v->gate && v->note == note && v->degree == (degree != 0)) {
            v->gate = 0;
            for (int e = 0; e < 3; ++e)
                env_off(&v->env[e]);
        }
    }
}
void sy_all_off(struct sy_state* s) {
    for (int i = 0; i < SY_VOICES; ++i) {
        s->v[i].gate = 0;
        for (int e = 0; e < 3; ++e)
            env_off(&s->v[i].env[e]);
    }
}
void sy_bend(struct sy_state* s, int v14) { s->bend = clampi(v14, 0, 16383) - 8192; }
void sy_patch_reese(struct sy_state* s) {
#define V(x) ((int32_t)((x) * 65536.0))
    static const struct {
        uint8_t p;
        int32_t v;
    } k[] = {
        /* mono, legato, a short glide */
        {SP_MONO, V(1)}, {SP_LEGATO, V(1)}, {SP_PORTA, V(0.04)}, {SP_VOLUME, V(0.35)},
        /* A: four saws detuned about +-1/4 semitone, free running (the beating) */
        {SP_A_UNISON, V(4)}, {SP_A_DETUNE, V(0.35)}, {SP_A_BLEND, V(0.6)}, {SP_A_LEVEL, V(0.6)}, {SP_A_RAND, V(1)},
        /* B: a table a little sharp, two voices, drifting through its frames (LFO 2) */
        {SP_B_ON, V(1)}, {SP_B_LEVEL, V(0.45)}, {SP_B_FINE, V(7)}, {SP_B_POS, V(0.3)}, {SP_B_SMOOTH, V(1)},
        {SP_B_UNISON, V(2)}, {SP_B_DETUNE, V(0.2)}, {SP_B_RAND, V(1)},
        /* a sine an octave down */
        {SP_SUB_ON, V(1)}, {SP_SUB_LEVEL, V(0.4)}, {SP_SUB_OCT, V(-1)},
        /* a Moog low-pass with a little drive, key tracked */
        {SP_SLOT1, V(1)}, {SP_CUTOFF, V(750)}, {SP_RES, V(1.2)}, {SP_DRIVE, V(0.2)}, {SP_FILTKEY, V(0.3)},
        /* envelopes: amplitude, and a bite on the cutoff */
        {SP_ENV1A, V(0.004)}, {SP_ENV1R, V(0.18)},
        {SP_ENV2A, V(0.005)}, {SP_ENV2D, V(0.4)}, {SP_ENV2S, V(0.3)}, {SP_ENV2R, V(0.3)},
        {SP_MOD1SRC, V(SYS_ENV2)}, {SP_MOD1DST, V(19)}, {SP_MOD1AMT, V(0.12)},   /* env 2 -> cutoff */
        {SP_LFO2RATE, V(0.2)}, {SP_LFO2MODE, V(2)},
        {SP_MOD3SRC, V(SYS_LFO2)}, {SP_MOD3DST, V(12)}, {SP_MOD3AMT, V(0.25)},   /* lfo 2 -> bPos */
        {SP_MOD4SRC, V(SYS_MODWHEEL)}, {SP_MOD4DST, V(19)}, {SP_MOD4AMT, V(0.4)}, /* the mod wheel opens it */
        {SP_LFO1RATE, V(0.12)}, {SP_LFO1MODE, V(2)},
        {SP_MOD5SRC, V(SYS_LFO1)}, {SP_MOD5DST, V(19)}, {SP_MOD5AMT, V(0.05)},   /* lfo 1: slow cutoff drift */
    };
#undef V
    for (unsigned i = 0; i < sizeof(k) / sizeof(k[0]); ++i)
        sy_set_natural(s, k[i].p, k[i].v);
}

int sy_active_voices(const struct sy_state* s) {
    int n = 0;
    for (int i = 0; i < SY_VOICES; ++i)
        n += s->v[i].active;
    return n;
}

/* ---- per tick ---------------------------------------------------------------------------- */
static int32_t env_step(const struct sy_envp* E, struct sy_env* e) {
    ++e->t;
    switch (e->st) {
    case 1: /* attack (linear) */
        e->v += E->atk_step;
        if (e->v >= ONE24) e->v = ONE24, e->st = 2, e->t = 0;
        break;
    case 2: /* hold */
        e->v = ONE24;
        if (e->t >= E->hold) e->st = 3, e->t = 0;
        break;
    case 3: /* decay to the sustain level: -60 dB of the distance in the decay time */
        e->v = E->sus + (int32_t)(((int64_t)(e->v - E->sus) * E->dec_k) >> 30);
        if (e->t >= E->dec_ticks) e->v = E->sus, e->st = 4;
        break;
    case 4: e->v = E->sus; break;
    case 5: /* release */
        e->v = (int32_t)(((int64_t)e->v * E->rel_k) >> 30);
        if (e->t >= E->rel_ticks || e->v < 168) e->st = 0, e->v = 0;
        break;
    default: e->v = 0; break;
    }
    return e->v;
}

static int32_t lfo_shape(int32_t shape_q16, uint32_t ph, int32_t held) { /* 0..1, Q16 */
    const int a = clampi(shape_q16 >> 16, 0, 5), b = a < 5 ? a + 1 : 5;
    const int32_t u = shape_q16 - (a << 16);
    int32_t y[2];
    for (int j = 0; j < 2; ++j) {
        const int k = j ? b : a;
        const int32_t p = (int32_t)(ph >> 16); /* 0..65535 */
        switch (k) {
        case 0: { /* 0.5 - 0.5 cos(2 pi p) */
            const int32_t q = p & 0x3FFF, quad = p >> 14;
            const int32_t sv = sinq(q * 4); /* sin of the quarter */
            const int32_t c = (quad == 0) ? sinq(65536 - q * 4) : (quad == 1) ? -sv : (quad == 2) ? -sinq(65536 - q * 4) : sv;
            y[j] = 32768 - c / 2;
            break;
        }
        case 1: y[j] = 65536 - (p < 32768 ? (32768 - p) * 2 : (p - 32768) * 2); break;
        case 2: y[j] = 65536 - p; break;
        case 3: y[j] = p; break;
        case 4: y[j] = p < 32768 ? 65536 : 0; break;
        default: y[j] = held; break;
        }
    }
    return y[0] + mul16(y[1] - y[0], clampi(u, 0, 65536));
}

/* warp registers and the largest phase slope (for the level choice), synth_warp.h */
struct warp { uint32_t par, parb, parc; int32_t slope, pm; };
static struct warp warp_regs(int w, int32_t a, int sub_oct) {
    struct warp r = {0, 0, 0, 65536, 0};
    a = clampi(a, 0, 65536);
    switch (w) {
    case 1: case 2: { /* sync: 2^(4a), u5.12 */
        const uint32_t k = exp2_q16(4 * a);
        r.par = k >> 4;
        r.slope = (int32_t)k;
        break;
    }
    case 3: case 4: case 5: { /* bend */
        const int32_t b = w == 3 ? a : w == 4 ? -a : 2 * a - 65536;
        r.par = (uint32_t)b & 0x3FFFF;
        r.slope = 65536 + (b < 0 ? -b : b);
        break;
    }
    case 6: { /* PWM: 1 / (1 - 0.95 a) */
        const int32_t width = 65536 - (int32_t)(((int64_t)a * 62259) >> 16);
        const uint32_t inv = (uint32_t)(((uint64_t)1 << 32) / (uint32_t)width); /* Q16 */
        r.par = inv >> 4 > 262143 ? 262143 : inv >> 4;
        r.slope = (int32_t)inv;
        break;
    }
    case 7: case 8: case 9: case 11: { /* asym / mirror: the knee d */
        const int32_t m49 = (int32_t)(((int64_t)a * 32113) >> 16); /* 0.49 a */
        int32_t d = w == 7 ? 32768 - m49 : w == 8 ? 32768 + m49 : 32768 + 32113 - 2 * m49;
        d = clampi(d, 655, 64881);
        const uint32_t i0 = (uint32_t)(((uint64_t)32768 << 16) / (uint32_t)d);           /* 0.5 / d, Q16 */
        const uint32_t i1 = (uint32_t)(((uint64_t)32768 << 16) / (uint32_t)(65536 - d)); /* 0.5 / (1 - d) */
        r.par = (uint32_t)d << 16;
        r.parb = i0 >> 4;
        r.parc = i1 >> 4;
        r.slope = (int32_t)(i0 > i1 ? i0 : i1) * (w == 11 ? 2 : 1);
        break;
    }
    case 10: r.par = a >= 65536 ? 0 : (uint32_t)(65536 - a) << 16; break; /* flip */
    case 12: r.par = (uint32_t)((21 * 65536 + 10 * a + 32768) >> 16); break; /* quantize */
    case 13: { /* odd / even: e = 2a - 1, norm 1 / (1 + |e|) */
        const int32_t e = 2 * a - 65536;
        r.par = (uint32_t)e & 0x3FFFF;
        r.parb = (uint32_t)(((uint64_t)1 << 32) / (uint32_t)(65536 + (e < 0 ? -e : e)));
        if (a > 32768) r.slope = 131072;
        break;
    }
    /* FM: index 2a cycles; the instantaneous frequency rises by 2a x 2 pi x the modulator's top
       (~4 harmonics of the other oscillator, the sub's own frequency) */
    case 14: r.par = (uint32_t)(2 * a) & 0x3FFFF; r.pm = a * 50; break;
    case 15: r.par = (uint32_t)(2 * a) & 0x3FFFF; r.pm = (a * 12) >> (sub_oct < 0 ? -sub_oct : 0); break;
    case 16: r.par = (uint32_t)(2 * a) & 0x3FFFF; break;
    case 17: case 18: r.par = (uint32_t)a; break; /* AM / RM depth u1.16 */
    case 19: r.par = (uint32_t)(2 * a) & 0x3FFFF; r.slope = 65536 + 4 * a; break; /* thru-zero FM, Q3.15 */
    default: break;
    }
    return r;
}
static int is_xmod(int w) { return w == 14 || w == 17 || w == 18 || w == 19; }

static const int32_t g_unorm[5] = {0, 65536, 46341, 37837, 32768}; /* 1 / sqrt(N) */

/* the mip level: the fewest harmonics' level that still covers fmax, with hysteresis */
static int choose_level(const struct sy_state* s, uint32_t fmax, int cur) {
    int ideal = 9;
    for (int k = 1; k <= 9; ++k)
        if ((uint64_t)((1024 >> k) - 1) * fmax <= s->limit_inc) {
            ideal = k;
            break;
        }
    if (cur <= 0)
        return ideal;
    if (cur < ideal) /* too many harmonics: keep them while the images stay above the band */
        return (uint64_t)((1024 >> cur) - 1) * fmax <= (uint64_t)s->limit_inc * 7 / 5 ? cur : ideal;
    if (cur > ideal) /* fewer than it could: one more only when it clearly fits */
        return (uint64_t)((1024 >> (cur - 1)) - 1) * fmax <= (uint64_t)s->limit_inc * 4 / 5 ? ideal : cur;
    return cur;
}

static void voice_idle(struct sy_state* s, int vi) {
    W(s, A_AMP(vi), 0);
    for (int o = 0; o < 2; ++o)
        W(s, A_OSC(2 * vi + o, 10), 0); /* off: no frame loads */
    s->v[vi].idle_written = 1;
}

static void voice_update(struct sy_state* s, int vi, int note_on) {
    struct sy_voice* v = &s->v[vi];
    /* modulation sources */
    int32_t src[SYS_NUM];
    src[SYS_NONE] = 0;
    for (int e = 0; e < 3; ++e)
        src[SYS_ENV1 + e] = v->env[e].v >> 8;
    for (int l = 0; l < 4; ++l) {
        const int b = SP_LFO1SHAPE + 7 * l;
        const struct sy_lfo* L = &v->lfo[l];
        int32_t y = 2 * lfo_shape(lin_q16(b, s->knob[b]), L->ph, L->held) - 65536; /* bipolar */
        if (L->age < s->lfo_delay[l]) /* delay, then a rise */
            y = 0;
        else if (L->age - s->lfo_delay[l] < s->lfo_rise[l])
            y = (int32_t)(((int64_t)y * (L->age - s->lfo_delay[l])) / s->lfo_rise[l]);
        src[SYS_LFO1 + l] = y;
    }
    src[SYS_VEL] = v->vel;
    src[SYS_NOTE] = v->note * 65536 / 127;
    src[SYS_RAND] = v->rnd;
    for (int m = 0; m < 4; ++m)
        src[SYS_MACRO1 + m] = s->knob[SP_MACRO1 + m];
    src[SYS_MODWHEEL] = s->knob[SP_MODWHEEL];
    src[SYS_AFTERTOUCH] = s->knob[SP_AFTERTOUCH];
    /* the mod matrix: knob deltas */
    int32_t K[SP_NUM];
    memcpy(K, s->knob, sizeof(K));
    for (int i = 0; i < 8; ++i) {
        const int sr = ival(s, SP_MOD1SRC + 3 * i), d = ival(s, SP_MOD1DST + 3 * i);
        if (sr <= 0 || sr >= SYS_NUM || d <= 0 || d > s->ndst)
            continue;
        const int32_t amt = lin_q16(SP_MOD1AMT + 3 * i, s->knob[SP_MOD1AMT + 3 * i]);
        K[s->dst_param[d]] += mul16(amt, src[sr]);
    }
    for (int d = 1; d <= s->ndst; ++d)
        K[s->dst_param[d]] = clampi(K[s->dst_param[d]], 0, 65536);

    /* pitch: the note (glided), bend, the pitch parameter */
    const int32_t bend = (int32_t)(((int64_t)s->bend * ival(s, SP_BENDRANGE)) << 16 >> 13);
    const int32_t pitch = v->pitch + bend + lin_q16(SP_PITCH, K[SP_PITCH]);
    const int filt = ival(s, SP_FILT_ON);
    int xm[2];
    for (int o = 0; o < 2; ++o)
        xm[o] = is_xmod(ival(s, SP_A_WARP + o * OSTRIDE)) || is_xmod(ival(s, SP_A_WARP2 + o * OSTRIDE));
    const int bfirst = xm[0] && !xm[1];
    const int sub_oct = ival(s, SP_SUB_OCT);

    for (int o = 0; o < 2; ++o) {
        const int b = o * OSTRIDE, oi = 2 * vi + o;
        const struct sy_table* T = s->osc_tab[o];
        if (!ival(s, SP_A_ON + b) || !T) {
            W(s, A_OSC(oi, 15), 0);
            W(s, A_OSC(oi, 10), 0);
            continue;
        }
        const int32_t semis = pitch + ((12 * ival(s, SP_A_OCT + b) + ival(s, SP_A_SEMI + b)) << 16) +
                              lin_q16(SP_A_FINE + b, K[SP_A_FINE + b]) / 100;
        const int uni = clampi(ival(s, SP_A_UNISON + b), 1, 4);
        const int32_t dk = K[SP_A_DETUNE + b], det = mul16(mul16(dk, dk), 2 << 16); /* semitones Q16 */
        const uint32_t inc0 = pitch_inc(s, semis);
        for (int u = 0; u < 4; ++u) {
            uint32_t inc = inc0;
            int32_t g = 0;
            if (u < uni) {
                if (uni > 1) {
                    const int32_t sp = (int32_t)((2 * u * 65536) / (uni - 1)) - 65536;
                    inc = pitch_inc(s, semis + mul16(sp, det));
                }
                const int centre = uni <= 2 || (uni == 3 ? u == 1 : (u == 1 || u == 2));
                const int32_t bl = K[SP_A_BLEND + b];
                g = uni <= 2 ? 65536 : centre ? clampi((65536 - bl) * 4, 0, 65536) : clampi(bl * 4 / 3, 0, 65536);
                g = mul16(g, g_unorm[uni]);
            }
            W(s, A_OSC(oi, u), inc);
            W(s, A_OSC(oi, 4 + u), (uint32_t)g & 0x3FFFF);
        }
        /* warps */
        const int w1 = ival(s, SP_A_WARP + b), w2 = ival(s, SP_A_WARP2 + b);
        const struct warp r1 = warp_regs(w1, K[SP_A_WARPAMT + b], sub_oct), r2 = warp_regs(w2, K[SP_A_WARP2AMT + b], sub_oct);
        W(s, A_OSC(oi, 11), r1.par);
        W(s, A_OSC(oi, 12), r2.par);
        W(s, A_OSC(oi, 13), r1.parb);
        W(s, A_OSC(oi, 14), r2.parb);
        W(s, A_OSC(oi, 17), r1.parc);
        W(s, A_OSC(oi, 18), r2.parc);
        /* level: the warps' slope, phase modulation, the unison spread */
        int64_t fmax = ((int64_t)inc0 * mul16(r1.slope, r2.slope)) >> 16;
        fmax += ((int64_t)inc0 * (r1.pm + r2.pm)) >> 16;
        if (uni > 1)
            fmax = (fmax * exp2_q16(det / 12)) >> 16;
        if (fmax > 0x7FFFFFFF) fmax = 0x7FFFFFFF;
        const int lev = choose_level(s, (uint32_t)fmax, note_on ? 0 : v->lev[o]);
        v->lev[o] = lev;
        const int logn = lev <= 2 ? 11 - lev : 8;
        W(s, A_OSC(oi, 9), T->lvl[lev] | (uint32_t)logn << 25);
        W(s, A_OSC(oi, 16), (uint32_t)(T->frames - 1));
        W(s, A_OSC(oi, 8), (uint32_t)(((T->frames - 1) * (int64_t)K[SP_A_POS + b])) | (uint32_t)ival(s, SP_A_SMOOTH + b) << 24);
        W(s, A_OSC(oi, 15), (uint32_t)K[SP_A_LEVEL + b] & 0x3FFFF);
        uint32_t mode = (uint32_t)w1 | (uint32_t)w2 << 5 | (uint32_t)(uni - 1) << 10 |
                        (uint32_t)(filt && ival(s, SP_A_FILT + b)) << 12 | 1u << 14;
        if (note_on) {
            const int32_t rnd = s->knob[SP_A_RAND + b];
            if (rnd < 64880) { /* below ~0.99: the unison phases restart */
                for (int u = 0; u < 4; ++u) {
                    const uint32_t ph = (uint32_t)s->knob[SP_A_PHASE + b] + (uint32_t)mul16(rnd, (int32_t)(rnd32(s) >> 16));
                    s->valid[A_OSC(oi, 20 + u) >> 3] &= (uint8_t)~(1 << (A_OSC(oi, 20 + u) & 7));
                    W(s, A_OSC(oi, 20 + u), (ph << 2) & 0x3FFFF);
                }
                mode |= 1u << 15;
            }
            mode |= 1u << 13;
            s->valid[A_OSC(oi, 10) >> 3] &= (uint8_t)~(1 << (A_OSC(oi, 10) & 7)); /* always written */
        }
        W(s, A_OSC(oi, 10), mode);
    }
    /* sub and noise */
    W(s, A_VOI(vi, 0), pitch_inc(s, pitch + (sub_oct * 12 << 16)));
    W(s, A_VOI(vi, 1), (ival(s, SP_SUB_ON) ? ((uint32_t)K[SP_SUB_LEVEL] & 0x3FFFF) : 0) | (uint32_t)ival(s, SP_SUB_SHAPE) << 20);
    W(s, A_VOI(vi, 2), ival(s, SP_NOISE_ON) ? (uint32_t)K[SP_NOISE_LEVEL] & 0x3FFFF : 0);
    W(s, A_VOI(vi, 3), (uint32_t)(filt && ival(s, SP_SUB_FILT)) | (uint32_t)(filt && ival(s, SP_NOISE_FILT)) << 1 |
                           (uint32_t)bfirst << 2);
    /* filter: four slots, the XY morph's equal-power weights */
    if (filt) {
        const int32_t oct0 = (int32_t)(((int64_t)K[SP_CUTOFF] * 653118) >> 16) /* log2(1000) */ +
                             (int32_t)(((int64_t)lin_q16(SP_FILTKEY, s->knob[SP_FILTKEY]) * (pitch - (60 << 16)) / 12) >> 16);
        const int32_t res = mul16(6554, (int32_t)exp2_q16((int32_t)(((int64_t)K[SP_RES] * 435412) >> 16))); /* 0.1 x 100^k */
        const int32_t drive = 65536 + 7 * K[SP_DRIVE];
        for (int t = 0; t < 4; ++t) {
            const int32_t oct = oct0 + lin_q16(SP_SLOT1OCT + t, s->knob[SP_SLOT1OCT + t]);
            const int32_t key0 = s->slot_model[t] | ival(s, SP_SLOT1TYPE + t) << 4;
            int32_t* fk = v->fkey[t];
            if (fk[0] == key0 && fk[1] == oct && fk[2] == res && fk[3] == drive)
                continue;
            fk[0] = key0, fk[1] = oct, fk[2] = res, fk[3] = drive;
            struct sy_fslot f = {s->slot_model[t], ival(s, SP_SLOT1TYPE + t), gtab(oct), res, drive};
            uint8_t reg[16];
            int32_t val[16];
            const int n = sy_filt_coefs(&f, 2 * s->fs_q16, reg, val);
            for (int i = 0; i < n; ++i)
                W(s, A_FIN(t, vi, reg[i]), (uint32_t)val[i]);
        }
        const int32_t x = K[SP_MORPHX], y = K[SP_MORPHY];
        const int32_t cx = sinq(65536 - x), sx = sinq(x), cy = sinq(65536 - y), sy = sinq(y);
        W(s, A_W(vi, 0), (uint32_t)mul16(cx, cy) & 0x3FFFF);
        W(s, A_W(vi, 1), (uint32_t)mul16(sx, cy) & 0x3FFFF);
        W(s, A_W(vi, 2), (uint32_t)mul16(cx, sy) & 0x3FFFF);
        W(s, A_W(vi, 3), (uint32_t)mul16(sx, sy) & 0x3FFFF);
    }
    /* gain: envelope 1 x velocity x volume */
    const int32_t amp = mul16(mul16(mul16(v->env[0].v >> 8, v->vel), K[SP_VOLUME]), TRIM);
    W(s, A_AMP(vi), (uint32_t)amp & 0x3FFFF);
}

void sy_tick(struct sy_state* s) {
    ++s->ticks;
    for (int vi = 0; vi < SY_VOICES; ++vi) {
        struct sy_voice* v = &s->v[vi];
        if (!v->active) {
            if (!v->idle_written)
                voice_idle(s, vi);
            continue;
        }
        ++v->age;
        if (v->pitch != v->target) { /* portamento: exponential, ~99 % in the glide time */
            const int32_t d = (int32_t)(((int64_t)(v->pitch - v->target) * s->porta_k) >> 30);
            v->pitch = (d > -4 && d < 4) ? v->target : v->target + d;
        }
        for (int e = 0; e < 3; ++e)
            env_step(&s->envp[e], &v->env[e]);
        for (int l = 0; l < 4; ++l) {
            const int b = SP_LFO1SHAPE + 7 * l;
            struct sy_lfo* L = &v->lfo[l];
            ++L->age;
            const int mode = ival(s, b + 3);
            if (L->age <= s->lfo_delay[l] || (mode == 1 && L->done))
                continue;
            const uint32_t np = L->ph + s->lfo_inc[l];
            if (np < L->ph) { /* wrapped */
                L->held = (int32_t)(rnd32(s) >> 16);
                if (mode == 1) {
                    L->done = 1;
                    L->ph = 0xFFFFFFFFu;
                    continue;
                }
            }
            L->ph = np;
        }
        voice_update(s, vi, 0);
        if (!v->gate && v->env[0].st == 0) {
            v->active = 0;
            voice_idle(s, vi);
        }
    }
}
