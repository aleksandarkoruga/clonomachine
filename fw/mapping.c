/* Event -> slot mapping: Monomachine-style table plus the
 * RAW_PARAM namespace (/slot/<n>/<param>). MIDI channels are SC's 0-based numbers. */
#include "mapping.h"

#include <stdio.h>

#include "engine.h"
#include "samples.h"
#include "slot.h"

/* ---- boot-time LUTs (soft-float only here) ---------------------------------------- */
static double d_exp(double x) {
    /* e^x = 2^k * e^r, |r| <= ln2/2 */
    const double ln2 = 0.69314718055994530942;
    int k = (int)(x / ln2 + (x >= 0 ? 0.5 : -0.5));
    double r = x - k * ln2, term = 1, sum = 1;
    for (int i = 1; i < 18; ++i) {
        term *= r / i;
        sum += term;
    }
    while (k > 0) { sum *= 2; --k; }
    while (k < 0) { sum *= 0.5; ++k; }
    return sum;
}
static double d_log(double x) {
    /* ln x = 2 atanh((x-1)/(x+1)) after scaling into [0.5, 2) */
    const double ln2 = 0.69314718055994530942;
    int k = 0;
    while (x >= 2) { x *= 0.5; ++k; }
    while (x < 0.5) { x *= 2; --k; }
    const double y = (x - 1) / (x + 1), y2 = y * y;
    double t = y, sum = 0;
    for (int i = 1; i < 60; i += 2) {
        sum += t / i;
        t *= y2;
    }
    return 2 * sum + k * ln2;
}
static double bilin(double v, double inC, double inMin, double inMax, double outC, double outMin, double outMax) {
    if (v <= inC)
        return outMin + (v - inMin) * (outC - outMin) / (inC - inMin);
    return outC + (v - inC) * (outMax - outC) / (inMax - inC);
}
static double linexp(double v, double lo, double hi) { return lo * d_exp(d_log(hi / lo) * v / 127.0); }
static int32_t q16(double v) { return (int32_t)(v * 65536.0 + (v >= 0 ? 0.5 : -0.5)); }

static int32_t lut_rate[128], lut_pitch[128], lut_grain[128], lut_overlap[128], lut_randpitch[128];
static int32_t lut_interval[129]; /* bend: lincurve(0..1 -> 1e-6..0.25 s, curve 5), 128 steps */

void mapping_init(void) {
    for (int v = 0; v < 128; ++v) {
        double r = bilin(v, 63, 0, 127, 1, 0, 4);
        lut_rate[v] = q16(r < 0 ? 0 : r > 4 ? 4 : r);
        lut_pitch[v] = q16(bilin(v, 63, 0, 127, 1, 0.1, 4));
        lut_grain[v] = q16(linexp(v, 0.001, 1.0));
        lut_overlap[v] = q16(linexp(v, 0.25, 16.0));
        lut_randpitch[v] = q16(bilin(v, 63, 0, 127, 0, -2, 2));
    }
    const double grow = d_exp(5.0), a = (0.25 - 1e-6) / (1.0 - grow), b = 1e-6 + a;
    for (int i = 0; i <= 128; ++i)
        lut_interval[i] = q16(b - a * d_exp(5.0 * i / 128.0));
}

/* ---- helpers ------------------------------------------------------------------------ */
static int chan_to_slot(int chan) {
    if (chan == 9)
        return 0;
    if (chan == 10)
        return NUM_SLOTS > 1 ? 1 : -1;
    return -1;
}

static void cc(int s, int c, int v) {
    struct slot* sl = &g_slots[s];
    switch (c) {
    case 1: slot_select(s, v); break;                                            /* bufNum */
    case 2: sl->rate_q16 = lut_rate[v]; slot_apply_tempo(s); break;
    case 7: sl->pitch_q16 = lut_pitch[v]; slot_apply_tempo(s); break;
    case 10: sl->env_abs = 0; sl->env_bars_q16 = (v + 1) * 2048; slot_apply_tempo(s); break; /* (cc+1)/32 bar */
    case 12: sl->graindur_q16 = lut_grain[v]; slot_apply_grain(s); break;
    case 13: sl->overlap_q16 = lut_overlap[v]; slot_apply_grain(s); break;
    case 14: sl->prob_q16 = (v * 65536) / 127; slot_apply_grain(s); break;
    case 15: sl->separation_q16 = (v * 65536) / 127; slot_apply_grain(s); break;
    case 16: sl->randpos_q16 = (v * 65536) / 127; slot_apply_grain(s); break;
    case 17: sl->randompitch_q16 = lut_randpitch[v]; slot_apply_grain(s); break;
    case 18: sl->amp_q16 = (v * 65536) / 127; slot_apply_output(s); break;
    case 20: sl->tempo_lock = v >= 64; slot_apply_tempo(s); break;             /* proposal */
    case 21: sl->bg_advance = v >= 64; slot_apply_flags(s); break;             /* proposal */
    default: break;
    }
}

static int32_t interval_from_bend(int32_t v14) {
    /* SC: val/16256 clipped to 0..1 by lincurve */
    int64_t x = ((int64_t)v14 << 16) / 16256; /* Q16 */
    if (x < 0) x = 0;
    if (x > 65536) x = 65536;
    const int i = (int)(x >> 9), f = (int)(x & 511);
    if (i >= 128)
        return lut_interval[128];
    return lut_interval[i] + (int32_t)(((int64_t)(lut_interval[i + 1] - lut_interval[i]) * f) >> 9);
}

static void set_interval(int s, int32_t v) {
    g_slots[s].interval_q16 = v;
    const uint32_t n = (uint32_t)(((uint64_t)(v > 0 ? v : 0) * eng_fs_q16()) >> 32);
    eng_slot_write(s, SMP_R_INTERVAL, n < 1 ? 1 : n > 12000 ? 12000 : n);
}

static void raw_param(int s, int p, int32_t v) {
    struct slot* sl = &g_slots[s];
    switch (p) {
    case P_TRIG: slot_trig(s); break;
    case P_BUFNUM: slot_select(s, v); break;
    case P_BANK: slot_select_bank(s, v); break;
    case P_POS: slot_set_pos(s, v); break;
    case P_RATE: sl->rate_q16 = v; slot_apply_tempo(s); break;
    case P_PITCH: sl->pitch_q16 = v; slot_apply_tempo(s); break;
    case P_GRAINDUR: sl->graindur_q16 = v; slot_apply_grain(s); break;
    case P_OVERLAP: sl->overlap_q16 = v; slot_apply_grain(s); break;
    case P_PROB: sl->prob_q16 = v; slot_apply_grain(s); break;
    case P_SEPARATION: sl->separation_q16 = v; slot_apply_grain(s); break;
    case P_RANDPOS: sl->randpos_q16 = v; slot_apply_grain(s); break;
    case P_RANDOMPITCH: sl->randompitch_q16 = v; slot_apply_grain(s); break;
    case P_TEMPOLOCK: sl->tempo_lock = !!v; slot_apply_tempo(s); break;
    case P_BGADVANCE: sl->bg_advance = !!v; slot_apply_flags(s); break;
    case P_MAXGRAINS: sl->max_grains = v; slot_apply_grain(s); break;
    case P_SEED: sl->seed = (uint32_t)v; eng_slot_write(s, SMP_R_SEED, (uint32_t)v); break;
    case P_ENVDUR: sl->env_abs = 1; sl->env_sec_q16 = v; slot_apply_tempo(s); break;
    case P_ENVBARS: sl->env_abs = 0; sl->env_bars_q16 = v; slot_apply_tempo(s); break;
    case P_ENVHOLD: sl->env_hold = !!v; slot_apply_flags(s); break;
    case P_GATEOFF: eng_slot_write(s, SMP_R_STROBE, SMP_STROBE_GATE_OFF); break;
    case P_REPEATS: sl->repeats = v; eng_slot_write(s, SMP_R_REPEATS, (uint32_t)(v < 0 ? 0 : v > 126 ? 126 : v)); break;
    case P_INTERVAL: set_interval(s, v); break;
    case P_AMP: sl->amp_q16 = v; slot_apply_output(s); break;
    case P_ALGOAMP: sl->algo_q16 = v; slot_apply_output(s); break;
    case P_KILL: eng_slot_write(s, SMP_R_STROBE, SMP_STROBE_KILL); break;
    case P_ENABLE: sl->enabled = !!v; slot_apply_flags(s); break;
    default: break;
    }
}

void mapping_event(const struct smp_event* e) {
    const int s = chan_to_slot(e->chan);
    switch (e->type) {
    case EV_NOTE_ON:
        if (s >= 0) {
            if (e->b == 0)
                slot_note_off(s);   /* running-status note-off */
            else
                slot_note_on(s, e->a, e->b);
        }
        break;
    case EV_NOTE_OFF:
        if (s >= 0)
            slot_note_off(s);
        break;
    case EV_CC:
        if (s >= 0)
            cc(s, e->a, e->b & 127);
        else if (e->chan == 14 && e->a == 1) { /* algoAmp = round(val/128) */
            g_slots[0].algo_q16 = (e->b >= 64) ? 65536 : 0;
            slot_apply_output(0);
        }
        break;
    case EV_PC:
        if (s >= 0)
            slot_select_bank(s, e->a & 127); /* program change = bank */
        break;
    case EV_BEND:
        if (s >= 0) {
            set_interval(s, interval_from_bend(e->value));
        }
        break;
    case EV_BPM:
        set_global_bpm((uint32_t)e->value);
        break;
    case EV_START: /* beat counter and every background playhead restart together */
        eng_write(SMP_ADDR(SMP_GLOBAL_SLOT, SMP_G_BEAT_CTRL), 1);
        for (int i = 0; i < NUM_SLOTS; ++i)
            eng_slot_write(i, SMP_R_STROBE, SMP_STROBE_BG_RESET);
        break;
    case EV_CLOCK:
    case EV_STOP:
        break; /* external clock mode: phase 2 */
    case EV_RAW_PARAM:
        if (e->a < NUM_SLOTS)
            raw_param(e->a, e->b, e->value);
        break;
    case EV_LOAD_BUF:
        if (e->a < NUM_SLOTS)
            slot_select(e->a, e->b); /* index inside the slot's bank */
        break;
    default:
        break;
    }
}
