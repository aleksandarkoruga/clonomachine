/* Event -> slot mapping: Monomachine-style table plus the
 * RAW_PARAM namespace (/slot/<n>/<param>). MIDI channels are SC's 0-based numbers. */
#include "mapping.h"

#include "esplink.h"

#include <stdio.h>

#include "engine.h"
#include "samples.h"
#include "slot.h"
#include "midimap.h"

/* ---- boot-time LUTs (soft-float only here) ---------------------------------------- */
#include "dmath.h"
static double bilin(double v, double inC, double inMin, double inMax, double outC, double outMin, double outMax) {
    if (v <= inC)
        return outMin + (v - inMin) * (outC - outMin) / (inC - inMin);
    return outC + (v - inC) * (outMax - outC) / (inMax - inC);
}
static double linexp(double v, double lo, double hi) { return lo * d_exp(d_log(hi / lo) * v / 127.0); }
static int32_t q16(double v) { return (int32_t)(v * 65536.0 + (v >= 0 ? 0.5 : -0.5)); }

static int32_t lut_rate[128], lut_pitch[128], lut_grain[128], lut_overlap[128], lut_randpitch[128];
static int32_t lut_interval[129]; /* bend: lincurve(0..1 -> 1e-6..0.25 s, curve 5), 128 steps */
static int32_t lut_rate_bi[128], lut_amp[128], lut_graindur_at[128], lut_gate[128]; /* Machinedrum pages */

void mapping_init(void) {
    for (int v = 0; v < 128; ++v) {
        double r = bilin(v, 63, 0, 127, 1, 0, 4);
        lut_rate[v] = q16(r < 0 ? 0 : r > 4 ? 4 : r);
        lut_pitch[v] = q16(bilin(v, 63, 0, 127, 1, 0.1, 4));
        lut_grain[v] = q16(linexp(v, 0.001, 1.0));
        lut_overlap[v] = q16(linexp(v, 0.25, 16.0));
        lut_randpitch[v] = q16(bilin(v, 63, 0, 127, 0, -2, 2));
        lut_rate_bi[v] = q16(bilin(v, 63, 0, 127, 1, -2, 2));
        lut_amp[v] = q16(bilin(v, 63, 0, 127, 1, 0, 2));
        lut_gate[v] = q16(linexp(v, 1.0 / 32, 16.0)); /* bars: 1/32 .. 16, exponential */
        lut_graindur_at[v] = q16(linexp(v, 0.005, 0.5));
    }
    const double grow = d_exp(5.0), a = (0.25 - 1e-6) / (1.0 - grow), b = 1e-6 + a;
    for (int i = 0; i <= 128; ++i)
        lut_interval[i] = q16(b - a * d_exp(5.0 * i / 128.0));
}

/* ---- helpers ------------------------------------------------------------------------ */
static int chan_to_slot(int chan) {
    if (chan == 9)
        return 0;
    if (chan >= 10 && chan < 9 + NUM_SLOTS) /* 10: slot 1, 11: slot 2 */
        return chan - 9;
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
    case 22: sl->algo = v >= 64 ? 1 : 0; slot_apply_algo(s); break;            /* proposal */
    case 23: sl->note_mode = v / 43; break;                                     /* proposal: 0/1/2 */
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
    case P_ALGO: sl->algo = v < 0 ? 0 : v > 1 ? 1 : v; slot_apply_algo(s); break;
    case P_PVTRANSIENT: sl->pv_transient = !!v; slot_apply_algo(s); break;
    case P_NOTEMODE: sl->note_mode = v < 0 ? 0 : v > 2 ? 2 : v; break;
    case P_KILL: eng_slot_write(s, SMP_R_STROBE, SMP_STROBE_KILL); break;
    case P_ENABLE: sl->enabled = !!v; slot_apply_flags(s); break;
    case P_OUT: sl->out = v < 0 ? 0 : v > 2 ? 2 : v; slot_apply_output(s); break;
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
                slot_note_on(s, e->a, e->b - 1); /* velocity - 1 = stutter repeats */
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
        esplink_transport(1);
        eng_write(SMP_ADDR(SMP_GLOBAL_SLOT, SMP_G_BEAT_CTRL), 1);
        for (int i = 0; i < NUM_SLOTS; ++i)
            eng_slot_write(i, SMP_R_STROBE, SMP_STROBE_BG_RESET);
        break;
    case EV_STOP: /* transport stop: every voice's gate closes (1 ms ramp) */
        esplink_transport(0);
        for (int i = 0; i < NUM_SLOTS; ++i)
            eng_slot_write(i, SMP_R_STROBE, SMP_STROBE_GATE_OFF);
        break;
    case EV_CLOCK: /* MIDI clock arrives through mapping_midi() (tempo follow) */
        break;
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

/* ---- USB MIDI pages: /midimap.json on the SD card (fw/midimap.h) -------------------------
 * A page = one MIDI channel driving one voice: consecutive CCs from the page's first one
 * drive its knobs, mod wheel / aftertouch / pitch bend their targets, notes trigger (16ths of
 * the loop / transients; the velocity drives the page's "velocity" control, the volume by
 * default), a note-off ends the gate (only for
 * the note that plays). Other channels go through the Monomachine table of mapping_event. */
static struct midimap g_mm;
static struct {
    int32_t lut_rate[128];
    int last_note;
    int8_t last[MT_NUM]; /* last value per control, -1: never moved (cutter handover) */
} g_pg[MM_MAX_PAGES];
int g_md_slot0_external;
int g_clock_follow = 1;
static uint32_t g_midi_count;
/* what the SC cutter sets itself while it plays voice 0: the page's values wait */
/* what the cutter sets per hit while it plays voice 0; volume (velocity) and gate stay MIDI's */
static const uint8_t g_cutter_owns[MT_NUM] = {[MT_BUFNUM] = 1, [MT_RATE] = 1, [MT_PITCH] = 1,
                                              [MT_INTERVAL] = 1, [MT_AMP] = 1};

static void pages_prepare(void) {
    for (int i = 0; i < g_mm.npages; ++i) {
        const double lo = g_mm.page[i].rate_lo / 65536.0, hi = g_mm.page[i].rate_hi / 65536.0;
        for (int v = 0; v < 128; ++v)
            g_pg[i].lut_rate[v] = q16(bilin(v, 63, 0, 127, 1, lo, hi));
        g_pg[i].last_note = -1;
        for (int t = 0; t < MT_NUM; ++t)
            g_pg[i].last[t] = -1;
    }
    g_clock_follow = g_mm.clock_follow;
}

void mapping_midimap_load(void) {
    static char buf[4096];
    char err[96];
    const int n = samples_read_file("/midimap.json", buf, sizeof(buf));
    if (n < 0) {
        midimap_defaults(&g_mm);
        const int len = midimap_write(&g_mm, buf, sizeof(buf));
        printf("MIDI: /midimap.json %s\n",
               samples_write_file("/midimap.json", buf, len) == 0 ? "written with the defaults (edit it on a PC)"
                                                                   : "not on the card: defaults");
    } else if (midimap_parse(&g_mm, buf, n, err, sizeof(err)) != 0)
        printf("MIDI: /midimap.json: %s -- using the defaults\n", err);
    else
        printf("MIDI: /midimap.json: %d pages\n", g_mm.npages);
    pages_prepare();
}

/* console "midimap defaults": the current file goes to /midimap.json.bak, the defaults replace it */
void mapping_midimap_defaults(void) {
    static char buf[4096];
    const int n = samples_read_file("/midimap.json", buf, sizeof(buf));
    if (n > 0 && samples_write_file("/midimap.json.bak", buf, n) != 0) {
        printf("MIDI: could not save /midimap.json.bak, nothing changed\n");
        return;
    }
    midimap_defaults(&g_mm);
    const int len = midimap_write(&g_mm, buf, sizeof(buf));
    printf("MIDI: /midimap.json %s\n", samples_write_file("/midimap.json", buf, len) == 0
                                           ? "now the defaults (the old one is /midimap.json.bak)"
                                           : "could not be written (defaults in use until reboot)");
    pages_prepare();
}

void mapping_midimap_print(void) {
    static char buf[4096];
    midimap_write(&g_mm, buf, sizeof(buf));
    fputs(buf, stdout);
}

static void md_set_external(int on);

/* one control of page pi to value v (0..127) */
static void apply(int pi, int t, int v) {
    const int s = g_mm.page[pi].voice;
    if (t <= MT_NONE || t >= MT_NUM || s >= NUM_SLOTS)
        return;
    g_pg[pi].last[t] = (int8_t)v;
    if (t == MT_BUFNUM && s == 0)
        esplink_base_buf(v); /* the cutter plays from the sample this knob picks */
    if (t == MT_CUTTER) {
        if (s == 0)
            md_set_external(v < 64);
        return;
    }
    if (s == 0 && g_md_slot0_external && g_cutter_owns[t])
        return;
    switch (t) {
    case MT_BUFNUM: raw_param(s, P_BUFNUM, v); break;
    case MT_BANK: raw_param(s, P_BANK, v); break;
    case MT_RATE: raw_param(s, P_RATE, g_pg[pi].lut_rate[v]); break;
    case MT_PITCH: raw_param(s, P_PITCH, lut_pitch[v]); break;
    case MT_GATE: raw_param(s, P_ENVBARS, lut_gate[v]); break;                    /* 1/32 .. 16 bars */
    case MT_INTERVAL: raw_param(s, P_INTERVAL, interval_from_bend(v * 128)); break; /* v/127 of the curve */
    case MT_AMP: raw_param(s, P_AMP, lut_amp[v]); break;
    case MT_OVERLAP: raw_param(s, P_OVERLAP, lut_overlap[v]); break;
    case MT_RANDPOS: raw_param(s, P_RANDPOS, (int32_t)(((int64_t)v * v * 65536 + 8064) / 16129)); break;
    case MT_RANDOMPITCH: raw_param(s, P_RANDOMPITCH, lut_randpitch[v]); break;
    case MT_PROB: raw_param(s, P_PROB, (v * 65536 + 63) / 127); break;
    case MT_SEPARATION: raw_param(s, P_SEPARATION, (v * 65536 + 63) / 127); break;
    case MT_MAXGRAINS: raw_param(s, P_MAXGRAINS, (v * 31 + 63) / 127 + 1); break;
    case MT_TEMPOLOCK: raw_param(s, P_TEMPOLOCK, v >= 64); break;
    case MT_BGADVANCE: raw_param(s, P_BGADVANCE, v >= 64); break;
    case MT_ENVHOLD: raw_param(s, P_ENVHOLD, v >= 64); break;
    case MT_PVTRANSIENT: raw_param(s, P_PVTRANSIENT, v >= 64); break;
    case MT_ALGO: raw_param(s, P_ALGO, v >= 64); break;
    case MT_GRAINDUR: raw_param(s, P_GRAINDUR, lut_graindur_at[v]); break;
    case MT_NOTEMODE: raw_param(s, P_NOTEMODE, v * 3 / 128); break;
    case MT_OUT: raw_param(s, P_OUT, v * 3 / 128); break;
    case MT_REPEATS: raw_param(s, P_REPEATS, v > 126 ? 126 : v); break;
    case MT_FXSEND: break; /* reverb send: not in the gateware yet */
    case MT_VOLUME: g_slots[s].vol_q16 = lut_amp[v]; slot_apply_output(s); break;
    default: break;
    }
}

static void md_set_external(int on) {
    if (on == g_md_slot0_external)
        return;
    g_md_slot0_external = on;
    printf("voice 0: %s\n", on ? "played by the cutter, its MIDI notes ignored" : "MIDI");
    esplink_owner(0, on);
    if (on) { /* no MIDI stutter left over */
        raw_param(0, P_REPEATS, 0);
        return;
    }
    for (int pi = 0; pi < g_mm.npages; ++pi) /* the controls the cutter overrode come back */
        if (g_mm.page[pi].voice == 0)
            for (int t = 0; t < MT_NUM; ++t)
                if (g_cutter_owns[t] && g_pg[pi].last[t] >= 0)
                    apply(pi, t, g_pg[pi].last[t]);
}

/* MIDI clock: tempo over the last 96 ticks (4 beats), applied when it moves by > 0.1 bpm */
static uint32_t g_clk_ring[128], g_clk_x[128], g_clk_dt[15], g_clk_n, g_clk_last, g_clock_bpm_milli;

/* t: when the tick arrived (engine samples). Every tick gets its index in the stream: one more
   than the last, plus the ticks the interface lost in between (a cheap non-isolated cable drops
   out while the board's audio plays), counted from the interval with the median of the last 15
   intervals as the period. Once a beat the tempo is the least-squares slope of arrival time over
   index for the last 96 ticks: missing ticks don't slow it down, and a tick that was picked up a
   few ms late (USB serviced between SD reads) hardly moves it. */
static void clock_tick(uint32_t t) {
    if (g_clk_n && t - g_clk_last > 48000) /* a gap of more than 1 s: start over */
        g_clk_n = 0;
    const uint32_t n = g_clk_n++;
    uint32_t x = 0;
    if (n) {
        const uint32_t dt = t - g_clk_last;
        g_clk_dt[(n - 1) % 15] = dt;
        x = g_clk_x[(n - 1) & 127] + 1;
        if (n >= 15) {
            uint32_t m[15];
            for (int i = 0; i < 15; ++i) { /* insertion sort */
                int j = i;
                for (; j > 0 && m[j - 1] > g_clk_dt[i]; --j)
                    m[j] = m[j - 1];
                m[j] = g_clk_dt[i];
            }
            const uint32_t per = m[7];
            const uint32_t k = per ? (dt + per / 2) / per : 1;
            if (k > 1)
                x += k - 1;
        }
    }
    g_clk_last = t;
    g_clk_ring[n & 127] = t;
    g_clk_x[n & 127] = x;
    esplink_midi_tick(x);
    if (n < 47 || (n + 1) % 24)
        return;
    const uint32_t m = n + 1 < 96 ? n + 1 : 96, i0 = n + 1 - m;
    const uint32_t x0 = g_clk_x[i0 & 127], t0 = g_clk_ring[i0 & 127];
    int64_t sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (uint32_t i = i0; i <= n; ++i) {
        const int64_t xi = g_clk_x[i & 127] - x0, yi = g_clk_ring[i & 127] - t0;
        sx += xi;
        sy += yi;
        sxx += xi * xi;
        sxy += xi * yi;
    }
    const int64_t den = (int64_t)m * sxx - sx * sx, num = (int64_t)m * sxy - sx * sy;
    if (den <= 0 || num <= 0)
        return;
    const uint64_t per_q16 = ((uint64_t)num << 16) / (uint64_t)den; /* samples per tick */
    if (!per_q16)
        return;
    uint32_t bpm = (uint32_t)((uint64_t)2500 * eng_fs_q16() / per_q16); /* 60 fs / 24 per, milli */
    bpm = (bpm + 5) / 10 * 10;
    g_clock_bpm_milli = bpm;
    const int32_t d = (int32_t)(bpm - g_bpm_milli);
    if (g_clock_follow && (d > 100 || d < -100))
        set_global_bpm(bpm);
}

void mapping_midi(uint8_t st, uint8_t d1, uint8_t d2) {
    mapping_midi_at(st, d1, d2, eng_status(SMP_SG_SAMPLES));
}

void mapping_midi_at(uint8_t st, uint8_t d1, uint8_t d2, uint32_t t) {
    ++g_midi_count;
    struct smp_event e = {0};
    if (st >= 0xF8) { /* realtime */
        if (st == 0xF8)
            clock_tick(t);
        else if (st == 0xFA) { /* start: beat and wall clock from zero */
            g_clk_n = 0;
            e.type = EV_START;
            mapping_event(&e);
        } else if (st == 0xFB) { /* continue: nothing to reset */
        } else if (st == 0xFC) {
            e.type = EV_STOP;
            mapping_event(&e);
        }
        return;
    }
    if (st < 0x80 || st >= 0xF0)
        return;
    const int type = st & 0xF0, chan = st & 0xF;
    if (g_mm.cutter_chan && chan == g_mm.cutter_chan - 1) { /* the cutter's channel: to the ESP */
        esplink_forward_midi(st, d1, d2);
        return;
    }
    int pi = -1;
    for (int i = 0; i < g_mm.npages && pi < 0; ++i)
        if (g_mm.page[i].chan == chan)
            pi = i;
    if (pi < 0) { /* not a page: the Monomachine table (chan 9 / 10 / 11, 14) */
        e.chan = (uint8_t)chan;
        e.a = d1;
        e.b = d2;
        switch (type) {
        case 0x90: e.type = EV_NOTE_ON; break;
        case 0x80: e.type = EV_NOTE_OFF; break;
        case 0xB0: e.type = EV_CC; break;
        case 0xC0: e.type = EV_PC; break;
        case 0xE0: e.type = EV_BEND; e.value = d1 | (d2 << 7); break;
        default: return;
        }
        mapping_event(&e);
        return;
    }
    const struct mm_page* pg = &g_mm.page[pi];
    const int s = pg->voice;
    if (s >= NUM_SLOTS)
        return;
    const int ext = s == 0 && g_md_slot0_external;
    switch (type) {
    case 0x90:
        if (d2) {
            apply(pi, pg->vel, d2); /* velocity: volume by default, also while the cutter plays */
            if (!ext) {
                g_pg[pi].last_note = d1;
                slot_note_on(s, d1, -1);
            }
            break;
        }
        /* fall through: velocity 0 = note off */
    case 0x80: /* only the note that plays: a late note-off of the previous one doesn't cut */
        if (!ext && pg->note_off && g_pg[pi].last_note == d1) {
            g_pg[pi].last_note = -1;
            slot_note_off(s);
        }
        break;
    case 0xB0:
        if (d1 == 1 && pg->mw) {
            apply(pi, pg->mw, d2);
            break;
        }
        for (int i = pi; i < g_mm.npages; ++i) /* every page on this channel whose CCs cover d1 */
            if (g_mm.page[i].chan == chan && d1 >= g_mm.page[i].cc && d1 - g_mm.page[i].cc < g_mm.page[i].nknobs)
                apply(i, g_mm.page[i].knobs[d1 - g_mm.page[i].cc], d2);
        break;
    case 0xD0: /* channel aftertouch */
        apply(pi, pg->at, d1 & 127);
        break;
    case 0xE0: /* pitch bend, 14 bit -> 0..127 */
        apply(pi, pg->pb, ((d1 | (d2 << 7)) >> 7) & 127);
        break;
    default: /* program change, poly pressure */
        break;
    }
}

void mapping_midi_status(void) {
    printf("midi: %lu messages  clock ", (unsigned long)g_midi_count);
    if (g_clock_bpm_milli && eng_status(SMP_SG_SAMPLES) - g_clk_last < 48000)
        printf("%lu.%02lu bpm", (unsigned long)(g_clock_bpm_milli / 1000), (unsigned long)(g_clock_bpm_milli % 1000 / 10));
    else
        printf("none");
    printf(" (follow %s)  voice 0 %s  %d pages\n", g_clock_follow ? "on" : "off",
           g_md_slot0_external ? "external (cutter)" : "MIDI", g_mm.npages);
}

void mapping_set_cutter(int on) { md_set_external(!!on); }
