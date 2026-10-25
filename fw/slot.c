/* Slot parameter logic. All runtime maths is integer; see slot.h.
 *
 * Tempo (absolute mode):
 *   trackBpm   = bpm_of_length(frames, sr)       (samples.c, beats = 2^k)
 *   tempoRatio = globalBpm / trackBpm            (1 when tempo lock is off)
 *   PHASOR_INC = rate * tempoRatio * bufSR / fs  (fs = measured engine rate)
 *   gate       = envBars * 240/globalBpm s       (real time, the same for every loop)
 *   note->pos  = ((note-12) mod N16) / N16, N16 = 16 * nBars = 4 * 2^k sixteenths
 *
 * Buffer switches are seamless: the engine keeps every running grain on the buffer it
 * started in, and the playhead is moved (SEEK) to the same sixteenth of the bar in the
 * new loop, so a switch crossfades through the grain overlap without a gap.
 */
#include "slot.h"

#include <string.h>

#include "engine.h"
#include "samples.h"

struct slot g_slots[NUM_SLOTS];
uint32_t g_bpm_milli = 120000;

static uint32_t g_last_base[NUM_SLOTS];

static inline int32_t clampi(int64_t v, int64_t lo, int64_t hi) {
    return (int32_t)(v < lo ? lo : v > hi ? hi : v);
}

static int slot_lib(int s) {
    const int l = g_slots[s].lib;
    return (l >= 0 && l < MAX_LIBS && g_libs[l].count) ? l : 0;
}

static int wrap(int v, int n) {
    if (n <= 0)
        return -1;
    v %= n;
    return v < 0 ? v + n : v;
}

static int bank_count(int s, int bank) {
    const struct bank* b = samples_bank(slot_lib(s), bank);
    return b ? b->count : 0;
}

/* The buffer the engine is playing (resident). */
static const struct sample* cur_sample(int s) {
    const struct slot* sl = &g_slots[s];
    if (samples_base(slot_lib(s), sl->bank, sl->buf) == NOT_RESIDENT)
        return 0;
    return samples_get(slot_lib(s), sl->bank, sl->buf);
}

int samples_bank_keep(int lib, int bank) {
    int keep = 0;
    for (int s = 0; s < NUM_SLOTS; ++s) {
        const struct slot* sl = &g_slots[s];
        if (slot_lib(s) != lib)
            continue;
        if (sl->bank == bank || sl->want_bank == bank)
            return 2;
        const int nb = g_libs[lib].nbanks;
        if (nb > 1 && sl->bank >= 0 && bank == wrap(sl->bank + 1, nb))
            keep = 1;
    }
    return keep;
}

/* bufSR / fs as Q8.24 */
static uint32_t sr_factor_q24(const struct sample* smp) {
    return (uint32_t)((((uint64_t)smp->sr * ENGINE_CYCLES_PER_SAMPLE) << 24) / AUDIO_CLK_FREQ);
}

uint64_t bar_samples_q16(void) {
    /* 240 s / bpm * fs in Q16 samples: > 2^32 at every usable tempo, keep 64 bit */
    return ((uint64_t)240000 * eng_fs_q16()) / g_bpm_milli;
}

static uint32_t seconds_to_samples(int32_t sec_q16) {
    if (sec_q16 <= 0)
        return 0;
    return (uint32_t)(((uint64_t)sec_q16 * eng_fs_q16()) >> 32);
}

uint32_t slot_tempo_ratio_q24(int s) {
    const struct sample* smp = cur_sample(s);
    if (!smp || !g_slots[s].tempo_lock || !smp->bpm_q16)
        return 1u << 24;
    return (uint32_t)((((uint64_t)g_bpm_milli << 16) << 24) / ((uint64_t)smp->bpm_q16 * 1000));
}

static uint32_t sixteenths_of(const struct sample* smp) {
    if (!smp)
        return 16;
    if (smp->n16)
        return smp->n16;
    const int32_t k = smp->beats_log2;
    if (k < -2)
        return 1;
    if (k > 20)
        return 1u << 22;
    return 1u << (k + 2); /* 4 * 2^k */
}

uint32_t slot_sixteenths(int s) { return sixteenths_of(cur_sample(s)); }

void slots_init(void) {
    memset(g_slots, 0, sizeof(g_slots));
    for (int s = 0; s < NUM_SLOTS; ++s) {
        struct slot* sl = &g_slots[s];
        sl->lib = s == 0 ? 0 : 1;       /* voice 0 /drums, every other voice /melody */
        sl->bank = -1;
        sl->buf = -1;
        sl->enabled = 1;
        sl->tempo_lock = 1;
        sl->algo = 1;                   /* the time stretcher (phase vocoder) */
        sl->note_mode = s == 0 ? 1 : 0; /* drum notes start on transients */
        sl->pv_transient = 1;
        sl->out = s == 0 ? 0 : 1;       /* drums left, every other voice right */
        sl->bg_advance = s != 0;        /* melody voices: a note latches pos + the wall clock */
        sl->max_grains = 32;
        /* initial BufGrainPlay values: a smooth time-stretch; any other texture is just
           other values of the same parameters */
        sl->rate_q16 = 65536;
        sl->pitch_q16 = 65536;
        sl->graindur_q16 = 2621;        /* 40 ms */
        sl->overlap_q16 = 4 * 65536;
        sl->prob_q16 = 65536;
        sl->env_bars_q16 = 65536;       /* 1 bar */
        sl->interval_q16 = 6554;        /* 0.1 s (SC default) */
        sl->amp_q16 = 65536;
        sl->algo_q16 = 65536;
        g_last_base[s] = NOT_RESIDENT;
    }
}

void slot_apply_flags(int s) {
    const struct slot* sl = &g_slots[s];
    const uint32_t ctrl = (sl->enabled && cur_sample(s) ? SMP_CTRL_ENABLE : 0)
        | (sl->pos_pin ? SMP_CTRL_POS_PIN : 0) | (sl->env_hold ? SMP_CTRL_ENV_HOLD : 0)
        | (sl->bg_advance ? SMP_CTRL_POS_BG : 0);
    eng_slot_write(s, SMP_R_CTRL, ctrl);
}

static void apply_rate_pitch(int s) {
    const struct slot* sl = &g_slots[s];
    const struct sample* smp = cur_sample(s);
    if (!smp)
        return;
    const int64_t srf = sr_factor_q24(smp);
    const int64_t t24 = slot_tempo_ratio_q24(s);
    const int64_t r24 = ((int64_t)sl->rate_q16 * t24) >> 16;
    eng_slot_write(s, SMP_R_PHASOR_INC, (uint32_t)clampi((r24 * srf) >> 24, -0x7F000000LL, 0x7F000000LL));
    /* the background (bgAdvance) is the wall clock: tempo only, RATE applies from the latch */
    eng_slot_write(s, SMP_R_BG_INC, (uint32_t)clampi((t24 * srf) >> 24, 0, 0x7F000000LL));
    eng_slot_write(s, SMP_R_PITCH,
                   (uint32_t)clampi(((int64_t)sl->pitch_q16 * srf) >> 16, -0x7F000000LL, 0x7F000000LL));
}

static void apply_env(int s) {
    const struct slot* sl = &g_slots[s];
    uint64_t n;
    if (sl->env_abs)
        n = seconds_to_samples(sl->env_sec_q16);
    else
        n = ((uint64_t)(uint32_t)clampi(sl->env_bars_q16, 0, 64 << 16) * bar_samples_q16()) >> 32;
    eng_slot_write(s, SMP_R_ENV_DUR, (uint32_t)clampi((int64_t)n, 48, 0xFFFFFF));
}

void slot_apply_grain(int s) {
    const struct slot* sl = &g_slots[s];
    const struct sample* smp = cur_sample(s);
    /* winInc = 2^32 / grain samples (Q16 samples for precision), clip [1, 2^32-1] */
    const uint64_t gs_q16 = ((uint64_t)clampi(sl->graindur_q16, 1, 0x7FFFFFFF) * eng_fs_q16()) >> 16;
    uint64_t wi = gs_q16 ? (1ull << 48) / gs_q16 : 0xFFFFFFFFull;
    if (wi < 1)
        wi = 1;
    if (wi > 0xFFFFFFFFull)
        wi = 0xFFFFFFFFull;
    const int32_t ov = clampi(sl->overlap_q16, 0, 16 * 65536);
    eng_slot_write(s, SMP_R_WIN_INC, (uint32_t)wi);
    eng_slot_write(s, SMP_R_OVERLAP, (uint32_t)ov << 8);
    eng_slot_write(s, SMP_R_AMP_TARGET, (uint32_t)((1ull << 32) / (uint32_t)(ov > 65536 ? ov : 65536)));
    eng_slot_write(s, SMP_R_PROB, (uint32_t)clampi(sl->prob_q16, 0, 65536));
    eng_slot_write(s, SMP_R_SEPARATION, (uint32_t)clampi(sl->separation_q16, 0, 65535));
    eng_slot_write(s, SMP_R_RANDPITCH, (uint32_t)clampi(sl->randompitch_q16 >> 4, -16384, 16384) & 0xFFFF);
    eng_slot_write(s, SMP_R_MAX_GRAINS, (uint32_t)clampi(sl->max_grains, 1, 32));
    if (smp)
        eng_slot_write(s, SMP_R_RANDPOS,
                       (uint32_t)(((uint64_t)clampi(sl->randpos_q16, 0, 65536) * smp->frames) >> 16));
}

void slot_apply_output(int s) {
    const struct slot* sl = &g_slots[s];
    eng_slot_write(s, SMP_R_AMP, (uint32_t)clampi(sl->amp_q16 >> 4, 0, 65535));
    eng_slot_write(s, SMP_R_ALGO_AMP, (uint32_t)clampi(sl->algo_q16 >> 1, 0, 32768));
    eng_slot_write(s, SMP_R_OUT, sl->out == 0 ? 1u : sl->out == 1 ? 2u : 3u);
}

void slot_apply_tempo(int s) {
    apply_rate_pitch(s);
    apply_env(s);
}

static uint32_t pos_reg(const struct slot* sl, const struct sample* smp) {
    return (uint32_t)(((uint64_t)(sl->pos_q30 & 0x3FFFFFFF) * smp->frames) >> 22); /* u24.8 */
}

/* Writes the new buffer; moves the playhead to the same sixteenth of the bar. */
static void apply_buffer(int s, const struct sample* old) {
    const struct slot* sl = &g_slots[s];
    const struct sample* smp = cur_sample(s);
    if (smp) {
        const uint32_t base = samples_base(slot_lib(s), sl->bank, sl->buf);
        if (base != g_last_base[s]) {
            uint32_t seek = NOT_RESIDENT;
            if (old && !sl->pos_pin) {
                /* phasor (u24.8 samples) -> sixteenth position in the old loop, modulo the
                   new loop's length in sixteenths -> samples in the new loop */
                const uint64_t ph8 = eng_status((unsigned)(s * 8 + SMP_S_PHASOR));
                const uint64_t frac32 = (ph8 << 24) / old->frames;      /* Q32 of loop   */
                const uint64_t n16o = sixteenths_of(old), n16n = sixteenths_of(smp);
                const uint64_t p16 = (frac32 * n16o) % (n16n << 32);     /* Q32 16ths     */
                seek = (uint32_t)(((p16 / n16n) * smp->frames) >> 24);   /* u24.8 samples */
            }
            eng_slot_write(s, SMP_R_BASE, base);
            eng_slot_write(s, SMP_R_FRAMES, smp->frames);
            if (sl->pos_pin)
                eng_slot_write(s, SMP_R_POS, pos_reg(sl, smp));
            else if (seek != NOT_RESIDENT) {
                eng_slot_write(s, SMP_R_POS, seek);
                eng_slot_write(s, SMP_R_STROBE, SMP_STROBE_SEEK);
            }
            g_last_base[s] = base;
        }
    }
    slot_apply_flags(s);
    slot_apply_tempo(s);
    slot_apply_grain(s);
}

void slot_apply_algo(int s) {
    const struct slot* sl = &g_slots[s];
    eng_slot_write(s, SMP_R_ALGO, (uint32_t)(sl->algo & SMP_ALGO_MASK) | (sl->pv_transient ? 0 : SMP_ALGO_NO_TRANSIENT));
}

void slot_apply(int s) {
    apply_buffer(s, 0);
    slot_apply_output(s);
    slot_apply_algo(s);
}

/* Switches to the wanted buffer if it is in SDRAM. Returns 1 when playing it. */
static int try_switch(int s) {
    struct slot* sl = &g_slots[s];
    if (sl->want_bank == sl->bank && sl->want_buf == sl->buf)
        return 1;
    if (samples_base(slot_lib(s), sl->want_bank, sl->want_buf) == NOT_RESIDENT)
        return 0;
    const struct sample* old = cur_sample(s);
    if (old && sl->bank != sl->want_bank) {
        /* grains started before the switch still read the old bank for <= grainDur */
        uint32_t ms = (uint32_t)(((uint64_t)(uint32_t)sl->graindur_q16 * 1000) >> 16) + 50;
        samples_hold(slot_lib(s), sl->bank, ms > 1100 ? 1100 : ms);
    }
    sl->bank = sl->want_bank;
    sl->buf = sl->want_buf;
    apply_buffer(s, old);
    return 1;
}

void slot_select(int s, int idx) {
    struct slot* sl = &g_slots[s];
    sl->cc1 = idx;
    if (sl->want_bank < 0)
        sl->want_bank = 0;
    sl->want_buf = wrap(idx, bank_count(s, sl->want_bank));
    if (sl->want_buf < 0)
        return;
    samples_want(slot_lib(s), sl->want_bank, sl->want_buf, 1);
    try_switch(s);
}

void slot_select_bank(int s, int bank) {
    struct slot* sl = &g_slots[s];
    sl->want_bank = wrap(bank, g_libs[slot_lib(s)].nbanks);
    if (sl->want_bank < 0)
        return;
    sl->want_buf = wrap(sl->cc1, bank_count(s, sl->want_bank));
    samples_want(slot_lib(s), sl->want_bank, sl->want_buf, 1);
    try_switch(s);
}

void slots_service(void) {
    int busy = 0;
    for (int s = 0; s < NUM_SLOTS; ++s) {
        struct slot* sl = &g_slots[s];
        if (sl->want_buf < 0 || try_switch(s))
            continue;
        samples_want(slot_lib(s), sl->want_bank, sl->want_buf, 1);
        busy = 1;
    }
    if (busy)
        return;
    /* idle: finish the playing banks, then prefetch the next bank of each slot */
    for (int s = 0; s < NUM_SLOTS; ++s)
        if (g_slots[s].bank >= 0)
            samples_want(slot_lib(s), g_slots[s].bank, -1, 0);
    for (int s = 0; s < NUM_SLOTS; ++s)
        if (g_slots[s].bank >= 0 && g_libs[slot_lib(s)].nbanks > 1)
            samples_want(slot_lib(s), wrap(g_slots[s].bank + 1, g_libs[slot_lib(s)].nbanks), -1, 0);
}

void slot_set_pos(int s, int32_t pos_q30) {
    struct slot* sl = &g_slots[s];
    const struct sample* smp = cur_sample(s);
    if (pos_q30 < 0) { /* BufGrainPlay pos = -1: continue from wherever the playhead is */
        sl->pos_pin = 0;
    } else {
        sl->pos_q30 = (uint32_t)pos_q30 & 0x3FFFFFFF;
        sl->pos_pin = 1;
        if (smp)
            eng_slot_write(s, SMP_R_POS, pos_reg(sl, smp));
    }
    slot_apply_flags(s);
}

void slot_trig(int s) {
    const struct slot* sl = &g_slots[s];
    const struct sample* smp = cur_sample(s);
    if (!smp)
        return;
    eng_slot_write(s, SMP_R_POS, pos_reg(sl, smp));
    eng_slot_write(s, SMP_R_REPEATS, (uint32_t)clampi(sl->repeats, 0, 126));
    eng_slot_write(s, SMP_R_INTERVAL, (uint32_t)clampi(seconds_to_samples(sl->interval_q16), 1, 12000));
    eng_slot_write(s, SMP_R_STROBE, SMP_STROBE_TRIG); /* strobe last: params land first */
}

static int32_t fmod_i(int32_t a, int32_t n) {
    const int32_t m = a % n;
    return m < 0 ? m + n : m; /* SC % is a floored modulo */
}

void slot_note_on(int s, int note, int vel) {
    struct slot* sl = &g_slots[s];
    const int32_t n16 = (int32_t)slot_sixteenths(s);
    const int32_t m = fmod_i(note - 12, n16);
    sl->pos_q30 = (uint32_t)(((uint64_t)m << 30) / (uint32_t)n16);
    const struct sample* smp = cur_sample(s);
    const uint32_t* tp;
    const int nt = (sl->note_mode && smp) ? samples_transients(smp, &tp) : -1;
    if (nt > 0 && smp->frames) {
        uint32_t p = 0;
        if (sl->note_mode == 1) {
            p = tp[fmod_i(note - 12, nt)];                     /* k-th transient */
        } else {
            /* the 16th, moved to the nearest transient within half a 16th */
            const uint32_t g = (uint32_t)(((uint64_t)m * smp->frames) / (uint32_t)n16);
            const uint32_t half = smp->frames / (uint32_t)(2 * n16);
            uint32_t best = half + 1;
            p = g;
            for (int k = 0; k < nt; ++k) {
                const uint32_t d = tp[k] > g ? tp[k] - g : g - tp[k];
                if (d < best) {
                    best = d;
                    p = tp[k];
                }
            }
        }
        sl->pos_q30 = (uint32_t)(((uint64_t)p << 30) / smp->frames);
    }
    sl->repeats = vel - 1;
    if (sl->pos_pin) { /* as in SC: Latch(pos, trig), the playhead runs from there */
        sl->pos_pin = 0;
        slot_apply_flags(s);
    }
    slot_apply_tempo(s); /* the tempo, on every note */
    slot_trig(s);
}

/* Note-off ends the gate (1 ms ramp): the gate length / envHold is the longest a note
   plays, the note's own length can cut it shorter. */
void slot_note_off(int s) { eng_slot_write(s, SMP_R_STROBE, SMP_STROBE_GATE_OFF); }

void set_global_bpm(uint32_t bpm_milli) {
    if (bpm_milli < 20000)
        bpm_milli = 20000;
    if (bpm_milli > 400000)
        bpm_milli = 400000;
    g_bpm_milli = bpm_milli;
    /* beat phasor: bpm/60/fs beats per sample, u0.32 */
    const uint64_t inc = (((uint64_t)bpm_milli << 32) * ENGINE_CYCLES_PER_SAMPLE) / ((uint64_t)60000 * AUDIO_CLK_FREQ);
    eng_write(SMP_ADDR(SMP_GLOBAL_SLOT, SMP_G_BEAT_INC), (uint32_t)inc);
    for (int s = 0; s < NUM_SLOTS; ++s)
        slot_apply_tempo(s);
}
