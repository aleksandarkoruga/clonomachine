/* The wavetable synth on the board, see synth.h. */
#include "synth.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <generated/csr.h>
#include <generated/mem.h>
#include <system.h>

#include "../include/sampler_events.h"
#include "engine.h"
#include "esplink.h"
#include "midimap.h"
#include "samples.h"
#include "scales.h"
#include "slot.h"
#include "synth_ctl.h"
#include "synth_wt.h"

#ifdef CSR_SAMPLER_SYNTH_CMD_DATA_ADDR

#include "fatfs/ff.h"

static struct sy_state g_sy;
static int g_on;
static uint32_t g_tick_at;
#define WT_BASE ((uint32_t)((MAIN_RAM_SIZE - (SYNTH_TABLE_MB << 20)) / 2)) /* words */
#define WT_END ((uint32_t)(MAIN_RAM_SIZE / 2))
#define WT_DIR "/wavetables"

static void hw_write(uint32_t a, uint32_t d) {
    while (!sampler_synth_cmd_ready_read())
        ;
    sampler_synth_cmd_data_write(d);
    sampler_synth_cmd_addr_write(a);
}
static int16_t* wptr(uint32_t word) { return (int16_t*)(MAIN_RAM_BASE + 2 * word); }

int synth_present(void) { return 1; }

/* ---- tables -------------------------------------------------------------------------------
 * The card's /wavetables/NN_name.wtl (prepared on the PC, host/prepare_wt.sh) are indexed at boot;
 * the 8 MB hold four of them (2 MB regions: the largest table fits). The table an oscillator
 * selects loads in the background (slices from the main loop) while the old one plays on, then
 * the next one in the direction the knob moved is fetched ahead; the least recently used region
 * that no oscillator plays is reused. Read only: the board never writes tables to the card.
 * Without tables on the card a few shapes are built here instead (all resident). */
#define WT_REGIONS 4
#define WT_REGION_WORDS (1u << 20)
#define WT_CAT 128
#define WT_SLICE (4u << 10)  /* words per main-loop slice (8 KB, ~2.5 ms of SD) */

struct wtl_head {
    uint32_t magic, version, frames, words;
    char name[32];
};
#define WTL_MAGIC 0x54575953u /* "SYWT" */

struct wt_entry {
    char file[96];
    char name[28];
    uint16_t frames;           /* known once loaded (the header is read then); 0xFFFF: not a table */
    int num;                   /* the number before the name: the order */
};
static struct wt_entry g_cat[WT_CAT];
static int g_ncat;
static struct {
    int entry;                 /* catalog index held, -1 empty / being filled */
    uint32_t lru;
    struct sy_table T;
} g_reg[WT_REGIONS];
static uint32_t g_lru;
static int g_want[2] = {-1, -1}, g_dir[2] = {1, 1}, g_last_osc;
static struct {
    int active, region, entry;
    uint32_t done, words;
    FIL f;
} g_ld;
static int g_builtin; /* no tables on the card: built-ins, all resident */
static struct sy_table g_bi[SY_MAX_TABLES];
static int g_nbi;
static uint32_t g_loads, g_load_ms;

static int16_t g_x[WT_N0];

static void build_frame(const struct sy_table* T, int f) {
    int16_t* dst[SY_LEVELS] = {0};
    for (int k = 1; k < SY_LEVELS; ++k)
        dst[k] = wptr(T->lvl[k] + (uint32_t)f * (uint32_t)wt_level_size(k));
    wt_build_frame(g_x, dst);
}

static void load_builtins(void) {
    uint32_t a = WT_BASE;
    g_nbi = 0;
    for (int t = 0; t < wt_builtin_count() && g_nbi < SY_MAX_TABLES; ++t) {
        struct sy_table* T = &g_bi[g_nbi];
        const int fr = wt_builtin_frames(t);
        if (wt_words(fr) > WT_END - a)
            break;
        memset(T, 0, sizeof(*T));
        strncpy(T->name, wt_builtin_name(t), sizeof(T->name) - 1);
        T->frames = (uint16_t)fr;
        wt_layout(a, fr, T->lvl);
        for (int f = 0; f < fr; ++f) {
            wt_builtin_frame(t, f, g_x);
            build_frame(T, f);
        }
        a += wt_words(fr);
        ++g_nbi;
    }
}

/* the catalog: the table files in the order of their numbers (= the table index), names from the
   file names; a header is read when its table loads (boot stays quick with many files) */
static void scan_tables(void) {
    g_ncat = 0;
    if (!samples_card_ok())
        return;
    DIR dj;
    FILINFO fi;
    if (f_opendir(&dj, WT_DIR) != FR_OK)
        return;
    while (g_ncat < WT_CAT && f_readdir(&dj, &fi) == FR_OK && fi.fname[0]) {
        const size_t L = strlen(fi.fname);
        if ((fi.fattrib & AM_DIR) || fi.fname[0] == '.' || L < 5 || L >= sizeof(g_cat[0].file))
            continue;
        const char* e = fi.fname + L - 4;
        if (e[0] != '.' || (e[1] | 32) != 'w' || (e[2] | 32) != 't' || (e[3] | 32) != 'l')
            continue;
        struct wt_entry* c = &g_cat[g_ncat++];
        strcpy(c->file, fi.fname);
        c->num = atoi(fi.fname);
        const char* nm = fi.fname;
        while (*nm >= '0' && *nm <= '9')
            ++nm;
        if (*nm == '_')
            ++nm;
        size_t k = 0;
        for (; nm[k] && nm + k < e && k + 1 < sizeof(c->name); ++k)
            c->name[k] = nm[k];
        c->name[k] = 0;
        c->frames = 0;
    }
    f_closedir(&dj);
    for (int i = 1; i < g_ncat; ++i) /* by number, then name (insertion sort) */
        for (int j = i; j > 0 && (g_cat[j - 1].num > g_cat[j].num ||
                                  (g_cat[j - 1].num == g_cat[j].num && strcmp(g_cat[j - 1].file, g_cat[j].file) > 0));
             --j) {
            struct wt_entry t = g_cat[j];
            g_cat[j] = g_cat[j - 1];
            g_cat[j - 1] = t;
        }
}

static int resident(int entry) {
    for (int r = 0; r < WT_REGIONS; ++r)
        if (g_reg[r].entry == entry)
            return r;
    return -1;
}
static int in_use(int r) { /* an oscillator plays it */
    return g_reg[r].entry >= 0 && (g_sy.osc_tab[0] == &g_reg[r].T || g_sy.osc_tab[1] == &g_reg[r].T);
}
static void use_resident(int o) {
    const int r = g_want[o] >= 0 ? resident(g_want[o]) : -1;
    if (r >= 0) {
        g_reg[r].lru = ++g_lru;
        sy_use_table(&g_sy, o, &g_reg[r].T);
    }
}

/* an oscillator's table knob moved: play it now if it's in, else it loads (synth_service) */
static void table_request(int o) {
    if (g_builtin || !g_ncat)
        return;
    int idx = sy_natural_q16(&g_sy, o ? SP_B_TABLE : SP_A_TABLE) >> 16;
    if (idx >= g_ncat)
        idx = g_ncat - 1;
    if (idx == g_want[o])
        return;
    g_dir[o] = idx < g_want[o] ? -1 : 1;
    g_want[o] = idx;
    g_last_osc = o;
    use_resident(o);
}

static void load_start(int entry, int region) {
    char p[128];
    snprintf(p, sizeof(p), WT_DIR "/%s", g_cat[entry].file);
    struct wtl_head h;
    UINT got = 0;
    if (f_open(&g_ld.f, p, FA_READ) != FR_OK)
        return;
    if (f_read(&g_ld.f, &h, sizeof(h), &got) != FR_OK || got != sizeof(h) || h.magic != WTL_MAGIC || h.version != 1 ||
        h.frames < 1 || h.frames > 256 || h.words != wt_words((int)h.frames)) {
        f_close(&g_ld.f);
        printf("synth: %s: not a table file\n", p);
        g_cat[entry].frames = 0xFFFF;
        return;
    }
    g_cat[entry].frames = (uint16_t)h.frames;
    g_reg[region].entry = -1; /* being overwritten */
    struct sy_table* T = &g_reg[region].T;
    memset(T, 0, sizeof(*T));
    h.name[sizeof(h.name) - 1] = 0;
    strncpy(T->name, h.name, sizeof(T->name) - 1);
    T->frames = (uint16_t)h.frames;
    wt_layout(WT_BASE + (uint32_t)region * WT_REGION_WORDS, T->frames, T->lvl);
    g_ld.active = 1;
    g_ld.region = region;
    g_ld.entry = entry;
    g_ld.done = 0;
    g_ld.words = wt_words(T->frames);
    g_load_ms = eng_status(SMP_SG_SAMPLES);
}

/* a free region, else the least recently used one no oscillator plays or wants */
static int pick_region(void) {
    int best = -1;
    for (int r = 0; r < WT_REGIONS; ++r) {
        if (g_reg[r].entry < 0)
            return r;
        if (in_use(r) || g_reg[r].entry == g_want[0] || g_reg[r].entry == g_want[1])
            continue;
        if (best < 0 || g_reg[r].lru < g_reg[best].lru)
            best = r;
    }
    return best;
}

static void tables_service(void) {
    static int busy;
    if (g_builtin || !g_ncat || busy)
        return;
    busy = 1;
    if (!g_ld.active) {
        int entry = -1;
        for (int k = 0; k < 2 && entry < 0; ++k) { /* the wanted ones: the knob last moved first */
            const int o = k ? !g_last_osc : g_last_osc;
            if (g_want[o] >= 0 && resident(g_want[o]) < 0 && g_cat[g_want[o]].frames != 0xFFFF)
                entry = g_want[o];
        }
        if (entry < 0) { /* both in: the next one in the direction the knob last moved */
            const int n = g_want[g_last_osc] + g_dir[g_last_osc];
            if (g_want[g_last_osc] >= 0 && n >= 0 && n < g_ncat && resident(n) < 0 && g_cat[n].frames != 0xFFFF)
                entry = n;
        }
        const int r = entry >= 0 ? pick_region() : -1;
        if (r >= 0)
            load_start(entry, r);
    } else {
        struct sy_table* T = &g_reg[g_ld.region].T;
        uint32_t n = g_ld.words - g_ld.done;
        if (n > WT_SLICE)
            n = WT_SLICE;
        UINT got = 0;
        const int ok = f_read(&g_ld.f, wptr(T->lvl[1] + g_ld.done), n * 2, &got) == FR_OK && got == n * 2;
        g_ld.done += n;
        if (!ok || g_ld.done >= g_ld.words) {
            f_close(&g_ld.f);
            g_ld.active = 0;
            if (ok) {
                flush_cpu_dcache();
                flush_l2_cache(); /* the oscillators read SDRAM behind the CPU's caches */
                g_reg[g_ld.region].entry = g_ld.entry;
                g_reg[g_ld.region].lru = ++g_lru;
                ++g_loads;
                g_load_ms = (eng_status(SMP_SG_SAMPLES) - g_load_ms) / 48;
                for (int o = 0; o < 2; ++o)
                    if (g_want[o] == g_ld.entry)
                        use_resident(o);
            }
        }
    }
    busy = 0;
}

/* ---- control ------------------------------------------------------------------------------ */
static void apply_send(void) {
    const int32_t x = sy_natural_q16(&g_sy, SP_FXSEND); /* 0..1 -> u4.12 */
    eng_write(SMP_ADDR(SMP_GLOBAL_SLOT, SMP_G_SYN_SEND), (uint32_t)(x > 0 ? x >> 4 : 0));
}
static int harm_param(int p) { return (p >= SP_HARMONY && p <= SP_HARMTRIG) || p == SP_SCALE; }
static int harm_int(int p) { return (sy_natural_q16(&g_sy, p) + 32768) >> 16; }
static int harmony_mode(void) { return harm_int(SP_HARMONY); } /* 0 off, 1 play, 2 offset */

/* offset mode: the ESP sends the next chord's offset ahead, applied here on its step (the 32nd
   tick or the trig) so the notes of that step already get it */
static struct {
    int pending, ofs;
    uint32_t at;     /* the 32nd step (harmTrig 0) */
    uint32_t trigs;  /* note-ons on the harmTrig channel since the start (every harmBeats-th steps) */
    int mode;        /* the last mode seen */
} g_hm;

/* the trig count from the start; the waiting offset goes when the ESP starts over (mode,
   harmTrig: it sends a new one) or its 32nd means nothing any more (a start on the beat); on
   trigs the ESP waits with the first chord through stop and start */
static void harm_reset(int drop) {
    g_hm.trigs = 0;
    if (drop)
        g_hm.pending = 0;
}

void synth_esp_state(void) {
    if (!g_on)
        return;
    esplink_syn_param(SP_SCALE, sy_natural_q16(&g_sy, SP_SCALE));
    for (int p = SP_HARMONY; p <= SP_HARMTRIG; ++p)
        esplink_syn_param(p, sy_natural_q16(&g_sy, p));
}

static void after_param(int p) {
    if (harm_param(p)) { /* the ESP's harmony generator plays from these */
        esplink_syn_param(p, sy_natural_q16(&g_sy, p));
        if (p == SP_HARMONY) {
            const int m = harmony_mode();
            if (m != g_hm.mode && (m == 0 || g_hm.mode == 1))
                sy_all_off(&g_sy); /* off, or the generator's notes when it stops playing */
            if (m != 2)
                sy_set_harm_offset(&g_sy, 0);
            if (m != g_hm.mode)
                harm_reset(1);
            g_hm.mode = m;
        } else if (p == SP_HARMTRIG || p == SP_HARMBEATS)
            harm_reset(p == SP_HARMTRIG);
    }
    if (p == SP_FXSEND)
        apply_send();
    else if (p == SP_A_TABLE || p == SP_B_TABLE)
        table_request(p == SP_B_TABLE);
}
static void set_knob(int p, int32_t k) {
    sy_set_knob(&g_sy, p, k);
    after_param(p);
}

/* a table's index by name (the card's catalog, or the built-ins), else fallback */
static int table_named(const char* name, int fallback) {
    if (g_builtin) {
        for (int i = 0; i < g_nbi; ++i)
            if (!strcmp(g_bi[i].name, name))
                return i;
    } else {
        for (int i = 0; i < g_ncat; ++i)
            if (!strcmp(g_cat[i].name, name))
                return i;
    }
    return fallback;
}

/* the power-up patch: the reese (fw/synth_ctl.c), A on the saw, B on the reese table if the card
   has it */
static void patch_reese(void) {
    sy_patch_reese(&g_sy);
    const int saw = table_named("saw", 0);
    sy_set_natural(&g_sy, SP_A_TABLE, saw << 16);
    sy_set_natural(&g_sy, SP_B_TABLE, table_named("reese_techstep", saw) << 16);
    after_param(SP_A_TABLE);
    after_param(SP_B_TABLE);
    after_param(SP_FXSEND);
}

void synth_init(void) {
    const uint32_t t0 = eng_status(SMP_SG_SAMPLES);
    scales_init();
    wt_init();
#if defined(SRAM_BASE) && defined(SRAM_SIZE)
    if (SRAM_SIZE >= 8192) /* on-chip, unused after the BIOS: the transforms' work array */
        wt_set_work((int32_t*)SRAM_BASE);
#endif
    for (int r = 0; r < WT_REGIONS; ++r)
        g_reg[r].entry = -1;
    scan_tables();
    g_builtin = g_ncat == 0;
    if (g_builtin)
        load_builtins();
    flush_cpu_dcache();
    flush_l2_cache();
    sy_init(&g_sy, eng_fs_q16(), hw_write);
    if (g_builtin)
        sy_set_tables(&g_sy, g_bi, g_nbi);
    sy_set_bpm(&g_sy, g_bpm_milli);
    sy_hw_reset(&g_sy);
    apply_send();
    g_tick_at = eng_status(SMP_SG_SAMPLES);
    g_on = 1;
    patch_reese(); /* the power-up sound; its tables are requested, the first loads now */
    while (g_ncat && resident(g_want[0]) < 0) {
        const uint32_t before = g_ld.done;
        tables_service();
        if (!g_ld.active && resident(g_want[0]) < 0 && g_ld.done == before)
            break;
    }
    printf("synth: %d tables %s, %lu ms\n", g_builtin ? g_nbi : g_ncat,
           g_builtin ? "built in (no " WT_DIR "/NN_name.wtl on the card)" : "on the card (" WT_DIR "), loaded as selected",
           (unsigned long)((eng_status(SMP_SG_SAMPLES) - t0) / 48));
}

void synth_service(void) {
    if (!g_on)
        return;
    const uint32_t now = eng_status(SMP_SG_SAMPLES);
    if (now - g_tick_at < SY_TICK)
        return;
    g_tick_at += SY_TICK;
    if (now - g_tick_at > 4 * SY_TICK) /* fell behind (a long SD read): skip ahead */
        g_tick_at = now;
    sy_tick(&g_sy);
    tables_service();
}

void synth_bpm(uint32_t bpm_milli) {
    if (g_on)
        sy_set_bpm(&g_sy, bpm_milli);
}

static void harm_note_trig(void);

void synth_midi(const struct mm_synth* pg, uint8_t st, uint8_t d1, uint8_t d2) {
    if (!g_on)
        return;
    switch (st & 0xF0) {
    case 0x90:
        if (d2)
            harm_note_trig();
        if (harmony_mode() == 1) /* the harmony generator plays the voices */
            break;
        if (d2) {
            sy_note_on(&g_sy, d1, d2, 0);
            break;
        }
        /* fall through: velocity 0 = note off */
    case 0x80:
        if (harmony_mode() != 1)
            sy_note_off(&g_sy, d1, 0);
        break;
    case 0xB0:
        if (d1 == 120 || d1 == 123) {
            sy_all_off(&g_sy);
            break;
        }
        if (d1 == 1 && pg->mw < SP_NUM) {
            set_knob(pg->mw, (d2 & 127) * 65536 / 127);
            break;
        }
        if (d1 >= pg->cc && d1 - pg->cc < pg->nknobs && pg->knobs[d1 - pg->cc] < SP_NUM)
            set_knob(pg->knobs[d1 - pg->cc], (d2 & 127) * 65536 / 127);
        break;
    case 0xD0:
        if (pg->at < SP_NUM)
            set_knob(pg->at, (d1 & 127) * 65536 / 127);
        break;
    case 0xE0:
        if (pg->pb == MM_SYN_BEND)
            sy_bend(&g_sy, d1 | d2 << 7);
        else if (pg->pb < SP_NUM)
            set_knob(pg->pb, (d2 & 127) * 65536 / 127);
        break;
    default: break;
    }
}

void synth_event(const struct smp_event* e) {
    if (!g_on)
        return;
    if (e->type == EV_SYN_NOTE) {
        if (e->b)
            sy_note_on(&g_sy, e->a, e->b, 1);
        else
            sy_note_off(&g_sy, e->a, 1);
    } else if (e->type == EV_SYN_PARAM && e->a < SP_NUM) {
        sy_set_natural(&g_sy, e->a, e->value);
        after_param(e->a);
    } else if (e->type == EV_SYN_HOFS && harmony_mode() == 2) {
        if (!e->b)
            sy_set_harm_offset(&g_sy, (int)e->a - 64);
        else {
            g_hm.ofs = (int)e->a - 64;
            g_hm.at = (uint32_t)e->value;
            g_hm.pending = 1;
        }
    }
}

/* a note-on on the synth's channel with harmTrig 1: every harmBeats-th steps the chords, before
   the note plays (it gets the new offset) */
static void harm_note_trig(void) {
    if (!harmony_mode() || !harm_int(SP_HARMTRIG))
        return;
    const uint32_t n = g_hm.trigs++, beats = (uint32_t)harm_int(SP_HARMBEATS);
    if (n % (beats ? beats : 1))
        return;
    if (g_hm.pending) {
        g_hm.pending = 0;
        sy_set_harm_offset(&g_sy, g_hm.ofs);
    }
    esplink_syn_trig();
}

void synth_harm_tick(uint32_t step) {
    if (g_on && g_hm.pending && !harm_int(SP_HARMTRIG) && (int32_t)(step - g_hm.at) >= 0) {
        g_hm.pending = 0;
        sy_set_harm_offset(&g_sy, g_hm.ofs);
    }
}

void synth_harm_transport(void) { harm_reset(!harm_int(SP_HARMTRIG)); }

/* ---- console ------------------------------------------------------------------------------- */
static int32_t parse_q16(const char* s) {
    int neg = *s == '-';
    if (*s == '-' || *s == '+')
        ++s;
    int64_t ip = 0, fr = 0, div = 1;
    while (*s >= '0' && *s <= '9')
        ip = ip * 10 + (*s++ - '0');
    if (*s == '.')
        for (++s; *s >= '0' && *s <= '9' && div < 100000000; ++s)
            fr = fr * 10 + (*s - '0'), div *= 10;
    const int64_t v = (ip << 16) + (fr << 16) / div;
    return (int32_t)(neg ? -v : v);
}
static void print_q16(int32_t v) {
    const int neg = v < 0;
    const uint32_t a = (uint32_t)(neg ? -v : v);
    printf("%s%lu.%03lu", neg ? "-" : "", (unsigned long)(a >> 16), (unsigned long)(((a & 0xFFFF) * 1000 + 32768) >> 16));
}

static void status(void) {
    const uint32_t cy = sampler_synth_cycles_read();
    printf("synth: %d of %d voices active, %d tables, most cycles per sample: oscillators %lu, filter %lu "
           "(of 1024); lag %lu; register writes %lu\n",
           sy_active_voices(&g_sy), SY_VOICES, g_builtin ? g_nbi : g_ncat, (unsigned long)(cy & 0x7ff), (unsigned long)(cy >> 11),
           (unsigned long)sampler_synth_lag_read(), (unsigned long)g_sy.writes);
    printf("  scale %s, tuning %s, root %d, scaleMap %d, noteOffset %d\n",
           scale_name(sy_natural_q16(&g_sy, SP_SCALE) >> 16), tuning_name(sy_natural_q16(&g_sy, SP_TUNING) >> 16),
           (int)(sy_natural_q16(&g_sy, SP_ROOT) >> 16), (int)(sy_natural_q16(&g_sy, SP_SCALEMAP) >> 16),
           (int)(sy_natural_q16(&g_sy, SP_NOTEOFFSET) >> 16));
    static const char* const modes[] = {"off", "play", "offset"};
    const int m = harmony_mode();
    printf("  harmony %s, steps on ", modes[m < 0 ? 0 : m > 2 ? 2 : m]);
    if (harm_int(SP_HARMTRIG))
        printf("its note-ons (%lu so far, every %d)", (unsigned long)g_hm.trigs, harm_int(SP_HARMBEATS));
    else
        printf("the beat (every %d)", harm_int(SP_HARMBEATS));
    printf(", offset %d degrees%s\n", g_sy.harm_ofs, g_hm.pending ? ", the next one waiting" : "");
}

void synth_console(int argc, char** argv) {
    if (argc < 2) {
        status();
        puts("synth tables | params | scales | note <n> [vel] | deg <d> [vel] | off | reese (the power-up patch) | "
             "<param> <value> | reg <hex> <hex>");
        return;
    }
    const char* c = argv[1];
    if (!strcmp(c, "tables")) {
        if (g_builtin)
            for (int i = 0; i < g_nbi; ++i)
                printf("  %3d %-28s %3u frames (built in)\n", i, g_bi[i].name, g_bi[i].frames);
        for (int i = 0; i < g_ncat; ++i) {
            const int r = resident(i);
            char fr[16];
            if (g_cat[i].frames == 0xFFFF)
                snprintf(fr, sizeof(fr), "not a table");
            else if (g_cat[i].frames)
                snprintf(fr, sizeof(fr), "%3u frames", g_cat[i].frames);
            else
                fr[0] = 0;
            printf("  %3d %-28s %-11s%s%s%s\n", i, g_cat[i].name, fr, r >= 0 ? "  in memory" : "",
                   (g_want[0] == i) ? "  A" : "", (g_want[1] == i) ? "  B" : "");
        }
        printf("  %lu loads, the last %lu ms\n", (unsigned long)g_loads, (unsigned long)g_load_ms);
    } else if (!strcmp(c, "params")) {
        for (int p = 0; p < SP_NUM; ++p) {
            printf("  %-12s ", sy_param_name(p));
            print_q16(sy_natural_q16(&g_sy, p));
            putchar((p % 4) == 3 ? '\n' : ' ');
        }
        putchar('\n');
    } else if (!strcmp(c, "scales")) {
        for (int i = 0; i < scales_count(); ++i)
            printf("  scale %2d %s (%d)\n", i, scale_name(i), scale_size(i));
        for (int i = 0; i < tunings_count(); ++i)
            printf("  tuning %2d %s\n", i, tuning_name(i));
        for (int i = 0; i < SYS_NUM; ++i)
            printf("  source %2d %s\n", i, sy_source_name(i));
    } else if (!strcmp(c, "note") && argc > 2) {
        sy_note_on(&g_sy, atoi(argv[2]), argc > 3 ? atoi(argv[3]) : 100, 0);
    } else if (!strcmp(c, "deg") && argc > 2) {
        sy_note_on(&g_sy, 64 + atoi(argv[2]), argc > 3 ? atoi(argv[3]) : 100, 1);
    } else if (!strcmp(c, "reese")) { /* the power-up patch again */
        sy_all_off(&g_sy);
        patch_reese();
        puts("  the reese patch");
    } else if (!strcmp(c, "off")) {
        sy_all_off(&g_sy);
    } else if (!strcmp(c, "reg") && argc > 3) {
        hw_write((uint32_t)strtoul(argv[2], NULL, 16), (uint32_t)strtoul(argv[3], NULL, 16));
    } else if (argc > 2) {
        const int p = sy_param_of(c);
        if (p < 0) {
            printf("synth: unknown parameter %s (synth params)\n", c);
            return;
        }
        sy_set_natural(&g_sy, p, parse_q16(argv[2]));
        after_param(p);
        printf("  %s = ", sy_param_name(p));
        print_q16(sy_natural_q16(&g_sy, p));
        putchar('\n');
    } else {
        status();
    }
}

#else /* no synth in the gateware */

int synth_present(void) { return 0; }
void synth_init(void) {}
void synth_service(void) {}
void synth_bpm(uint32_t bpm_milli) { (void)bpm_milli; }
void synth_midi(const struct mm_synth* pg, uint8_t st, uint8_t d1, uint8_t d2) { (void)pg, (void)st, (void)d1, (void)d2; }
void synth_event(const struct smp_event* e) { (void)e; }
void synth_harm_tick(uint32_t step) { (void)step; }
void synth_harm_transport(void) {}
void synth_console(int argc, char** argv) { (void)argc, (void)argv; puts("synth: not in this gateware"); }
void synth_esp_state(void) {}

#endif
