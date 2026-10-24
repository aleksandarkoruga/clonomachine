/* Sample library: SD index + per-bank SDRAM residency, see samples.h.
 *
 * SDRAM buffer layout (16-bit words, 64-word aligned, include/sampler_regs.h):
 *   [64 pre-pad][frames][128 post-pad]; a bank is one contiguous region of buffers.
 * Accepted: RIFF/WAVE PCM 8/16/24/32-bit and IEEE float 32/64-bit, 1..8 channels (downmixed),
 * any sample rate (compensated in PHASOR_INC/PITCH; 48 kHz mono 16-bit is the native format).
 */
#include "samples.h"
#include "transients.h"
#include "wavutil.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>
#include <system.h>

#include "../include/sampler_regs.h"
#include "engine.h"

#ifdef CSR_SDCARD_BASE
#include "fatfs/diskio.h"
#include "fatfs/ff.h"
#include <liblitesdcard/sdcard.h>
#endif

struct lib g_libs[MAX_LIBS];
int g_normalize = 1;
int g_sd_verbose = 0;

static struct sample g_store[MAX_LIBS][MAX_PER_LIB];
static struct bank g_bstore[MAX_LIBS][MAX_BANKS_PER_LIB];
static const char* const g_dirs[MAX_LIBS] = {"/drums", "/melody", "/bass", "/kick"};

#define POOL_START_WORDS (0x200000u / 2)  /* firmware owns the first 2 MB (fw/linker.ld) */
static volatile int16_t* const g_mem = (volatile int16_t*)MAIN_RAM_BASE;
static uint32_t pool_end_words(void) { return MAIN_RAM_SIZE / 2; }
static uint32_t g_lru_clock;
static int g_card_ok;

static inline uint32_t now_samples(void) { return eng_status(SMP_SG_SAMPLES); }

void (*samples_idle_hook)(void);  /* loader slices: may dispatch events  */
void (*samples_drain_hook)(void); /* SD sector I/O: buffer UART input only */
static inline void idle_hook(void) {
    if (samples_idle_hook)
        samples_idle_hook();
}
static inline void drain_hook(void) {
    if (samples_drain_hook)
        samples_drain_hook();
}

uint32_t samples_track_bpm_q16(uint32_t frames, uint32_t sr, int32_t* beats_log2) {
    /* bpm(k) = 60 * sr * 2^k / frames, fold while bpm < 100 || bpm > 200 (as in SC) */
    int32_t k = 0;
    uint64_t bpm = 0;
    for (int it = 0; it < 1000; ++it) {
        const uint64_t num = (uint64_t)60 * sr << 16;
        if (k >= 0)
            bpm = (k < 24) ? (num << k) / frames : 0xFFFFFFFFull;
        else
            bpm = (num / frames) >> (-k);
        if (bpm < (100ull << 16))
            ++k;
        else if (bpm > (200ull << 16))
            --k;
        else
            break;
    }
    if (beats_log2)
        *beats_log2 = k;
    return (uint32_t)bpm;
}

__attribute__((weak)) int samples_bank_keep(int lib, int bank) {
    (void)lib;
    (void)bank;
    return 0;
}

static uint32_t footprint(uint32_t frames) { return (SMP_PREPAD + frames + SMP_POSTPAD + 63) & ~63u; }

uint32_t samples_base(int lib, int bank, int idx) {
    const struct bank* b = samples_bank(lib, bank);
    const struct sample* s = samples_get(lib, bank, idx);
    if (!b || !s || b->base == NOT_RESIDENT || !s->ready)
        return NOT_RESIDENT;
    return b->base + s->offset;
}

void samples_hold(int lib, int bank, uint32_t ms) {
    struct bank* b = samples_bank(lib, bank);
    if (b)
        b->hold_until = now_samples() + ms * 48;
}

/* ---- pool allocator: one bank per region, LRU eviction ----------------------------------
 * The pool is cut into equal regions that each hold the largest bank (BANK_MAX_BYTES):
 * 4 x 7.5 MB in the 30 MB pool. Any free region fits any bank, so free space can't
 * fragment. (First fit at any address left gaps just under a bank between two playing
 * banks: a 7.1 MB bank then found no room until one of them was replaced.) */
static struct bank* g_job_bank;

static uint32_t region_words(void) {
    const uint32_t pool = pool_end_words() - POOL_START_WORDS;
    const uint32_t n = pool / (BANK_MAX_BYTES / 2);
    return n ? (pool / n) & ~63u : pool;
}

static uint32_t find_gap(uint32_t need) {
    const uint32_t rw = region_words();
    if (need > rw)
        return NOT_RESIDENT;
    for (uint32_t at = POOL_START_WORDS; at + rw <= pool_end_words(); at += rw) {
        int used = 0;
        for (int l = 0; l < MAX_LIBS && !used; ++l)
            for (int i = 0; i < g_libs[l].nbanks && !used; ++i)
                used = g_libs[l].b[i].base == at;
        if (!used)
            return at;
    }
    return NOT_RESIDENT;
}

static void bank_evict(int lib, struct bank* b) {
    b->base = NOT_RESIDENT;
    b->state = BANK_NONE;
    for (int i = 0; i < b->count; ++i)
        g_libs[lib].s[b->first + i].ready = 0;
}

/* Evicts the least valuable bank with keep level < max_keep (LRU within a level).
   ignore_hold: last resort for a bank a slot is waiting for. */
static int evict_one(int max_keep, int ignore_hold) {
    struct bank* victim = 0;
    int vlib = 0, vkeep = 0;
    const uint32_t now = now_samples();
    for (int l = 0; l < MAX_LIBS; ++l)
        for (int i = 0; i < g_libs[l].nbanks; ++i) {
            struct bank* b = &g_libs[l].b[i];
            const int keep = samples_bank_keep(l, i);
            if (b->base == NOT_RESIDENT || b->builtin || b == g_job_bank || keep >= max_keep
                || (!ignore_hold && (int32_t)(now - b->hold_until) < 0))
                continue;
            if (!victim || keep < vkeep || (keep == vkeep && b->lru < victim->lru)) {
                victim = b;
                vlib = l;
                vkeep = keep;
            }
        }
    if (!victim)
        return 0;
    if (g_sd_verbose)
        printf("[evict %s/%s]\n", g_libs[vlib].dir, victim->name);
    bank_evict(vlib, victim);
    return 1;
}

/* max_keep: 0 = free space only, 1 = may evict unneeded banks, 2 = also prefetches. */
static uint32_t pool_alloc(uint32_t words, int max_keep) {
    if (words > region_words())
        return NOT_RESIDENT;
    for (;;) {
        const uint32_t at = find_gap(words);
        if (at != NOT_RESIDENT)
            return at;
        if (!max_keep || (!evict_one(max_keep, 0) && !(max_keep == 2 && evict_one(2, 1))))
            return NOT_RESIDENT;
    }
}

/* Pads from the data already in SDRAM, optional peak scan / normalise, cache flush. */
static void finish_buffer(volatile int16_t* d, struct sample* s, int normalized) {
    const uint32_t F = s->frames;
    if (!s->peak) {
        int16_t pk = 1;
        for (uint32_t i = 0; i < F; ++i) {
            if (!(i & 8191))
                idle_hook();
            const int16_t v = d[i];
            const int16_t a = v < 0 ? (v == -32768 ? 32767 : -v) : v;
            if (a > pk)
                pk = a;
        }
        s->peak = pk;
    }
    if (!normalized && g_normalize && s->peak < 32767) {
        const int32_t g = (int32_t)((32767u << 16) / (uint32_t)s->peak);
        for (uint32_t i = 0; i < F; ++i) {
            if (!(i & 8191))
                idle_hook();
            const int32_t v = (d[i] * g) >> 16;
            d[i] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
        }
    }
    for (uint32_t i = 0; i < SMP_PREPAD; ++i)
        d[(int32_t)i - SMP_PREPAD] = d[(F - (SMP_PREPAD % F) + i) % F];
    for (uint32_t i = 0; i < SMP_POSTPAD; ++i)
        d[F + i] = d[i % F];
    flush_cpu_dcache();
#ifdef CONFIG_L2_SIZE
    flush_l2_cache(); /* the engine reads SDRAM directly, behind the CPU's L2 */
#endif
}

/* Largest sample a bank holds. */
#define SAMPLE_MAX_FRAMES (BANK_MAX_BYTES / 2 - SMP_PREPAD - SMP_POSTPAD - 64)

/* Replaces every long file (wavutil.h) by its parts. The file list is moved to the top of
   the store and expanded downwards; a file never gets more parts than leave one entry
   for each file after it (MAX_PER_LIB). */
static void split_long(struct lib* L) {
    const int n = L->count, top = MAX_PER_LIB - n;
    memmove(&L->s[top], &L->s[0], (size_t)n * sizeof(struct sample));
    int out = 0;
    for (int i = 0; i < n; ++i) {
        const struct sample f = L->s[top + i];
        const uint32_t len = wav_split_len(f.frames, f.sr, f.name, SAMPLE_MAX_FRAMES);
        uint32_t parts = len ? wav_split_count(f.frames, len) : 1;
        const uint32_t room = (uint32_t)(top + i + 1 - out);
        if (parts > room)
            parts = room;
        if (parts > 0xFFFF)
            parts = 0xFFFF;
        const uint32_t fb = f.ch * (f.bits / 8u);
        for (uint32_t k = 0; k < parts; ++k) {
            struct sample* s = &L->s[out++];
            *s = f;
            if (!len)
                continue;
            const uint32_t start = wav_split_start(f.frames, len, k);
            s->data_off = f.data_off + start * fb;
            s->data_len = len * fb;
            s->frames = len;
            s->seg = (uint16_t)k;
            s->nseg = (uint16_t)parts;
            s->peak = 0; /* per part, cached in .smptr */
            s->bpm_q16 = samples_track_bpm_q16(len, s->sr, &s->beats_log2);
        }
    }
    L->count = out;
}

static const char* base_name(const char* p) {
    const char* sl = strrchr(p, '/');
    return sl ? sl + 1 : p;
}

/* Groups samples [0,n) of a library into banks: subfolders, then size/count packing; the
   parts of a split file fill banks of their own ("name #first-last"). */
static void build_banks(struct lib* L) {
    L->nbanks = 0;
    const uint32_t max_words = BANK_MAX_BYTES / 2;
    char cur[SAMPLE_NAME_LEN] = "";
    struct bank* b = 0;
    uint32_t cap = BANK_MAX_FILES; /* split file: parts per bank, the same in all its banks */
    for (int i = 0; i < L->count; ++i) {
        struct sample* s = &L->s[i];
        char grp[SAMPLE_NAME_LEN] = "";
        const char* sl = strchr(s->name, '/');
        if (s->nseg > 1)
            strcpy(grp, s->name);
        else if (sl) {
            memcpy(grp, s->name, (size_t)(sl - s->name));
            grp[sl - s->name] = 0;
        }
        const uint32_t fp = footprint(s->frames);
        if (s->nseg > 1 && s->seg == 0) {
            uint32_t fit = max_words / fp;
            if (fit > BANK_MAX_FILES)
                fit = BANK_MAX_FILES;
            const uint32_t nb = (s->nseg + fit - 1) / fit;
            cap = (s->nseg + nb - 1) / nb;
        } else if (s->nseg <= 1)
            cap = BANK_MAX_FILES;
        if (!b || strcmp(grp, cur) || (uint32_t)b->count >= cap || (b->words && b->words + fp > max_words)) {
            if (L->nbanks == MAX_BANKS_PER_LIB) {
                L->count = i; /* out of bank slots: ignore the rest */
                break;
            }
            b = &L->b[L->nbanks++];
            memset(b, 0, sizeof(*b));
            b->first = i;
            b->base = NOT_RESIDENT;
            b->want = -1;
            strcpy(cur, grp);
            if (s->nseg <= 1 && grp[0])
                snprintf(b->name, sizeof(b->name), "%s", grp);
            else if (s->nseg <= 1)
                snprintf(b->name, sizeof(b->name), "%d", L->nbanks - 1);
        }
        if (s->nseg > 1) { /* "<stem, 13 chars> #first-last" */
            const char* bn = base_name(s->name);
            const char* dot = strrchr(bn, '.');
            size_t stem = dot ? (size_t)(dot - bn) : strlen(bn);
            if (stem > 13)
                stem = 13;
            memcpy(b->name, bn, stem);
            if (s->seg == L->s[b->first].seg)
                snprintf(b->name + stem, sizeof(b->name) - stem, " #%u", s->seg);
            else
                snprintf(b->name + stem, sizeof(b->name) - stem, " #%u-%u", L->s[b->first].seg, s->seg);
        }
        s->offset = b->words;
        s->ready = 0;
        b->words += fp;
        ++b->count;
    }
}

/* ======================================================================================= */
#ifdef CSR_SDCARD_BASE
/* FatFs disk ops with write support (LiteX's own set is read-only). */
static DSTATUS g_sd_stat = STA_NOINIT;
static DSTATUS sd_status(BYTE d) { return d ? STA_NOINIT : g_sd_stat; }
static DSTATUS sd_init(BYTE d) {
    if (d)
        return STA_NOINIT;
    /* the card may still be mid-transfer from before an FPGA reload: retry */
    for (int i = 0; i < 10 && g_sd_stat; ++i) {
        if (i) {
            (void)sdcard_stop_transmission(); /* CMD12: end a multi-block read left open */
            busy_wait(20 * i);
        }
        g_sd_stat = sdcard_init() ? 0 : STA_NOINIT;
    }
    return g_sd_stat;
}
/* The SD DMA engine needs word-aligned buffers; FatFs reads straight into the caller's
   buffer, which is unaligned e.g. for 24-bit stereo (6-byte frames). Bounce those. */
static uint32_t g_bounce[512 / 4];
static DRESULT sd_read(BYTE d, BYTE* b, LBA_t sec, UINT n) {
    (void)d;
    drain_hook();
    if (((uintptr_t)b & 3) == 0)
        return sdcard_read((uint32_t)sec, n, b) == SD_OK ? RES_OK : RES_ERROR;
    for (UINT i = 0; i < n; ++i, b += 512) {
        if (sdcard_read((uint32_t)sec + i, 1, (uint8_t*)g_bounce) != SD_OK)
            return RES_ERROR;
        memcpy(b, g_bounce, 512);
    }
    return RES_OK;
}
static DRESULT sd_write(BYTE d, const BYTE* b, LBA_t sec, UINT n) {
    (void)d;
    drain_hook();
    if (((uintptr_t)b & 3) == 0)
        return sdcard_write((uint32_t)sec, n, (uint8_t*)b) == SD_OK ? RES_OK : RES_ERROR;
    for (UINT i = 0; i < n; ++i, b += 512) {
        memcpy(g_bounce, b, 512);
        if (sdcard_write((uint32_t)sec + i, 1, (uint8_t*)g_bounce) != SD_OK)
            return RES_ERROR;
    }
    return RES_OK;
}
static DRESULT sd_ioctl(BYTE d, BYTE cmd, void* buf) {
    (void)d;
    (void)buf;
    return cmd == CTRL_SYNC ? RES_OK : RES_PARERR;
}
static DISKOPS g_sd_ops = {sd_init, sd_status, sd_read, sd_write, sd_ioctl};
static FATFS g_fs;

static uint32_t rd32(const uint8_t* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

/* ---- per-folder index file -------------------------------------------------------- */
#define IDX_MAGIC 0x58444953u /* 'SIDX' */
#define IDX_VERSION 2         /* 2: + fmt; version 1 files (PCM only) are still read */
struct idx_hdr { uint32_t magic, version, count, rec_size; };
struct idx_rec {              /* one per file (the parts of a split file are derived) */
    char     name[SAMPLE_NAME_LEN];
    uint32_t fsize;
    uint16_t fdate, ftime;
    uint32_t data_off, data_len, sr, frames;
    uint8_t  ch, bits;
    int16_t  peak;            /* 0 for split files: the parts' peaks are in .smptr */
    uint8_t  fmt, pad[3];     /* 0 (version 1): PCM */
};
static struct idx_rec g_idx[MAX_PER_LIB];
static int g_idx_dirty[MAX_LIBS];

static void path_of(char* out, int n, int lib, const char* name) { snprintf(out, (size_t)n, "%s/%s", g_dirs[lib], name); }

static int idx_load(int lib) {
    char p[32];
    FIL f;
    UINT n;
    struct idx_hdr h;
    snprintf(p, sizeof(p), "%s/.smpidx", g_dirs[lib]);
    if (f_open(&f, p, FA_READ) != FR_OK)
        return 0;
    int cnt = 0;
    if (f_read(&f, &h, sizeof(h), &n) == FR_OK && n == sizeof(h) && h.magic == IDX_MAGIC
        && (h.version == 1 || h.version == IDX_VERSION) && h.rec_size >= offsetof(struct idx_rec, fmt) && h.rec_size <= sizeof(struct idx_rec)
        && h.count <= MAX_PER_LIB) {
        if (f_read(&f, g_idx, h.count * h.rec_size, &n) == FR_OK && n == h.count * h.rec_size) {
            cnt = (int)h.count;
            /* older, shorter records: spread out from the end, zero the new fields */
            for (int i = cnt - 1; h.rec_size < sizeof(struct idx_rec) && i >= 0; --i) {
                memmove(&g_idx[i], (uint8_t*)g_idx + (uint32_t)i * h.rec_size, h.rec_size);
                memset((uint8_t*)&g_idx[i] + h.rec_size, 0, sizeof(struct idx_rec) - h.rec_size);
            }
        }
    }
    f_close(&f);
    return cnt;
}

static void idx_save(int lib) {
    char p[32];
    FIL f;
    UINT n;
    const struct lib* L = &g_libs[lib];
    snprintf(p, sizeof(p), "%s/.smpidx", g_dirs[lib]);
    if (f_open(&f, p, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return;
    uint32_t files = 0;
    for (int i = 0; i < L->count; ++i)
        files += L->s[i].seg == 0;
    const struct idx_hdr h = {IDX_MAGIC, IDX_VERSION, files, sizeof(struct idx_rec)};
    f_write(&f, &h, sizeof(h), &n);
    for (int i = 0; i < L->count; ++i) {
        const struct sample* s = &L->s[i];
        if (s->seg) /* a split file is one record, from its first part */
            continue;
        struct idx_rec r;
        memset(&r, 0, sizeof(r));
        memcpy(r.name, s->name, SAMPLE_NAME_LEN);
        r.fsize = s->fsize;
        r.fdate = s->fdate;
        r.ftime = s->ftime;
        r.data_off = s->data_off;
        r.data_len = s->file_len;
        r.sr = s->sr;
        r.frames = s->file_len / (s->ch * (s->bits / 8u));
        r.ch = s->ch;
        r.bits = s->bits;
        r.peak = s->nseg > 1 ? 0 : s->peak;
        r.fmt = s->fmt;
        f_write(&f, &r, sizeof(r), &n);
    }
    f_close(&f);
    g_idx_dirty[lib] = 0;
}

/* Parses the RIFF header: fills format, data offset/length, frames. 0 on success. */
static int parse_wav(int lib, struct sample* s) {
    char path[SAMPLE_NAME_LEN + 16];
    FIL f;
    UINT n;
    path_of(path, sizeof(path), lib, s->name);
    if (f_open(&f, path, FA_READ) != FR_OK)
        return -1;
    uint8_t h[12];
    int rc = -2;
    if (f_read(&f, h, 12, &n) != FR_OK || n != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4))
        goto out;
    uint16_t fmt = 0;
    for (;;) {
        uint8_t c[8];
        if (f_read(&f, c, 8, &n) != FR_OK || n != 8) {
            rc = -3;
            goto out;
        }
        const uint32_t len = rd32(c + 4);
        if (!memcmp(c, "fmt ", 4)) {
            uint8_t b[40];
            const UINT want = len < sizeof(b) ? len : sizeof(b);
            if (f_read(&f, b, want, &n) != FR_OK || n != want) {
                rc = -3;
                goto out;
            }
            fmt = rd16(b);
            s->ch = (uint8_t)rd16(b + 2);
            s->sr = rd32(b + 4);
            s->bits = (uint8_t)rd16(b + 14);
            if (fmt == 0xFFFE && len >= 26) /* WAVE_FORMAT_EXTENSIBLE: sub-format GUID */
                fmt = rd16(b + 24);
            if (rd16(b + 2) > 255 || rd16(b + 14) > 255)
                s->ch = 0;
            f_lseek(&f, f_tell(&f) + (len - want) + (len & 1));
        } else if (!memcmp(c, "data", 4)) {
            s->data_off = (uint32_t)f_tell(&f);
            s->data_len = len;
            break;
        } else
            f_lseek(&f, f_tell(&f) + len + (len & 1));
    }
    if (!wav_supported(fmt, s->ch, s->bits) || !s->sr) {
        printf("  skip %s (format %u, %u ch, %u bit: not PCM / float)\n", s->name, fmt, s->ch, s->bits);
        rc = -4;
        goto out;
    }
    s->fmt = (uint8_t)fmt;
    if (s->data_off + s->data_len > s->fsize) /* truncated / streaming writers */
        s->data_len = s->fsize - s->data_off;
    s->frames = s->data_len / (s->ch * (s->bits / 8u));
    s->file_len = s->frames * (s->ch * (s->bits / 8u));
    rc = s->frames >= 128 ? 0 : -5;
out:
    f_close(&f);
    return rc;
}

static int ends_with_wav(const char* s) {
    const size_t l = strlen(s);
    if (l < 4)
        return 0;
    const char* e = s + l - 4;
    return e[0] == '.' && (e[1] | 32) == 'w' && (e[2] | 32) == 'a' && (e[3] | 32) == 'v';
}

/* loose files first, then subfolders; by name inside each */
static int name_order(const char* a, const char* b) {
    const int sa = strchr(a, '/') != 0, sb = strchr(b, '/') != 0;
    return sa != sb ? sa - sb : strcmp(a, b);
}

static void sort_lib(struct lib* L) {
    struct sample t;
    for (int gap = L->count / 2; gap > 0; gap /= 2)
        for (int i = gap; i < L->count; ++i)
            for (int j = i; j >= gap && name_order(L->s[j - gap].name, L->s[j].name) > 0; j -= gap) {
                t = L->s[j];
                L->s[j] = L->s[j - gap];
                L->s[j - gap] = t;
            }
}

static void add_dir_wavs(struct lib* L, const char* dir, const char* prefix) {
    DIR dj;
    FILINFO fi;
    if (f_opendir(&dj, dir) != FR_OK)
        return;
    while (L->count < MAX_PER_LIB && f_readdir(&dj, &fi) == FR_OK && fi.fname[0]) {
        if ((fi.fattrib & AM_DIR) || fi.fname[0] == '.' || !ends_with_wav(fi.fname))
            continue;
        if (strlen(prefix) + strlen(fi.fname) >= SAMPLE_NAME_LEN) {
            printf("  skip (name too long): %s\n", fi.fname);
            continue;
        }
        struct sample* s = &L->s[L->count++];
        memset(s, 0, sizeof(*s));
        s->ntr = TR_NONE;
        snprintf(s->name, sizeof(s->name), "%s%s", prefix, fi.fname);
        s->fsize = (uint32_t)fi.fsize;
        s->fdate = fi.fdate;
        s->ftime = fi.ftime;
    }
    f_closedir(&dj);
}

/* ---- transients: pool + per-folder cache file ------------------------------------- */
#define TR_POOL (MAX_LIBS * MAX_PER_LIB * TR_MAX)
static uint32_t g_tr_pool[TR_POOL];
static uint32_t g_tr_used;
static int g_tr_dirty[MAX_LIBS];
#define TRF_MAGIC 0x4E525453u /* 'STRN' */
#define TRF_VERSION 1
struct trf_hdr { uint32_t magic, version, count, rec_size; };
/* peak: of the 16-bit copy (0 in older files); the only place the peak of a part is kept */
struct trf_rec { uint32_t fsize; uint16_t fdate, ftime; uint32_t hash; uint8_t n, pad; int16_t peak; };

static uint32_t fnv(uint32_t h, const uint8_t* p, uint32_t n) {
    while (n--)
        h = (h ^ *p++) * 16777619u;
    return h;
}

/* name hash; a part also hashes its place in the file, so other split settings miss */
static uint32_t sample_key(const struct sample* s) {
    uint32_t h = fnv(2166136261u, (const uint8_t*)s->name, (uint32_t)strlen(s->name));
    if (s->nseg > 1) {
        const uint32_t w[3] = {s->data_off, s->frames, s->nseg};
        h = fnv(h, (const uint8_t*)w, sizeof(w));
    }
    return h;
}

int samples_transients(const struct sample* s, const uint32_t** pos) {
    if (!s || s->ntr == TR_NONE)
        return -1;
    *pos = &g_tr_pool[s->tr];
    return s->ntr;
}

static void tr_store(struct sample* s, const uint32_t* p, int n) {
    if (n < 0)
        n = 0;
    if (n > TR_MAX)
        n = TR_MAX;
    if (g_tr_used + (uint32_t)n > TR_POOL)
        return;
    for (int k = 0; k < n; ++k)
        g_tr_pool[g_tr_used + (uint32_t)k] = p[k];
    s->tr = g_tr_used;
    s->ntr = (uint8_t)n;
    g_tr_used += (uint32_t)n;
}

/* date/time 0: written by the PC tool (host/prepare_sd.cpp), which can't know the FAT
   timestamps: matched by name and size, rewritten with the real ones */
static int stamp_match(uint16_t d, uint16_t t, uint16_t rd, uint16_t rt) { return (d == rd && t == rt) || (!rd && !rt); }

static int trf_match(const struct sample* s, const struct trf_rec* r) {
    return s->fsize == r->fsize && stamp_match(s->fdate, s->ftime, r->fdate, r->ftime) && sample_key(s) == r->hash;
}

static int trf_load(int lib) {
    char p[32];
    FIL f;
    UINT n;
    struct trf_hdr h;
    struct lib* L = &g_libs[lib];
    snprintf(p, sizeof(p), "%s/.smptr", g_dirs[lib]);
    if (f_open(&f, p, FA_READ) != FR_OK)
        return 0;
    int got = 0, cur = 0;
    if (f_read(&f, &h, sizeof(h), &n) == FR_OK && n == sizeof(h) && h.magic == TRF_MAGIC && h.version == TRF_VERSION
        && h.rec_size == sizeof(struct trf_rec)) {
        for (uint32_t i = 0; i < h.count; ++i) {
            struct trf_rec r;
            uint32_t pos[TR_MAX];
            if (f_read(&f, &r, sizeof(r), &n) != FR_OK || n != sizeof(r) || r.n > TR_MAX)
                break;
            if (r.n && (f_read(&f, pos, r.n * 4u, &n) != FR_OK || n != r.n * 4u))
                break;
            /* records are written in library order: search on from the last match */
            struct sample* s = 0;
            for (int k = 0; !s && k < L->count; ++k) {
                const int j = (cur + k) % L->count;
                if (trf_match(&L->s[j], &r)) {
                    s = &L->s[j];
                    cur = j + 1;
                }
            }
            if (s && s->ntr == TR_NONE) {
                tr_store(s, pos, r.n);
                if (!s->peak)
                    s->peak = r.peak;
                if (!r.fdate && !r.ftime)
                    g_tr_dirty[lib] = 1; /* store the real timestamps */
                ++got;
            }
        }
    }
    f_close(&f);
    return got;
}

static void trf_save(int lib) {
    char p[32];
    FIL f;
    UINT n;
    const struct lib* L = &g_libs[lib];
    snprintf(p, sizeof(p), "%s/.smptr", g_dirs[lib]);
    if (f_open(&f, p, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return;
    uint32_t cnt = 0;
    for (int i = 0; i < L->count; ++i)
        cnt += L->s[i].ntr != TR_NONE;
    const struct trf_hdr h = {TRF_MAGIC, TRF_VERSION, cnt, sizeof(struct trf_rec)};
    f_write(&f, &h, sizeof(h), &n);
    for (int i = 0; i < L->count; ++i) {
        const struct sample* s = &L->s[i];
        if (s->ntr == TR_NONE)
            continue;
        struct trf_rec r;
        memset(&r, 0, sizeof(r));
        r.fsize = s->fsize;
        r.fdate = s->fdate;
        r.ftime = s->ftime;
        r.hash = sample_key(s);
        r.n = s->ntr;
        r.peak = s->peak;
        f_write(&f, &r, sizeof(r), &n);
        if (s->ntr)
            f_write(&f, &g_tr_pool[s->tr], s->ntr * 4u, &n);
    }
    f_close(&f);
    g_tr_dirty[lib] = 0;
}

static int16_t tr_get(void* ctx, uint32_t i) { return ((volatile int16_t*)ctx)[i]; }

static int index_lib(int l) {
    struct lib* L = &g_libs[l];
    DIR dj;
    FILINFO fi;
    static char subs[MAX_BANKS_PER_LIB][32];
    int nsub = 0;
    add_dir_wavs(L, g_dirs[l], "");
    if (f_opendir(&dj, g_dirs[l]) == FR_OK) {
        while (nsub < MAX_BANKS_PER_LIB && f_readdir(&dj, &fi) == FR_OK && fi.fname[0])
            if ((fi.fattrib & AM_DIR) && fi.fname[0] != '.' && strlen(fi.fname) < sizeof(subs[0]))
                strcpy(subs[nsub++], fi.fname);
        f_closedir(&dj);
    }
    for (int i = 0; i < nsub; ++i) {
        char dir[48], prefix[36];
        snprintf(dir, sizeof(dir), "%s/%s", g_dirs[l], subs[i]);
        snprintf(prefix, sizeof(prefix), "%s/", subs[i]);
        add_dir_wavs(L, dir, prefix);
    }
    sort_lib(L);

    /* merge with the index file (both sorted with name_order) */
    const int ni = idx_load(l);
    int j = 0, parsed = 0, cached = 0, out = 0;
    for (int i = 0; i < L->count; ++i) {
        struct sample* s = &L->s[i];
        while (j < ni && name_order(g_idx[j].name, s->name) < 0)
            ++j;
        const struct idx_rec* r = (j < ni && !strcmp(g_idx[j].name, s->name)) ? &g_idx[j] : 0;
        if (r && r->fsize == s->fsize && stamp_match(s->fdate, s->ftime, r->fdate, r->ftime) && r->frames) {
            if (!r->fdate && !r->ftime)
                g_idx_dirty[l] = 1; /* store the real timestamps */
            s->data_off = r->data_off;
            s->data_len = r->data_len;
            s->sr = r->sr;
            s->frames = r->frames;
            s->ch = r->ch;
            s->bits = r->bits;
            s->peak = r->peak;
            s->fmt = r->fmt ? r->fmt : WAV_PCM;
            ++cached;
        } else {
            const int rc = parse_wav(l, s);
            ++parsed;
            if (rc) {
                if (rc != -4)
                    printf("  skip %s (%s)\n", s->name, rc == -1 ? "cannot open" : rc == -5 ? "too short" : "not a WAV");
                continue;
            }
            g_idx_dirty[l] = 1;
        }
        s->file_len = s->data_len;
        s->seg = 0;
        s->nseg = 1;
        s->bpm_q16 = samples_track_bpm_q16(s->frames, s->sr, &s->beats_log2);
        L->s[out++] = *s;
    }
    if (out != L->count || ni != L->count)
        g_idx_dirty[l] = 1;
    L->count = out;
    if (g_idx_dirty[l])
        idx_save(l);
    split_long(L);
    build_banks(L);
    const int ntr = trf_load(l);
    int split = 0;
    for (int i = 0; i < L->count; ++i)
        split += L->s[i].nseg > 1 && L->s[i].seg == 0;
    printf("SD: %-8s %4d WAVs, %4d samples in %3d banks (%d from index, %d parsed, %d split; %d analysed)\n",
           g_dirs[l], out, L->count, L->nbanks, cached, parsed, split, ntr);
    return L->count;
}

/* ---- background loader -------------------------------------------------------------- */
struct req { int8_t lib, prio; int16_t bank; };
#define MAX_REQ 16
static struct req g_req[MAX_REQ];
static int g_nreq;

static struct {
    int active, lib, bank, idx, prio, req_lib, req_bank;
    FIL f;
    uint32_t done, gain, t0;
    int normalized;
} g_job;
static uint8_t g_io[32768] __attribute__((aligned(4)));

static uint32_t g_load_bytes, g_load_ms, g_load_count;
static uint32_t g_idle_since;

static int bank_complete(int lib, const struct bank* b) {
    for (int i = 0; i < b->count; ++i)
        if (!g_libs[lib].s[b->first + i].ready)
            return 0;
    return 1;
}

static void drop_req(int k);

void samples_want(int lib, int bank, int idx, int prio) {
    struct bank* b = samples_bank(lib, bank);
    if (!b || !g_card_ok || b->builtin)
        return;
    b->lru = ++g_lru_clock;
    if (idx >= 0 && idx < b->count && (prio || b->want < 0))
        b->want = (int16_t)idx;
    if (b->state == BANK_RESIDENT)
        return;
    if (b->blocked) { /* the pool had no room: retry later, don't stall the loader */
        if ((int32_t)(now_samples() - b->retry_at) < 0)
            return;
        b->blocked = 0;
    }
    /* requests for banks no slot plays, wants or prefetches any more (a bank knob swept
       past them) go first: they used to fill the queue, and new requests were dropped */
    for (int i = 0; i < g_nreq;)
        if (!samples_bank_keep(g_req[i].lib, g_req[i].bank))
            drop_req(i);
        else
            ++i;
    for (int i = 0; i < g_nreq; ++i)
        if (g_req[i].lib == lib && g_req[i].bank == bank) {
            struct req r = g_req[i];
            if (prio > r.prio)
                r.prio = (int8_t)prio;
            if (prio) { /* re-queue as the newest intent */
                drop_req(i);
                g_req[g_nreq++] = r;
            } else
                g_req[i] = r;
            return;
        }
    if (g_nreq == MAX_REQ) { /* drop the oldest prefetch */
        int k = -1;
        for (int i = 0; i < g_nreq && k < 0; ++i)
            if (!g_req[i].prio)
                k = i;
        if (k < 0)
            return;
        memmove(&g_req[k], &g_req[k + 1], (size_t)(g_nreq - k - 1) * sizeof(g_req[0]));
        --g_nreq;
    }
    g_req[g_nreq++] = (struct req){(int8_t)lib, (int8_t)prio, (int16_t)bank};
}

/* 2: a slot waits for this bank's wanted sample, 1: the rest of a bank a slot plays or
   wants, 0: prefetch. A request only stays urgent while a slot plays or waits for the bank;
   a waiting slot goes before another slot's bank filling up in the background. */
static int eff_prio(const struct req* r) {
    if (samples_bank_keep(r->lib, r->bank) != 2)
        return 0;
    const struct bank* b = samples_bank(r->lib, r->bank);
    return (b && b->want >= 0 && b->want < b->count && !g_libs[(int)r->lib].s[b->first + b->want].ready) ? 2 : 1;
}

/* Newest request with the highest effective priority (latest intent wins). */
static int pick_req(void) {
    int k = -1;
    for (int i = 0; i < g_nreq; ++i)
        if (k < 0 || eff_prio(&g_req[i]) >= eff_prio(&g_req[k]))
            k = i;
    return k;
}

static void drop_req(int k) {
    memmove(&g_req[k], &g_req[k + 1], (size_t)(g_nreq - k - 1) * sizeof(g_req[0]));
    --g_nreq;
}

/* Picks the next file to load. Returns 1 if a job started. */
static int job_start(void) {
    for (;;) {
        const int k = pick_req();
        if (k < 0)
            return 0;
        const struct req r = g_req[k];
        struct lib* L = &g_libs[(int)r.lib];
        struct bank* b = samples_bank(r.lib, r.bank);
        const int keep = samples_bank_keep(r.lib, r.bank);
        if (!b || !keep) { /* nobody needs it any more */
            drop_req(k);
            continue;
        }
        if (b->base == NOT_RESIDENT) {
            g_job_bank = b; /* not evictable while we allocate for it */
            const uint32_t at = pool_alloc(b->words, keep == 2 ? 2 : 1);
            g_job_bank = 0;
            if (at == NOT_RESIDENT) {
                static const struct bank* last_msg;
                if (eff_prio(&r) && last_msg != b) {
                    printf("SD: no room for bank %s/%s (%lu KB), retrying\n", L->dir, b->name,
                           (unsigned long)(b->words / 512));
                    last_msg = b;
                }
                b->blocked = 1;
                b->retry_at = now_samples() + 48000;
                drop_req(k);
                continue;
            }
            b->base = at;
            b->state = BANK_LOADING;
        }
        int idx = -1;
        if (b->want >= 0 && b->want < b->count && !L->s[b->first + b->want].ready)
            idx = b->want;
        for (int i = 0; i < b->count && idx < 0; ++i)
            if (!L->s[b->first + i].ready)
                idx = i;
        if (idx < 0) {
            b->state = BANK_RESIDENT;
            drop_req(k);
            continue;
        }
        struct sample* s = &L->s[b->first + idx];
        char path[SAMPLE_NAME_LEN + 16];
        path_of(path, sizeof(path), r.lib, s->name);
        if (f_open(&g_job.f, path, FA_READ) != FR_OK || f_lseek(&g_job.f, s->data_off) != FR_OK) {
            printf("SD: cannot open %s\n", path);
            s->ready = 1; /* silence rather than retrying forever */
            continue;
        }
        g_job.active = 1;
        g_job.lib = r.lib;
        g_job.bank = r.bank;
        g_job.idx = idx;
        /* urgent: the very sample a slot is waiting for */
        g_job.prio = eff_prio(&r) && idx == b->want;
        g_job.req_lib = r.lib;
        g_job.req_bank = r.bank;
        g_job.done = 0;
        g_job.normalized = g_normalize && s->peak > 0 && s->peak < 32767;
        g_job.gain = g_job.normalized ? (32767u << 16) / (uint32_t)s->peak : 65536;
        g_job.t0 = now_samples();
        g_job_bank = b;
        if (g_sd_verbose)
            printf("[load %s/%s:%d %s%s]\n", L->dir, b->name, idx, s->name, g_job.prio ? " (wanted)" : "");
        return 1;
    }
}

/* Converts one chunk. Returns 1 when the file is complete. */
static int job_chunk(void) {
    struct bank* b = samples_bank(g_job.lib, g_job.bank);
    struct sample* s = samples_get(g_job.lib, g_job.bank, g_job.idx);
    const uint32_t bps = s->bits / 8u, fb = bps * s->ch;
    uint32_t want = s->frames - g_job.done;
    if (want > sizeof(g_io) / fb)
        want = sizeof(g_io) / fb;
    while (want > 4 && ((want * fb) & 3)) /* keep reads word aligned (fast DMA path) */
        --want;
    UINT n = 0;
    if (want && f_read(&g_job.f, g_io, want * fb, &n) != FR_OK)
        n = 0;
    idle_hook();
    want = n / fb;
    int16_t* d = (int16_t*)(g_mem + b->base + s->offset + SMP_PREPAD + g_job.done); /* flushed at finish */
    for (uint32_t i0 = 0; i0 < want; i0 += 4096) { /* 4096-frame slices, hook in between */
        const uint32_t i1 = (want - i0 > 4096) ? i0 + 4096 : want;
        wav_convert(d + i0, g_io + i0 * fb, i1 - i0, s->fmt, s->ch, s->bits, (int32_t)g_job.gain);
        idle_hook();
    }
    g_job.done += want;
    g_load_bytes += n;
    return want == 0 || g_job.done >= s->frames;
}

static void job_finish(void) {
    struct bank* b = samples_bank(g_job.lib, g_job.bank);
    struct sample* s = samples_get(g_job.lib, g_job.bank, g_job.idx);
    f_close(&g_job.f);
    if (g_job.done < s->frames) /* short read: keep the frame count (bank layout), zero the rest */
        for (uint32_t i = g_job.done; i < s->frames; ++i)
            g_mem[b->base + s->offset + SMP_PREPAD + i] = 0;
    const int had_peak = s->peak != 0;
    finish_buffer(g_mem + b->base + s->offset + SMP_PREPAD, s, g_job.normalized);
    if (!had_peak)
        g_idx_dirty[g_job.lib] = 1;
    s->ready = 1;
    if (bank_complete(g_job.lib, b))
        b->state = BANK_RESIDENT;
    if (s->ntr == TR_NONE) { /* first load: find the transients (cached in .smptr) */
        uint32_t pos[TR_MAX];
        const int n = tr_analyse(tr_get, (void*)(g_mem + b->base + s->offset + SMP_PREPAD), s->frames, pos, idle_hook);
        tr_store(s, pos, n);
        g_tr_dirty[g_job.lib] = 1;
    }
    const uint32_t ms = (now_samples() - g_job.t0) / 48;
    g_load_ms += ms;
    ++g_load_count;
    if (g_sd_verbose)
        printf("[done %s %lu ms]\n", s->name, (unsigned long)ms);
    g_job.active = 0;
    g_job_bank = 0;
}

int samples_poll(void) {
    if (!g_card_ok)
        return 0;
    if (g_job.active) {
        /* a slot waits for a sample that is not this one: drop this file if it is background
           fill, or the wanted sample of a bank no slot wants any more (knob sweeps); it is
           reloaded later from the start */
        const int k = pick_req();
        if (k >= 0 && eff_prio(&g_req[k]) == 2
            && !(g_req[k].lib == g_job.lib && g_req[k].bank == g_job.bank
                 && samples_bank(g_job.lib, g_job.bank)->want == g_job.idx)
            && (!g_job.prio || samples_bank_keep(g_job.lib, g_job.bank) != 2)) {
            f_close(&g_job.f);
            g_job.active = 0;
            g_job_bank = 0;
        }
    }
    if (!g_job.active) {
        if (!job_start()) {
            /* idle: persist newly measured peaks after 2 s without loads */
            if (now_samples() - g_idle_since > 96000)
                for (int l = 0; l < MAX_LIBS; ++l) {
                    if (g_idx_dirty[l])
                        idx_save(l);
                    if (g_tr_dirty[l])
                        trf_save(l);
                }
            return 0;
        }
    }
    g_idle_since = now_samples();
    if (job_chunk()) {
        job_finish();
        return 1;
    }
    return 0;
}

static uint8_t g_bench[32768] __attribute__((aligned(4)));
void samples_sdbench(void) {
    /* raw multi-block reads, then FatFs f_read with 8 KB and 32 KB requests */
    if (!g_card_ok || !g_libs[0].count)
        return;
    uint32_t t0 = now_samples();
    for (uint32_t blk = 0; blk < 4096; blk += 64)
        sdcard_read(32768 + blk, 64, g_bench);
    uint32_t ms = (now_samples() - t0) / 48;
    printf("raw sdcard_read 64-block: 2048 KB in %lu ms = %lu KB/s\n", (unsigned long)ms,
           (unsigned long)(ms ? 2048000 / ms : 0));
    for (int pass = 0; pass < 2; ++pass) {
        const UINT chunk = pass ? 32768 : 8192;
        char path[SAMPLE_NAME_LEN + 16];
        FIL f;
        UINT n;
        uint32_t total = 0;
        t0 = now_samples();
        for (int i = 0; i < g_libs[0].count && total < (4u << 20); ++i) {
            path_of(path, sizeof(path), 0, g_libs[0].s[i].name);
            if (f_open(&f, path, FA_READ) != FR_OK)
                continue;
            while (f_read(&f, g_bench, chunk, &n) == FR_OK && n)
                total += n;
            f_close(&f);
        }
        ms = (now_samples() - t0) / 48;
        printf("f_read %5u B chunks: %lu KB in %lu ms = %lu KB/s\n", chunk, (unsigned long)(total / 1024),
               (unsigned long)ms, (unsigned long)(ms ? total / ms : 0));
    }
}

int samples_read_file(const char* path, char* buf, int n) {
    FIL f;
    UINT got = 0;
    if (!g_card_ok || n < 1 || f_open(&f, path, FA_READ) != FR_OK)
        return -1;
    if (f_read(&f, buf, (UINT)(n - 1), &got) != FR_OK)
        got = 0;
    f_close(&f);
    buf[got] = 0;
    return (int)got;
}

int samples_write_file(const char* path, const char* buf, int n) {
    FIL f;
    UINT put = 0;
    if (!g_card_ok || f_open(&f, path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return -1;
    const FRESULT r = f_write(&f, buf, (UINT)n, &put);
    f_close(&f);
    return (r == FR_OK && put == (UINT)n) ? 0 : -1;
}

int samples_init(void) {
    if (g_job.active) { /* rescan while loading: drop the job before the table is rebuilt */
        f_close(&g_job.f);
        g_job.active = 0;
        g_job_bank = 0;
    }
    for (int l = 0; g_card_ok && l < MAX_LIBS; ++l) { /* rescan: keep what was measured */
        if (g_idx_dirty[l])
            idx_save(l);
        if (g_tr_dirty[l])
            trf_save(l);
    }
    g_tr_used = 0;
    g_nreq = 0;
    memset(g_libs, 0, sizeof(g_libs));
    for (int l = 0; l < MAX_LIBS; ++l) {
        g_libs[l].s = g_store[l];
        g_libs[l].b = g_bstore[l];
        strcpy(g_libs[l].dir, g_dirs[l] + 1);
    }
    g_card_ok = 0;
    FfDiskOps = &g_sd_ops;
    const FRESULT mr = f_mount(&g_fs, "", 1);
    if (mr != FR_OK) {
        printf("SD: mount failed (%d)\n", mr);
        f_mount(0, "", 0);
        return 0;
    }
    g_card_ok = 1;
    const uint32_t t0 = now_samples();
    int total = 0;
    for (int l = 0; l < MAX_LIBS; ++l)
        total += index_lib(l);
    printf("SD: indexed %d WAVs in %lu ms\n", total, (unsigned long)((now_samples() - t0) / 48));
    return total;
}

#else  /* no SD card controller */
int samples_init(void) { return 0; }
int samples_read_file(const char* path, char* buf, int n) { (void)path; (void)buf; (void)n; return -1; }
int samples_write_file(const char* path, const char* buf, int n) { (void)path; (void)buf; (void)n; return -1; }
void samples_want(int lib, int bank, int idx, int prio) { (void)lib; (void)bank; (void)idx; (void)prio; }
int samples_poll(void) { return 0; }
static uint32_t g_load_bytes, g_load_ms, g_load_count;
#endif

/* Two synthetic 8-beat loops (120 and 172 BPM) so the engine can be heard without a card. */
int samples_load_builtin(void) {
    memset(g_libs, 0, sizeof(g_libs));
    for (int l = 0; l < MAX_LIBS; ++l) {
        g_libs[l].s = g_store[l];
        g_libs[l].b = g_bstore[l];
    }
#ifdef CSR_SDCARD_BASE
    g_card_ok = 0;
#endif
    static const uint32_t bpms[2] = {120, 172};
    struct lib* L = &g_libs[0];
    strcpy(L->dir, "builtin");
    for (int k = 0; k < 2; ++k) {
        struct sample* s = &L->s[k];
        memset(s, 0, sizeof(*s));
        s->frames = 48000u * 60u * 8u / bpms[k];
        s->sr = 48000;
        s->ch = 1;
        s->bits = 16;
        snprintf(s->name, sizeof(s->name), "synth_%lubpm", (unsigned long)bpms[k]);
        s->bpm_q16 = samples_track_bpm_q16(s->frames, 48000, &s->beats_log2);
        s->peak = 32767;
    }
    L->count = 2;
    build_banks(L);
    struct bank* b = &L->b[0];
    b->builtin = 1;
    b->base = pool_alloc(b->words, 0);
    b->state = BANK_RESIDENT;
    for (int k = 0; k < 2; ++k) {
        struct sample* s = &L->s[k];
        volatile int16_t* d = g_mem + b->base + s->offset + SMP_PREPAD;
        const uint32_t frames = s->frames, beat = frames / 8, sixteenth = beat / 4;
        uint32_t x = 1;
        int32_t ph = 0;
        for (uint32_t i = 0; i < frames; ++i) {
            const uint32_t bt = i / beat, t = i % beat, t16 = i % sixteenth;
            x = x * 1664525u + 1013904223u;
            int32_t v = 0;
            if (!(bt & 1) && t < 9000) {
                const int32_t env = 9000 - (int32_t)t;
                ph += 2 * 400 * 65536 / 48000 + (int32_t)(env / 6);
                const int32_t tri = (ph & 0x8000) ? (0xFFFF - (ph & 0xFFFF)) : (ph & 0xFFFF);
                v += ((tri - 16384) * env) / 9000;
            }
            if ((bt & 1) && t < 6000)
                v += ((int32_t)(x >> 17) - 16384) * (6000 - (int32_t)t) / 12000;
            if (t16 < 700)
                v += ((int32_t)((x >> 9) & 0x3FFF) - 8192) * (700 - (int32_t)t16) / 1400;
            d[i] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
        }
        finish_buffer(d, s, 1);
        s->ready = 1;
    }
    return L->count;
}

static const char* bank_state(const struct bank* b) {
    return b->state == BANK_RESIDENT ? "in RAM" : b->state == BANK_LOADING ? "loading" : "";
}

void samples_list(int lib, int bank) {
    for (int l = 0; l < MAX_LIBS; ++l) {
        const struct lib* L = &g_libs[l];
        if (!L->count || (lib >= 0 && l != lib))
            continue;
        if (bank < 0) {
            printf("lib %d (%s): %d samples, %d banks\n", l, L->dir, L->count, L->nbanks);
            for (int i = 0; i < L->nbanks; ++i) {
                const struct bank* b = &L->b[i];
                printf("  bank %3d %-24s %3d files %5lu KB  %s\n", i, b->name, b->count,
                       (unsigned long)(b->words / 512), bank_state(b));
            }
            continue;
        }
        const struct bank* b = samples_bank(l, bank);
        if (!b)
            continue;
        printf("lib %d bank %d (%s) %s:\n", l, bank, b->name, bank_state(b));
        for (int i = 0; i < b->count; ++i) {
            const struct sample* s = &L->s[b->first + i];
            const uint32_t ms = (uint32_t)((uint64_t)s->frames * 1000 / s->sr);
            char nm[SAMPLE_NAME_LEN + 8];
            if (s->nseg > 1)
                snprintf(nm, sizeof(nm), "%s#%u", s->name, s->seg);
            else
                snprintf(nm, sizeof(nm), "%s", s->name);
            printf("  %3d %c %-40s %6lu ms %5lu Hz %uch %2ub%s  %3lu.%02lu bpm  2^%ld beats", i, s->ready ? '*' : ' ',
                   nm, (unsigned long)ms, (unsigned long)s->sr, s->ch, s->bits, s->fmt == WAV_FLOAT ? "f" : "",
                   (unsigned long)(s->bpm_q16 >> 16), (unsigned long)(((s->bpm_q16 & 0xFFFF) * 100) >> 16),
                   (long)s->beats_log2);
            if (s->ntr == TR_NONE)
                printf("  transients ?\n");
            else
                printf("  %u transients\n", s->ntr);
        }
    }
}

void samples_stats(void) {
    uint32_t used = 0, n = 0;
    for (int l = 0; l < MAX_LIBS; ++l)
        for (int i = 0; i < g_libs[l].nbanks; ++i)
            if (g_libs[l].b[i].base != NOT_RESIDENT) {
                used += g_libs[l].b[i].words;
                ++n;
            }
    printf("pool: %lu of %lu regions (%lu KB each), %lu KB used; loads %lu, %lu KB in %lu ms (%lu KB/s)\n",
           (unsigned long)n, (unsigned long)((pool_end_words() - POOL_START_WORDS) / region_words()),
           (unsigned long)(region_words() / 512), (unsigned long)(used / 512),
           (unsigned long)g_load_count, (unsigned long)(g_load_bytes / 1024), (unsigned long)g_load_ms,
           (unsigned long)(g_load_ms ? g_load_bytes / g_load_ms : 0));
}
