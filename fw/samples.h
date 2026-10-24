/* Sample library: WAV files on the SD card, held in SDRAM per bank.
 *
 * Libraries are the top folders (/drums, /melody, /bass, /kick). A library is split
 * into banks: every subfolder is a bank, loose files are packed in name order into
 * banks of at most BANK_MAX_BYTES / 127 files. A bank is the unit of SDRAM residency:
 * sample changes inside the playing bank are instant, a bank change loads in the
 * background (requested sample first) while the old bank keeps playing.
 *
 * Long files are split (fw/wavutil.h): a WAV longer than 60 s or larger than a bank
 * becomes a run of 4-bar parts ("name#k"), each a sample of its own; the parts of a file
 * fill banks of their own, so program change steps through the file and CC1 picks
 * the part inside the bank.
 *
 * Boot only indexes: "<dir>/.smpidx" caches format, length, BPM and peak per WAV,
 * keyed on name + size + timestamp, so only new or changed files are parsed.
 * Transients (fw/transients.c) are found when a sample is first loaded and cached in
 * "<dir>/.smptr" (same keys), for the transient note -> position mode.
 */
#ifndef SAMPLES_H
#define SAMPLES_H

#include <stdint.h>

#define MAX_LIBS          4
#define MAX_PER_LIB       1024
#define MAX_BANKS_PER_LIB 160
#define BANK_MAX_FILES    127
#define BANK_MAX_BYTES    (7u << 20)   /* one bank per pool region: 4 x 7.5 MB in the 30 MB pool */
#define SAMPLE_NAME_LEN   80
#define NOT_RESIDENT      0xFFFFFFFFu

struct sample {
    char     name[SAMPLE_NAME_LEN];    /* relative to the library folder ("sub/x.wav") */
    uint32_t fsize;
    uint16_t fdate, ftime;
    uint32_t data_off, data_len, sr;   /* of this sample (a part: its stretch of the file) */
    uint8_t  ch, bits, ready, fmt;     /* fmt: WAV_PCM / WAV_FLOAT                        */
    uint32_t frames;
    uint32_t file_len;                 /* data bytes of the whole file                    */
    uint16_t seg, nseg;                /* part seg of nseg (nseg 1: the whole file)       */
    int32_t  beats_log2;               /* the loop is 2^beats_log2 beats long         */
    uint32_t bpm_q16;
    int16_t  peak;                     /* 0 = not measured yet                            */
    uint8_t  ntr;                      /* transients (TR_NONE: not analysed yet)          */
    uint32_t tr;                       /* first transient in the pool                     */
    uint32_t offset;                   /* words from the bank base                        */
};
#define TR_NONE 0xFF

enum { BANK_NONE = 0, BANK_LOADING, BANK_RESIDENT };

struct bank {
    int      first, count;             /* samples [first, first+count) of the library    */
    uint32_t words;                    /* SDRAM footprint                                 */
    uint32_t base;                     /* engine word address or NOT_RESIDENT             */
    uint8_t  state, builtin;
    int16_t  want;                     /* index to load first                             */
    uint32_t lru, hold_until;          /* hold: old grains may still read it              */
    uint32_t retry_at;                 /* no room in the pool: requests ignored until then */
    uint8_t  blocked;
    char     name[24];
};

struct lib {
    char dir[12];
    int count, nbanks;
    struct sample* s;
    struct bank* b;
};

extern struct lib g_libs[MAX_LIBS];
extern int g_normalize;
extern int g_sd_verbose;

int samples_init(void);                /* mount + index; returns WAVs indexed            */
int samples_load_builtin(void);        /* no card: 2 synthetic loops, lib 0 bank 0       */
void samples_list(int lib, int bank);
void samples_stats(void);
void samples_sdbench(void);

static inline struct bank* samples_bank(int lib, int bank) {
    if (lib < 0 || lib >= MAX_LIBS || bank < 0 || bank >= g_libs[lib].nbanks)
        return 0;
    return &g_libs[lib].b[bank];
}
static inline struct sample* samples_get(int lib, int bank, int idx) {
    struct bank* b = samples_bank(lib, bank);
    if (!b || idx < 0 || idx >= b->count)
        return 0;
    return &g_libs[lib].s[b->first + idx];
}
/* Transient positions (samples, ascending) of an analysed sample; -1 if not analysed. */
int samples_transients(const struct sample* s, const uint32_t** pos);

/* Engine word address of a ready sample, NOT_RESIDENT otherwise. */
uint32_t samples_base(int lib, int bank, int idx);

/* Make (lib, bank) resident, loading sample idx first. prio 1: a slot waits, 0: prefetch. */
void samples_want(int lib, int bank, int idx, int prio);
/* Keep a bank resident for ms (grains of a replaced buffer still fade out). */
void samples_hold(int lib, int bank, uint32_t ms);
/* Incremental loader: call from the main loop. Returns 1 when a sample became ready. */
int samples_poll(void);

/* Called during long loader chunks so UART input never overflows (set by main). */
extern void (*samples_idle_hook)(void);
/* Called inside SD sector reads/writes (directory searches): must only buffer input. */
extern void (*samples_drain_hook)(void);

/* Provided by the slot layer: 2 = a slot plays or waits for the bank (never evicted),
   1 = prefetch target (next bank), 0 = not needed (evictable, requests dropped). */
int samples_bank_keep(int lib, int bank);

/* Small files on the card (config): bytes read (buf NUL-terminated) / 0 written, -1 on error. */
int samples_read_file(const char* path, char* buf, int n);
int samples_write_file(const char* path, const char* buf, int n);

/* Tempo from the length: bpm = 60/duration folded into [100, 200]. */
uint32_t samples_track_bpm_q16(uint32_t frames, uint32_t sr, int32_t* beats_log2);

#endif
