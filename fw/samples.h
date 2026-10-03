/* Sample library: WAV files on the SD card, held in SDRAM per bank.
 *
 * Libraries are the top folders (/drums, /melody, /bass, /kick). A library is split
 * into banks: every subfolder is a bank, loose files are packed in name order into
 * banks of at most BANK_MAX_BYTES / 127 files. A bank is the unit of SDRAM residency:
 * sample changes inside the playing bank are instant, a bank change loads in the
 * background (requested sample first) while the old bank keeps playing.
 *
 * Boot only indexes: "<dir>/.smpidx" caches format, length, BPM and peak per WAV,
 * keyed on name + size + timestamp, so only new or changed files are parsed.
 */
#ifndef SAMPLES_H
#define SAMPLES_H

#include <stdint.h>

#define MAX_LIBS          4
#define MAX_PER_LIB       1024
#define MAX_BANKS_PER_LIB 160
#define BANK_MAX_FILES    127
#define BANK_MAX_BYTES    (7u << 20)   /* 2 slots x (playing + loading) banks fit the 30 MB pool */
#define SAMPLE_NAME_LEN   80
#define NOT_RESIDENT      0xFFFFFFFFu

struct sample {
    char     name[SAMPLE_NAME_LEN];    /* relative to the library folder ("sub/x.wav") */
    uint32_t fsize;
    uint16_t fdate, ftime;
    uint32_t data_off, data_len, sr;
    uint8_t  ch, bits, ready, pad;
    uint32_t frames;
    int32_t  beats_log2;               /* the loop is 2^beats_log2 beats long         */
    uint32_t bpm_q16;
    int16_t  peak;                     /* 0 = not measured yet                            */
    uint32_t offset;                   /* words from the bank base                        */
};

enum { BANK_NONE = 0, BANK_LOADING, BANK_RESIDENT };

struct bank {
    int      first, count;             /* samples [first, first+count) of the library    */
    uint32_t words;                    /* SDRAM footprint                                 */
    uint32_t base;                     /* engine word address or NOT_RESIDENT             */
    uint8_t  state, builtin;
    int16_t  want;                     /* index to load first                             */
    uint32_t lru, hold_until;          /* hold: old grains may still read it              */
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

/* Tempo from the length: bpm = 60/duration folded into [100, 200]. */
uint32_t samples_track_bpm_q16(uint32_t frames, uint32_t sr, int32_t* beats_log2);

#endif
