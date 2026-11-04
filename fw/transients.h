/* Transient (onset) detection for note -> position mapping. Plain C, no hardware
 * dependencies: runs on the PC (host/prepare_sd.cpp, host/test_transients.cpp) and on the
 * SoC at boot for files the cache on the SD card (<folder>/.smptr) doesn't know yet.
 *
 * Detection: the sum of |x[i] - x[i-1]| (high-frequency energy, emphasises attacks) per
 * 64-sample block, Mitchell log2; an onset is a rise of TR_RISE over the minimum of the
 * previous three blocks (~4 ms), at most one per TR_REFRACT blocks (the strongest), and
 * within TR_RANGE of the loudest block. Each onset is placed just before the attack: the
 * first sample reaching a quarter of the block peak, moved back to a zero crossing. The cached
 * transients keep at most one onset per 1/TR_BINS of the sample, the strongest (the start,
 * when it counts, keeps its bin): dense files keep positions spread over the whole file, each
 * on a clear accent, instead of clusters that sound alike.
 */
#ifndef TRANSIENTS_H
#define TRANSIENTS_H

#include <stdint.h>

#define TR_MAX      64          /* transients per sample                               */
#define TR_BINS     64          /* at most one per 1/TR_BINS of the sample (<= TR_MAX) */
#define TR_BLOCK    64          /* analysis block (samples)                            */
#define TR_RISE     384         /* log2 Q8: 1.5 octaves (~9 dB) rise                   */
#define TR_REFRACT  24          /* blocks (~35 ms at 44.1 kHz) between onsets          */
#define TR_RANGE    (6 * 256)   /* onsets within ~36 dB of the loudest block           */

/* Reads sample i (0 <= i < frames). */
typedef int16_t (*tr_get_fn)(void* ctx, uint32_t i);

/* Finds up to TR_MAX onsets of a mono 16-bit sample; positions in samples, ascending.
   bins: TR_BINS for the cached transients (the strongest per 1/bins of the sample), 0 for
   the TR_MAX strongest anywhere (the PC tool's loop-tempo fit). idle (may be 0) is called
   every few thousand samples. Returns the count. */
int tr_analyse(tr_get_fn get, void* ctx, uint32_t frames, uint32_t bins, uint32_t* pos, void (*idle)(void));

#endif
