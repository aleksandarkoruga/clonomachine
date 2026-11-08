/* Wavetables for the synth: 2048-sample frames -> the band-limited mip levels the oscillator unit
 * reads (rtl/synth_osc.v): level k = 1..9 keeps harmonics 1 .. (1024 >> k) - 1 of every frame and
 * stores each frame at max(2048 >> k, 256) samples, s16; all frames of a level follow each other,
 * levels follow each other (model/wt_tables.h, without level 0). Integer FFTs (real input through
 * a half-size complex transform), no hardware here: fw/synth.c puts the levels in SDRAM,
 * sim/tb_synth_fw.cpp compares them with the model's. */
#ifndef SYNTH_WT_H
#define SYNTH_WT_H

#include <stdint.h>

#include "synth_ctl.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WT_N0 2048
#define WT_FRAME_WORDS 3328u /* 1024 + 512 + 7 x 256 */

void wt_init(void);                                  /* twiddles (once) */
void wt_set_work(int32_t* buf);                      /* 8 KB work array (on-chip SRAM: the transforms
                                                        run from memory the caches can't hold) */
static inline uint32_t wt_words(int frames) { return (uint32_t)frames * WT_FRAME_WORDS; }
int wt_level_size(int k);                            /* samples per frame at level k */
/* level k's frame 0 at lvl[k] (SDRAM word addresses) for a table at word 'base' */
void wt_layout(uint32_t base, int frames, uint32_t lvl[SY_LEVELS]);
/* one frame (2048 samples, peak at most full scale) -> its levels: dst[k] = level k's samples of
   this frame (wt_level_size(k) each). Scaled by 0.78: band-limiting overshoots (a square's
   fundamental alone is 4/pi of its peak). */
void wt_build_frame(const int16_t* x, int16_t* const dst[SY_LEVELS]);
int32_t wt_cos_q30(int i);                           /* cos(2 pi i / 2048), Q30 (for built-in shapes) */
/* the built-in tables: count, name, frames, and frame f's 2048 samples */
int wt_builtin_count(void);
const char* wt_builtin_name(int t);
int wt_builtin_frames(int t);
void wt_builtin_frame(int t, int f, int16_t* x);

#ifdef __cplusplus
}
#endif

#endif
