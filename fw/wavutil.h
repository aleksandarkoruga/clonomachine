/* WAV helpers without hardware dependencies (host-testable, host/test_wavutil.cpp):
 *
 *  - conversion of any accepted sample format to the SDRAM format (mono int16): PCM
 *    8/16/24/32-bit and IEEE float 32/64-bit, 1..8 channels (averaged), with a gain;
 *  - the split plan for long files: a file longer than WAV_SPLIT_SECONDS (or larger than
 *    a bank) becomes a run of parts that are samples of their own. A part is
 *    WAV_PART_BEATS beats at the tempo in the file name ("..._128bpm", "bpm 90") or at
 *    WAV_PART_BPM, so every part reads as a 2^k-beat loop at that tempo.
 */
#ifndef WAVUTIL_H
#define WAVUTIL_H

#include <stdint.h>

#define WAV_PCM   1
#define WAV_FLOAT 3
#define WAV_MAX_CH 8

#define WAV_SPLIT_SECONDS 60u   /* longer files are split                          */
#define WAV_PART_BEATS    16u   /* part length: 4 bars ...                         */
#define WAV_PART_BPM      120u  /* ... at this tempo when the name has none (8 s)  */

/* Format check: 1 if (fmt, ch, bits) can be converted. */
int wav_supported(int fmt, int ch, int bits);

/* n frames -> mono int16. gain_q16: 65536 = unity. Floats are taken at half scale
   (-6 dB, so +-2.0 does not clip); the peak/normalisation pass restores the level. */
void wav_convert(int16_t* dst, const uint8_t* src, uint32_t n, int fmt, int ch, int bits, int32_t gain_q16);

/* Tempo written in the name as "<n>bpm" / "<n> bpm" / "<n>_bpm" / "bpm<n>", 40..300;
   0 if none. In milli-BPM. */
uint32_t wav_name_bpm_milli(const char* name);

/* Frames per part, 0 when the file stays whole. max_frames: largest sample a bank holds.
   Parts are multiples of 64 frames. */
uint32_t wav_split_len(uint32_t frames, uint32_t sr, const char* name, uint32_t max_frames);
/* Number of parts: back to back; a remainder of at least a quarter part becomes a last
   part that ends at the end of the file (overlapping the one before), a shorter one is
   dropped, so all parts have the same length (and tempo). */
uint32_t wav_split_count(uint32_t frames, uint32_t len);
/* First frame of part k (a multiple of 4: keeps SD reads word aligned). */
uint32_t wav_split_start(uint32_t frames, uint32_t len, uint32_t k);

#endif
