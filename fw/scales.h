/* Scales and tunings for the synth: MIDI keys or the harmony generator's degrees -> pitch.
 * Degree d of a scale (0 = the root, negative below): step d mod n of the scale plus (d div n)
 * repetitions (octaves, or the tuning's own interval). The steps are the selected tuning's when
 * it has as many as the scale counts in (12, 24, 43), else an equal division into that many; the
 * chromatic scales play every step of the selected tuning. Pitch in semitones Q16 (69 = A 440
 * Hz), so the rest of the synth works in one unit whatever the tuning. Host-testable. */
#ifndef SCALES_H
#define SCALES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int scales_count(void);
int tunings_count(void);
const char* scale_name(int s);
const char* tuning_name(int t);
int scale_size(int s);  /* degrees per repetition */
int tuning_size(int t); /* steps per repetition */

void scales_init(void); /* the tunings' step tables (soft-float log2, once) */
/* pitch of degree d (relative to the root at MIDI 60 + root), semitones Q16 */
int32_t scale_pitch_q16(int scale, int tuning, int root, int degree);

#ifdef __cplusplus
}
#endif

#endif
