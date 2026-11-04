// Loudness matching for the sample card (PC only, host/prepare_sd.cpp).
//
// lufs(): ITU-R BS.1770-4 integrated loudness of a mono signal: K-weighting, 400 ms blocks
// with 75 % overlap, the absolute (-70 LUFS) and the relative (-10 LU) gate. A signal shorter
// than a block is measured as one block. -INFINITY for silence.
//
// level(): raises a quiet file (peak-normalised, -1..1) to a target loudness. Files at or
// above the target are left alone (the board brings any file below full scale back up at
// load, and turning loud files down only costs level). The room comes from a 3-band
// compressor (zero-phase complementary split, so the bands add up to the input exactly)
// and a look-ahead peak limiter at full scale. The compressor's threshold sits the same
// number of dB below each band's own peak and is found per file: the least compression
// that lets the limiter reach the target within lim_max of gain reduction. A file that
// can't get there within lim_max even with the deepest compression, or would need more
// than max_boost, is raised as far as it can (capped): by the limiter alone, unless the
// compression that helped most gets it at least 1 dB further.
#ifndef LOUDNESS_H
#define LOUDNESS_H

#include <cstddef>
#include <cstdint>
#include <vector>

double lufs(const float* x, size_t n, uint32_t sr);

struct LevelParams {
    double target = -12;        // LUFS
    double max_boost = 12;      // dB
    double ratio = 4, knee = 6; // compressor, knee width in dB
    double attack_ms = 12, release_ms = 60, low_release_ms = 120;
    double xover_lo = 120, xover_hi = 2500; // Hz
    double lim_max = 4;         // dB: the most the limiter may take off a peak
    double lim_lookahead_ms = 5, lim_release_ms = 60;
};

struct LevelResult {
    double before = 0, after = 0; // LUFS
    double boost = 0;             // dB the file came up
    double depth = 0;             // dB below each band's peak where compression starts (0: none)
    double comp = 0;              // dB: largest compressor gain reduction (any band)
    double limit = 0;             // dB: largest limiter gain reduction
    bool changed = false, capped = false;
};

LevelResult level(std::vector<float>& x, uint32_t sr, const LevelParams& p);

#endif
