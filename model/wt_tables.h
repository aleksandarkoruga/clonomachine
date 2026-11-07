// Wavetables for the synth model: wavetable WAVs (frames of 2048 samples, 'clm ' chunk) or
// built-in shapes, turned into band-limited mip levels the way a PC-side prepare step would
// (FFT per frame, level k keeps the harmonics that fit in n0 >> k samples).
//
// Level k of a table keeps harmonics 0 .. H_k = (n0 >> k)/2 - 1 of every frame (the Nyquist bin
// is dropped) and stores each frame at N_k = max(n0 >> k, nmin) samples: the small levels are
// kept oversampled, so that an interpolating read of a high note stays clean (its images land
// far above the audio band). Frames of one level are contiguous, levels follow each other: the
// layout the tables would have in SDRAM (one 16-bit word per sample).
#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <string>
#include <vector>

namespace wt {

void fft(std::vector<std::complex<double>>& a, bool inverse);  // radix 2, in place, unscaled

struct Table {
    std::string name;
    int frames = 0;
    int n0 = 2048;                         // level-0 frame size
    int levels = 0;                        // n0 >> (levels - 1) >= kMinN
    int nmin = 4;                          // the smallest stored frame size
    std::vector<std::vector<float>> lvl;   // [level][frame * N_k + i], full scale +-1
    std::vector<std::vector<int16_t>> q;   // the same in s16, scaled by qscale (no clipping)
    double qscale = 1.0;                   // q = round(lvl * 32767 * qscale)
    std::vector<uint32_t> base;            // word offset of each level (SDRAM layout)
    uint32_t words = 0;                    // total size in 16-bit words

    int size(int k) const { return std::max(n0 >> k, nmin); }
    int harm(int k) const { return (n0 >> k) / 2 - 1; }  // the highest harmonic of level k
    // frame f (0..frames-1), level k, sample i (wrapped)
    float at(int k, int f, int i) const { const int N = size(k); return lvl[k][size_t(f) * N + (i & (N - 1))]; }
    int16_t atq(int k, int f, int i) const { const int N = size(k); return q[k][size_t(f) * N + (i & (N - 1))]; }
};

constexpr int kMinN = 4;  // the smallest level: one harmonic (a sine)

// frames: frames * n0 samples (n0 a power of two). Builds the levels and the s16 copy.
Table build(const std::string& name, const std::vector<float>& frames, int n0 = 2048, int nmin = 4);
// a wavetable WAV (frame size from the 'clm ' chunk, else 2048); false if unreadable
bool load_clm_wav(const std::string& path, Table& t, int nmin = 4);
bool save_clm_wav(const std::string& path, const std::vector<float>& frames, int n0 = 2048);
// built-in: "basic" (sine -> triangle -> saw -> square over 256 frames), "saw", "sine",
// "pwm" (pulse width 50 % -> 2 %), "formant" (a saw through a resonance sweeping 1..32 x f)
Table builtin(const std::string& name, int frames = 256, int n0 = 2048, int nmin = 4);
// the table a name stands for: a built-in name or a path to a WAV
bool get(const std::string& spec, Table& t, int nmin = 4);
// the level whose top harmonic stays at or below 'limit' Hz at fundamental f (0 if all do)
int level_for(const Table& t, double f, double limit);

}  // namespace wt
