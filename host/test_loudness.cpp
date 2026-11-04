// Checks host/loudness.cpp: BS.1770 calibration (a full-scale 997 Hz sine in one channel is
// -3.01 LUFS, at any sample rate; the gates ignore silence) and level() on synthetic drums and
// pads (lands on the target, full-scale peak, untouched when loud already).
//   g++ -O2 -std=c++17 host/test_loudness.cpp host/loudness.cpp -o build/host/test_loudness
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "loudness.h"

static int fails;
static void check(const char* what, double got, double want, double tol) {
    const bool ok = std::fabs(got - want) <= tol;
    fails += !ok;
    std::printf("%-44s %8.2f  (want %.2f +- %.2f)%s\n", what, got, want, tol, ok ? "" : "  FAIL");
}

static std::vector<float> sine(uint32_t sr, double secs, double amp) {
    std::vector<float> x(size_t(sr * secs));
    for (size_t i = 0; i < x.size(); ++i)
        x[i] = float(amp * std::sin(2 * M_PI * 997.0 * double(i) / sr));
    return x;
}

static double peak(const std::vector<float>& x) {
    double p = 0;
    for (float v : x)
        p = std::max(p, double(std::fabs(v)));
    return p;
}

static void normalise(std::vector<float>& x) {
    const double p = peak(x);
    for (float& v : x)
        v = float(v / p);
}

int main() {
    for (uint32_t sr : {44100u, 48000u}) {
        auto x = sine(sr, 10, 1.0);
        check(sr == 48000 ? "sine 0 dBFS, 48 kHz" : "sine 0 dBFS, 44.1 kHz", lufs(x.data(), x.size(), sr), -3.01, 0.05);
    }
    {
        auto x = sine(48000, 10, 0.1);
        check("sine -20 dBFS", lufs(x.data(), x.size(), 48000), -23.01, 0.05);
        x.resize(x.size() * 3); // 20 s of silence after it: gated out
        // the blocks across the end hold part of the sine and pass the gates: a little lower
        check("sine -20 dBFS + 20 s silence", lufs(x.data(), x.size(), 48000), -23.08, 0.02);
    }
    std::mt19937 rng(1);
    std::normal_distribution<double> nd;
    const uint32_t sr = 48000;
    LevelParams p;
    { // drums: a kick (decaying 55 Hz) and hats (noise bursts) on a 16th grid, 4 bars at 120 bpm
        std::vector<float> x(sr * 8);
        const size_t s16 = sr / 8;
        for (size_t k = 0; k < x.size() / s16; ++k)
            for (size_t i = 0; i < s16 && k * s16 + i < x.size(); ++i) {
                const double t = double(i) / sr;
                double v = 0.2 * nd(rng) * std::exp(-t * 60);
                if (k % 4 == 0)
                    v += std::sin(2 * M_PI * 55 * t) * std::exp(-t * 12);
                x[k * s16 + i] = float(v * 0.3);
            }
        normalise(x);
        LevelResult r = level(x, sr, p);
        std::printf("drums  %.1f -> %.1f LUFS  +%.1f dB  depth %.0f  comp %.1f  limit %.1f%s\n", r.before, r.after, r.boost,
                    r.depth, r.comp, r.limit, r.capped ? "  capped" : "");
        check("drums: peak after", peak(x), 1.0, 1e-6);
        check("drums: limiter within lim_max", std::min(r.limit, p.lim_max), r.limit, 1e-9);
        if (!r.capped)
            check("drums: on target", r.after, p.target, 0.2);
        LevelResult again = level(x, sr, p);
        check("drums: a second pass leaves it", again.changed ? 1 : 0, r.capped ? 1 : 0, 0);
    }
    { // pad: a chord that swells and fades, with a hiss floor, quiet
        std::vector<float> x(sr * 12);
        for (size_t i = 0; i < x.size(); ++i) {
            const double t = double(i) / sr, env = 0.15 + 0.85 * std::pow(std::sin(M_PI * t / 6), 2);
            double v = 0;
            for (double f : {220.0, 277.2, 329.6, 440.0})
                v += std::sin(2 * M_PI * f * t);
            x[i] = float(env * v / 4 + 0.002 * nd(rng));
        }
        normalise(x);
        std::vector<float> loud = x;
        LevelParams pm = p;
        pm.target = -10;
        LevelResult r = level(x, sr, pm);
        std::printf("pad    %.1f -> %.1f LUFS  +%.1f dB  depth %.0f  comp %.1f  limit %.1f%s\n", r.before, r.after, r.boost,
                    r.depth, r.comp, r.limit, r.capped ? "  capped" : "");
        check("pad: peak after", peak(x), 1.0, 1e-6);
        if (!r.capped)
            check("pad: on target", r.after, pm.target, 0.2);
        pm.target = r.before - 1; // already louder than the target: untouched
        LevelResult u = level(loud, sr, pm);
        check("loud file untouched", u.changed ? 1 : 0, 0, 0);
    }
    std::printf(fails ? "%d FAILED\n" : "all passed\n", fails);
    return fails != 0;
}
