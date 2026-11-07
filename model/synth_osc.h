// Wavetable reads for the synth model: mip-level choice, interpolation within a frame and the
// crossfade between the two frames around the table position.
//
// Interpolators (all read a window of consecutive samples of one frame):
//   LIN    2 points
//   HERM   4-point, 3rd-order Hermite (Catmull-Rom)
//   SINC8  8-point Kaiser-windowed sinc, 256 phases, coefficients interpolated linearly between
//          phases (the hardware would hold the 257 x 8 coefficients in one block RAM)
// The level is the largest one whose top harmonic, at the oscillator's highest instantaneous
// frequency, stays below 'limit' (Hz): at an oscillator rate R x 48 kHz, harmonics above 24 kHz
// fold back above 48R - 24k, where the decimation filter removes them.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

#include "wt_tables.h"

namespace synth {

enum class Interp { LIN, HERM, SINC8 };

struct SincTable {
    static constexpr int T = 8, P = 256;
    std::vector<double> c;  // [P + 1][T]
    explicit SincTable(double fc = 0.92, double beta = 7.0) {
        c.resize((P + 1) * T);
        auto I0 = [](double x) {
            double s = 1, t = 1;
            for (int k = 1; k < 30; ++k)
                t *= x * x / (4.0 * k * k), s += t;
            return s;
        };
        for (int p = 0; p <= P; ++p) {
            double sum = 0;
            for (int j = 0; j < T; ++j) {
                const double x = (j - (T / 2 - 1)) - double(p) / P;
                const double sx = x == 0 ? fc : std::sin(M_PI * fc * x) / (M_PI * x);
                const double r = x / (T / 2.0);
                const double w = std::fabs(r) >= 1 ? 0 : I0(beta * std::sqrt(1 - r * r)) / I0(beta);
                c[p * T + j] = sx * w;
                sum += sx * w;
            }
            for (int j = 0; j < T; ++j)
                c[p * T + j] /= sum;  // unity gain at DC for every phase
        }
    }
};

inline const SincTable& sinc_table() {
    static const SincTable s;
    return s;
}

// one frame read at phase ph (0..1), float
inline double read_frame(const wt::Table& t, int k, int f, double ph, Interp in) {
    const int N = t.size(k);
    const double p = ph * N;
    const int i0 = int(std::floor(p));
    const double fr = p - i0;
    switch (in) {
    case Interp::LIN: {
        const double a = t.at(k, f, i0), b = t.at(k, f, i0 + 1);
        return a + fr * (b - a);
    }
    case Interp::HERM: {
        const double a = t.at(k, f, i0 - 1), b = t.at(k, f, i0), c = t.at(k, f, i0 + 1), d = t.at(k, f, i0 + 2);
        const double c1 = 0.5 * (c - a), c2 = a - 2.5 * b + 2 * c - 0.5 * d, c3 = 0.5 * (d - a) + 1.5 * (b - c);
        return ((c3 * fr + c2) * fr + c1) * fr + b;
    }
    case Interp::SINC8: {
        const auto& S = sinc_table();
        const double pp = fr * S.P;
        const int pi = std::min(int(pp), S.P - 1);
        const double pf = pp - pi;
        double v = 0;
        for (int j = 0; j < S.T; ++j)
            v += (S.c[pi * S.T + j] * (1 - pf) + S.c[(pi + 1) * S.T + j] * pf) * t.at(k, f, i0 - (S.T / 2 - 1) + j);
        return v;
    }
    }
    return 0;
}

// the table at position pos (0..1 over the frames): the two frames around it, crossfaded
inline double read_table(const wt::Table& t, int k, double pos, double ph, Interp in) {
    const double r = std::clamp(pos, 0.0, 1.0) * (t.frames - 1);
    const int f0 = std::min(int(r), t.frames - 1), f1 = std::min(f0 + 1, t.frames - 1);
    const double u = r - f0;
    const double a = read_frame(t, k, f0, ph, in);
    return u > 0 ? a + u * (read_frame(t, k, f1, ph, in) - a) : a;
}

}  // namespace synth
