// Measurements for the synth model: how much of a steady note is not on its harmonic grid
// (aliasing and noise), and how far a render is from a reference.
#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

#include "wt_tables.h"

namespace synth {

struct Inharm {
    double harmDb = -200;   // power on the grid (dBFS, sine = 0 dB)
    double offDb = -200;    // power off the grid, 20 Hz .. 20 kHz
    double ratioDb = -200;  // off - harm: the aliasing + noise level relative to the note
    double worstDb = -200;  // the strongest single off-grid bin relative to the note's total
    double worstHz = 0;
};

// x: a steady stretch (>= n samples from 'start'), grid = multiples of f0 (a note whose oscs
// all run at integer multiples of f0). Blackman-Harris window, n a power of two.
inline Inharm inharmonic(const std::vector<float>& x, size_t start, size_t n, double fs, double f0) {
    std::vector<std::complex<double>> X(n);
    double wsum = 0;
    for (size_t i = 0; i < n; ++i) {
        const double t = 2 * M_PI * double(i) / double(n);
        const double w = 0.35875 - 0.48829 * std::cos(t) + 0.14128 * std::cos(2 * t) - 0.01168 * std::cos(3 * t);
        X[i] = w * double(x[start + i]);
        wsum += w;
    }
    wt::fft(X, false);
    const double binHz = fs / double(n), norm = 2.0 / (wsum * wsum);  // a sine of amplitude 1 -> 1 (0 dB peak)
    const double guard = 4.5 * binHz;                                   // the window's main lobe: +-4 bins
    double harm = 0, off = 0, worst = 0, worstHz = 0;
    for (size_t k = 1; k < n / 2; ++k) {
        const double hz = double(k) * binHz, p = std::norm(X[k]) * norm;
        const double h = std::round(hz / f0), d = std::fabs(hz - h * f0);
        if (h >= 1 && d <= guard)
            harm += p;
        else if (hz >= 20 && hz <= 20000) {
            off += p;
            if (p > worst)
                worst = p, worstHz = hz;
        }
    }
    auto db = [](double p) { return p > 0 ? 10 * std::log10(p) : -200.0; };
    // the sum over bins counts a sine's power ~1.97x (window ENBW 2.0 bins); harmonic and off-grid
    // sums are scaled the same, so the ratio is right; the absolute dB are relative
    return {db(harm), db(off), db(off) - db(harm), db(worst) - db(harm), worstHz};
}

// SNR of x against ref (dB) over [start, start + n), best integer lag within +-maxLag
inline double snr(const std::vector<float>& x, const std::vector<float>& ref, size_t start, size_t n, int maxLag,
                  int* lagOut = nullptr) {
    double best = -1e30;
    int bestLag = 0;
    for (int lag = -maxLag; lag <= maxLag; ++lag) {
        double c = 0;
        for (size_t i = start; i < start + n; ++i)
            if (i + lag < x.size())
                c += double(x[i + lag]) * ref[i];
        if (c > best)
            best = c, bestLag = lag;
    }
    double s = 0, e = 0;
    for (size_t i = start; i < start + n; ++i) {
        const double r = ref[i], v = (i + bestLag < x.size()) ? x[i + bestLag] : 0.0;
        s += r * r;
        e += (v - r) * (v - r);
    }
    if (lagOut)
        *lagOut = bestLag;
    return e > 0 ? 10 * std::log10(s / e) : 200.0;
}

// SNR of x against ref (dB) after aligning x to ref with a fractional delay: integer lag from the
// cross-correlation peak, refined by a parabola through its neighbours, applied as a linear
// phase in the frequency domain. lagOut = the delay found (samples, x later than ref > 0).
inline double snr_frac(const std::vector<float>& x, const std::vector<float>& ref, double* lagOut = nullptr,
                       double bandHz = 18000, double fs = 48000) {
    const size_t len = std::min(x.size(), ref.size());
    size_t n = 1;
    while (n < 2 * len)
        n <<= 1;
    std::vector<std::complex<double>> X(n), R(n);
    for (size_t i = 0; i < len; ++i)
        X[i] = x[i], R[i] = ref[i];
    wt::fft(X, false);
    wt::fft(R, false);
    for (size_t k = 0; k < n; ++k) {  // compare the audio band only (0 .. bandHz)
        const double hz = (k <= n / 2 ? double(k) : double(n - k)) * fs / double(n);
        if (hz > bandHz)
            X[k] = R[k] = 0;
    }
    std::vector<std::complex<double>> C(n);
    for (size_t k = 0; k < n; ++k)
        C[k] = X[k] * std::conj(R[k]);
    wt::fft(C, true);
    auto cc = [&](long l) { return C[size_t((l + long(n)) % long(n))].real(); };
    long best = 0;
    for (long l = -256; l <= 256; ++l)
        if (cc(l) > cc(best))
            best = l;
    // refine: the error energy as a function of the fractional shift (Parseval), golden section
    auto err = [&](double l) {
        double e = 0;
        for (size_t k = 0; k < n; ++k) {
            const double f = (k <= n / 2 ? double(k) : double(k) - double(n)) / double(n);
            e += std::norm(X[k] * std::polar(1.0, 2 * M_PI * f * l) - R[k]);
        }
        return e;
    };
    double a = best - 1.0, b = best + 1.0;
    const double gr = 0.6180339887;
    double c = b - gr * (b - a), d = a + gr * (b - a), fc = err(c), fd = err(d);
    for (int it = 0; it < 30; ++it) {
        if (fc < fd)
            b = d, d = c, fd = fc, c = b - gr * (b - a), fc = err(c);
        else
            a = c, c = d, fc = fd, d = a + gr * (b - a), fd = err(d);
    }
    const double lag = 0.5 * (a + b);
    // shift x back by lag
    for (size_t k = 0; k < n; ++k) {
        const double f = (k <= n / 2 ? double(k) : double(k) - double(n)) / double(n);
        X[k] *= std::polar(1.0, 2 * M_PI * f * lag);
    }
    wt::fft(X, true);
    wt::fft(R, true);
    double s = 0, e = 0;
    const size_t edge = 512;  // skip the ends (the shift wraps around)
    for (size_t i = edge; i + edge < len; ++i) {
        const double v = X[i].real() / double(n), r = R[i].real() / double(n);
        s += r * r;
        e += (v - r) * (v - r);
    }
    if (lagOut)
        *lagOut = lag;
    return e > 0 ? 10 * std::log10(s / e) : 200.0;
}

}  // namespace synth
