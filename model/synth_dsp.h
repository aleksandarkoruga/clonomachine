// Small DSP blocks for the synth model: the 2x -> 1x decimation filter and helpers.
#pragma once
#include <cmath>
#include <cstdint>
#include <vector>

namespace synth {

// Halfband lowpass for 96 kHz -> 48 kHz: passband to 20 kHz, stopband from 28 kHz (everything
// that would fold into 20 .. 28 kHz lands above the audio band). Kaiser-windowed, odd length,
// every other tap zero except the centre: ~(len + 1) / 4 multiplies per output sample (the taps
// are symmetric). Coefficients are quantised to Q17 (an 18-bit multiplier operand).
struct Halfband {
    std::vector<double> h;   // full impulse response
    std::vector<int32_t> q;  // Q17
    explicit Halfband(int len = 71, double beta = 8.6) {
        h.resize(len);
        const int M = len - 1;
        auto I0 = [](double x) {
            double s = 1, t = 1;
            for (int k = 1; k < 40; ++k)
                t *= x * x / (4.0 * k * k), s += t;
            return s;
        };
        double sum = 0;
        for (int i = 0; i < len; ++i) {
            const double x = i - M / 2.0;
            const double s = x == 0 ? 0.5 : std::sin(M_PI * 0.5 * x) / (M_PI * x);
            const double r = 2.0 * i / M - 1;
            h[i] = s * I0(beta * std::sqrt(std::max(0.0, 1 - r * r))) / I0(beta);
            sum += h[i];
        }
        q.resize(len);
        for (int i = 0; i < len; ++i) {
            h[i] /= sum;
            q[i] = int32_t(std::lrint(h[i] * 131072.0));
        }
    }
    int nonzero_taps() const {
        int n = 0;
        for (int32_t c : q)
            n += c != 0;
        return n;
    }
};

// Decimates by 2 (float): push two input samples, get one output.
struct Decim2 {
    const Halfband& hb;
    std::vector<double> buf;
    int w = 0;
    explicit Decim2(const Halfband& b) : hb(b), buf(b.h.size() * 2, 0.0) {}
    double push2(double a, double b) {
        const int L = int(hb.h.size());
        auto put = [&](double v) {
            buf[w] = buf[w + L] = v;  // mirrored ring: a contiguous window always
            w = (w + 1) % L;
        };
        put(a);
        put(b);
        double s = 0;
        for (int i = 0; i < L; ++i)
            s += hb.h[i] * buf[w + i];
        return s;
    }
};

}  // namespace synth
