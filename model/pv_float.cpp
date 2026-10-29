// SPDX-License-Identifier: MPL-2.0
// Derived from Bungee (https://github.com/bungee-audio-stretch/bungee), Copyright (C) 2020-2026
// Parabola Research Limited, and therefore under the Mozilla Public License 2.0
// (https://mozilla.org/MPL/2.0/); the rest of this repository is MIT (LICENSE).
//
#include "pv_float.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>

namespace pvf {

double Source::at(double pos) const {
    if (frames <= 0)
        return 0;
    double fl = std::floor(pos);
    const double fr = pos - fl;
    int64_t i = int64_t(fl);
    auto get = [&](int64_t k) -> double {
        if (loop) {
            k %= frames;
            if (k < 0)
                k += frames;
            return x[k];
        }
        return (k >= 0 && k < frames) ? x[k] : 0.0;
    };
    const double a = get(i);
    return fr == 0 ? a : a + (get(i + 1) - a) * fr;
}

Pv::Pv(int log2Hop): log2H(log2Hop), H(1 << log2Hop), N(8 << log2Hop), n2(4 << log2Hop) {
    const double pi = std::numbers::pi;
    const double gain = (3 * pi) / (3 * pi + 8);
    wa.resize(N);
    for (int n = 0; n < N; ++n) // zero phase: index 0 = centre
        wa[n] = gain / N * (1 + std::cos(2 * pi * n / N));
    ws.resize(4 * H);
    for (int n = 0; n < 4 * H; ++n)
        ws[n] = 0.25 * (1 + std::cos(2 * pi * n / (4 * H)));
    lapped.assign(4 * H, 0);
    pPhase.assign(n2 + 1, 0);
    pRot.assign(n2 + 1, 0);
    pEnergy.assign(n2 + 3, 0);
    re.resize(N);
    im.resize(N);
    phase.resize(n2 + 1);
    rot.resize(n2 + 1);
    energy.resize(n2 + 3);
    partials.reserve(N);
}

void Pv::reset() {
    pValid = false;
    std::fill(lapped.begin(), lapped.end(), 0);
}

void Pv::fft(bool inverse) {
    for (int i = 1, j = 0; i < N; ++i) {
        int bit = N >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            std::swap(re[i], re[j]);
            std::swap(im[i], im[j]);
        }
    }
    for (int len = 2; len <= N; len <<= 1) {
        const double ang = 2 * std::numbers::pi / len * (inverse ? 1 : -1);
        for (int i = 0; i < N; i += len)
            for (int k = 0; k < len / 2; ++k) {
                const double wr = std::cos(ang * k), wi = std::sin(ang * k);
                const int a = i + k, b = i + k + len / 2;
                const double tr = re[b] * wr - im[b] * wi, ti = re[b] * wi + im[b] * wr;
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
    }
}

static inline int16_t radiansToPhase(float radians) {
    constexpr float k = float(65536.0 / (2 * std::numbers::pi));
    return int16_t(int32_t(k * radians)); // Bungee Phase::fromRadians (wraps +pi to -pi)
}

void Pv::grain(const Source& src, double position, double speed, double rate, bool resetReq, float* out) {
    // ---- specify (Grain::specify) ----
    double requestHop = position - pPosition;
    if (resetReq || !pValid)
        requestHop = speed * H * rate;
    const double hopIdeal = requestHop / rate; // in frame (resampled) samples
    const bool continuous = !resetReq && pValid;
    int hop;
    double err;
    if (continuous) {
        err = pErr - hopIdeal;
        hop = int(std::round(-err));
        err += hop;
    } else {
        hop = int(std::round(hopIdeal));
        err = std::round(position) - position;
    }
    const double aSpeed = hopIdeal / H;
    const bool reverse = hop < 0;

    // ---- analyse ----
    const double centre = position + err * rate;
    for (int j = 0; j < N; ++j) {
        const int t = j < N / 2 ? j : j - N;
        re[j] = src.at(centre + t * rate) * wa[j];
        im[j] = 0;
    }
    fft(false);
    const int nb = n2 + 1; // valid bins incl. Nyquist
    for (int i = 0; i < nb; ++i) {
        const std::complex<float> x(float(re[i]), float(im[i]));
        energy[i] = x.real() * x.real() + x.imag() * x.imag();
        phase[i] = radiansToPhase(std::arg(x));
    }
    // Partials::enumerate
    {
        const float u0 = energy[nb], u1 = energy[nb + 1];
        energy[nb] = -1.f;
        energy[nb + 1] = 0.f;
        partials.clear();
        int m = 1;
        do {
            while (energy[m] < energy[m + 1])
                m++;
            Partial p;
            p.peak = m++;
            while (!(energy[m] < energy[m + 1]))
                m++;
            p.end = m++;
            partials.push_back(p);
        } while (m < nb + 1);
        energy[nb] = u0;
        energy[nb + 1] = u1;
        if (binPartials) {
            partials.clear();
            for (int m = 0; m < nb; ++m)
                partials.push_back(Partial{m, m + 1});
        }
    }
    if (continuous && transientSuppression) {
        int strongest = 0;
        for (int i = 1; i < int(partials.size()); ++i)
            if (energy[partials[i].peak] > energy[partials[strongest].peak])
                strongest = i;
        for (int i = 1; i < int(partials.size()) - 1; ++i)
            if (i != strongest && energy[partials[i].peak] > 1.5f * pEnergy[partials[i].peak]) {
                if (energy[partials[i - 1].end] > energy[partials[i].end])
                    partials[i - 1].end = partials[i].end;
                else
                    partials[i].end = partials[i - 1].end;
            }
    }

    // ---- synthesise (Synthesis::synthesise) ----
    {
        float sp = float(std::abs(aSpeed)) + 1e-20f;
        const int32_t mul = int32_t(std::max<float>(std::round(256.f / -sp), -32768.f));
        rot[0] = 0;
        for (int m = 1; m < nb; ++m) {
            const int16_t d = int16_t(phase[m - 1] - phase[m]);
            int32_t x = d;
            x *= mul;
            x >>= 8;
            rot[m] = int16_t(rot[m - 1] + int16_t(x) + d);
        }
    }
    std::vector<int16_t> delta(partials.size());
    if (continuous) {
        const int log2N = log2H + 3;
        const uint32_t a = uint32_t(int32_t(hop)) << (32 - log2N);
        int32_t mulT = 0;
        const int32_t dividend = int32_t(H) << 16, divisor = int32_t(hop) << 1;
        if (divisor)
            mulT = (dividend + std::abs(divisor) / 2) / divisor;
        for (size_t i = 0; i < partials.size(); ++i) {
            const int pk = partials[i].peak;
            const int16_t off = int16_t((reverse ? -phase[pk] : phase[pk]) - (pHop < 0 ? -pPhase[pk] : pPhase[pk]));
            const uint32_t ph = static_cast<uint32_t>(static_cast<int32_t>(phase[pk])) << 16;
            const uint32_t pph = static_cast<uint32_t>(static_cast<int32_t>(pPhase[pk])) << 16;
            const uint32_t da = (ph - pph) - static_cast<uint32_t>(pk) * a;
            const uint32_t st32 = (uint32_t(pk) << 29) + uint32_t((int32_t(da) >> 15) * mulT);
            const int16_t stretched = int16_t(int32_t(st32) >> 16);
            delta[i] = int16_t(pRot[pk] - off + stretched - rot[pk]);
        }
    } else {
        for (size_t i = 0; i < partials.size(); ++i)
            delta[i] = int16_t(-rot[partials[i].peak]);
    }
    for (int i = 0, n = 0; i < int(partials.size()); ++i)
        do
            rot[n] = int16_t(rot[n] + delta[i]);
        while (++n < partials[i].end);
    rot[n2] = rot[n2 - 1];

    // rotate, inverse transform (real output from bins 0..N/2)
    for (int i = 0; i < nb; ++i) {
        const std::complex<float> t = std::exp(std::complex<float>(0, float(rot[i]) * (std::numbers::pi_v<float> / 0x8000)));
        std::complex<float> x(float(re[i]), float(im[i]));
        if (reverse)
            x = std::conj(x);
        x *= t;
        re[i] = x.real();
        im[i] = x.imag();
    }
    im[0] = 0;
    im[n2] = 0;
    for (int i = 1; i < n2; ++i) {
        re[N - i] = re[i];
        im[N - i] = -im[i];
    }
    fft(true);

    // ---- synthesis window + overlap-add (Output::applySynthesisWindow) ----
    for (int i = 0; i < 4; ++i) {
        const int wseg = H * (i ^ 2), j = (i + 8 - 2) % 8;
        for (int k = 0; k < H; ++k) {
            const double v = re[H * j + k] * ws[wseg + k];
            lapped[i * H + k] = v + (i < 3 ? lapped[(i + 1) * H + k] : 0);
        }
    }
    for (int k = 0; k < H; ++k)
        out[k] = float(lapped[k]);

    // ---- keep state ----
    pPhase.swap(phase);
    pRot.swap(rot);
    pEnergy.swap(energy);
    phase.resize(n2 + 1);
    rot.resize(n2 + 1);
    energy.resize(n2 + 3);
    pValid = true;
    pPosition = position;
    pErr = err;
    pHop = hop;
}

} // namespace pvf
