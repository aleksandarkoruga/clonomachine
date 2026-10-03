#include "pv_model.h"

#include <algorithm>
#include <bit>
#include <cstdio>

#include "../include/sampler_regs.h"

namespace smp {

static bool readHex(const std::string& path, uint32_t* dst, size_t n) {
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f)
        return false;
    size_t i = 0;
    unsigned v;
    while (i < n && std::fscanf(f, "%x", &v) == 1)
        dst[i++] = v;
    std::fclose(f);
    return i == n;
}

bool PvTables::load(const std::string& dir) {
    uint32_t w[2 * PV_H + 1 + 3 * PV_H + PV_FADE];
    sin[PV_N / 4] = 65536;
    if (!readHex(dir + "/pv_sin.hex", sin, PV_N / 4) || !readHex(dir + "/pv_atan.hex", atan, PV_CORDIC_ITER)
        || !readHex(dir + "/pv_win.hex", w, sizeof w / sizeof w[0]))
        return false;
    std::copy(w, w + 2 * PV_H + 1 + 3 * PV_H, win);
    std::copy(w + 2 * PV_H + 1 + 3 * PV_H, w + sizeof w / sizeof w[0], fadeOut);
    return true;
}

void PvTables::cs(uint32_t a, int32_t& c, int32_t& s) const {
    a &= PV_N - 1;
    const uint32_t q = a / (PV_N / 4), r = a % (PV_N / 4);
    const int32_t sr = int32_t(sin[r]), cr = int32_t(sin[PV_N / 4 - r]);
    switch (q) {
    case 0: s = sr; c = cr; break;
    case 1: s = cr; c = -sr; break;
    case 2: s = -sr; c = -cr; break;
    default: s = -cr; c = sr; break;
    }
}

static inline int64_t rnd(int64_t v, int s) { return (v + (int64_t(1) << (s - 1))) >> s; }

static inline uint32_t bitrev(uint32_t v, int bits) {
    uint32_t r = 0;
    for (int i = 0; i < bits; ++i)
        r |= ((v >> i) & 1) << (bits - 1 - i);
    return r;
}

uint16_t pvLog2(uint32_t m) {
    if (!m)
        return 0;
    const int e = 31 - std::countl_zero(m);
    const uint32_t f = e >= 11 ? (m >> (e - 11)) & 0x7FF : (m << (11 - e)) & 0x7FF;
    return uint16_t((e << 11) | f);
}

int32_t PvCore::sat27(int64_t v) {
    constexpr int64_t hi = (int64_t(1) << (PV_DATA_BITS - 1)) - 1, lo = -(int64_t(1) << (PV_DATA_BITS - 1));
    if (v > hi) {
        ++stats.sat;
        return int32_t(hi);
    }
    if (v < lo) {
        ++stats.sat;
        return int32_t(lo);
    }
    return int32_t(v);
}

// Radix-2 decimation in frequency, in place, natural order in, bit-reversed order out.
// Forward: W = exp(-j 2pi k/M), every stage /2. Inverse: conjugate twiddles, stages
// s < PV_INV_SCALE /2.
void PvCore::fft(bool inverse) {
    constexpr int stages = PV_LOG2H + 2; // log2(M)
    for (int s = 0; s < stages; ++s) {
        const int span = PV_M >> (s + 1);
        const int sc = (!inverse || s < PV_INV_SCALE) ? 1 : 0;
        for (int i0 = 0; i0 < PV_M; i0 += 2 * span)
            for (int k = 0; k < span; ++k) {
                const int i = i0 + k, j = i + span;
                const int64_t ar = zr[i], ai = zi[i], br = zr[j], bi = zi[j];
                const int64_t sr = ar + br, si = ai + bi, dr = ar - br, di = ai - bi;
                zr[i] = sat27(sc ? rnd(sr, 1) : sr);
                zi[i] = sat27(sc ? rnd(si, 1) : si);
                int32_t c, sn;
                tab.cs(uint32_t(k) << (s + 1), c, sn);
                int64_t pr, pi;
                if (!inverse) {
                    pr = dr * c + di * sn;
                    pi = di * c - dr * sn;
                } else {
                    pr = dr * c - di * sn;
                    pi = di * c + dr * sn;
                }
                zr[j] = sat27(rnd(pr, 16 + sc));
                zi[j] = sat27(rnd(pi, 16 + sc));
                const int64_t m = std::max({std::abs(int64_t(zr[i])), std::abs(int64_t(zi[i])), std::abs(int64_t(zr[j])), std::abs(int64_t(zi[j]))});
                int64_t& mx = inverse ? stats.maxInv : stats.maxFwd;
                mx = std::max(mx, m);
            }
    }
}

// Vectoring: m = |(x,y)| * K (rounded), ph = atan2(y,x) in 1/65536 revolutions.
void PvCore::cordicVector(int32_t x0, int32_t y0, int32_t& m, int16_t& ph) const {
    constexpr int G = 2; // guard bits
    int64_t x = int64_t(x0) << G, y = int64_t(y0) << G;
    int32_t z = 0;
    if (x < 0) {
        x = -x;
        y = -y;
        z = 1 << 23; // half a revolution in 2^24 units
    }
    for (int i = 0; i < PV_CORDIC_ITER; ++i) {
        const int64_t xs = x >> i, ys = y >> i;
        if (y >= 0) {
            x += ys;
            y -= xs;
            z += int32_t(tab.atan[i]);
        } else {
            x -= ys;
            y += xs;
            z -= int32_t(tab.atan[i]);
        }
    }
    m = int32_t(rnd(x, G));
    ph = int16_t(rnd(z, 8));
}

// Rotation of (m, 0) by ph: (x, y) = m * K * (cos, sin).
void PvCore::cordicRotate(int32_t m, int16_t ph, int32_t& xo, int32_t& yo) const {
    constexpr int G = 2;
    int64_t x = int64_t(m) << G, y = 0;
    int32_t z = int32_t(ph) << 8; // 2^24 units, [-2^23, 2^23)
    if (z > (1 << 22) || z < -(1 << 22)) {
        x = -x;
        z += z > 0 ? -(1 << 23) : (1 << 23);
    }
    for (int i = 0; i < PV_CORDIC_ITER; ++i) {
        const int64_t xs = x >> i, ys = y >> i;
        if (z >= 0) {
            x -= ys;
            y += xs;
            z -= int32_t(tab.atan[i]);
        } else {
            x += ys;
            y -= xs;
            z += int32_t(tab.atan[i]);
        }
    }
    xo = int32_t(rnd(x, G));
    yo = int32_t(rnd(y, G));
}

void PvCore::run(const PvJob& job, PvSlotState& st, const std::vector<int16_t>& mem) {
    const int64_t F = int64_t(job.frames) << 32;
    const int64_t rate = job.rate;
    const bool continuous = !job.reset && st.valid;

    // ---- analysis hop (Bungee Grain::specify), frame samples ----
    int64_t hopIdeal; // Q16
    int64_t err;      // Q16
    int32_t hop;
    if (continuous) {
        int64_t d = job.pos - st.pPos;
        if (d > F / 2)
            d -= F;
        else if (d < -F / 2)
            d += F;
        int64_t d16 = std::clamp<int64_t>(d >> 16, -(int64_t(1) << 36), int64_t(1) << 36);
        hopIdeal = (d16 << 24) / rate;
        err = st.pErr - hopIdeal;
        hop = int32_t((-err + 32768) >> 16);
        err += int64_t(hop) << 16;
    } else {
        int64_t d16 = std::clamp<int64_t>((job.inc * PV_H) >> 16, -(int64_t(1) << 36), int64_t(1) << 36);
        hopIdeal = (d16 << 24) / rate;
        hop = int32_t((hopIdeal + 32768) >> 16);
        const int64_t frac = job.pos & 0xFFFFFFFFll;
        err = (frac >= (int64_t(1) << 31) ? (int64_t(1) << 32) - frac : -frac) >> 16;
    }
    const bool reverse = hop < 0;

    // ---- frame: zero-phase Hann window, packed as z[n] = x[2n] + j x[2n+1] ----
    {
        int64_t P = job.pos + ((err * rate) >> 8) - ((int64_t(PV_M) * rate) << 8);
        P %= F;
        if (P < 0)
            P += F;
        const int64_t step = rate << 8;
        for (int t = -PV_M; t < PV_M; ++t) {
            const uint32_t idx = uint32_t(P >> 32);
            const int32_t fr = int32_t((P >> 16) & 0xFFFF);
            const int32_t s0 = mem[(job.base + SMP_PREPAD + idx) % mem.size()];
            const int32_t s1 = mem[(job.base + SMP_PREPAD + idx + 1) % mem.size()];
            const int32_t s = s0 + int32_t((int64_t(s1 - s0) * fr) >> 16);
            const uint32_t j = uint32_t(t) & (PV_N - 1);
            int32_t c, sn;
            tab.cs(j, c, sn);
            const int32_t w = (65536 + c) >> 1;
            const int32_t xw = int32_t(rnd(int64_t(s) * w, 7));
            if (j & 1)
                zi[j >> 1] = xw;
            else
                zr[j >> 1] = xw;
            P += step;
            if (P >= F)
                P -= F;
        }
    }
    if (dbg) {
        std::copy(zr, zr + PV_M, dbg->frameR);
        std::copy(zi, zi + PV_M, dbg->frameI);
        dbg->hopIdeal = hopIdeal;
        dbg->err = err;
        dbg->hop = hop;
        dbg->pos = job.pos;
    }
    fft(false);
    if (dbg) {
        std::copy(zr, zr + PV_M, dbg->fftR);
        std::copy(zi, zi + PV_M, dbg->fftI);
    }

    // ---- pass A: split, CORDIC, log magnitude, frequency rotation ----
    constexpr int MB = PV_LOG2H + 2;
    for (int k = 0; k < PV_BINS; ++k) {
        const uint32_t ka = bitrev(uint32_t(k) & (PV_M - 1), MB), kb = bitrev(uint32_t(PV_M - k) & (PV_M - 1), MB);
        const int64_t Ar = zr[ka], Ai = zi[ka], Br = zr[kb], Bi = -int64_t(zi[kb]);
        const int64_t Er = Ar + Br, Ei = Ai + Bi, Dr = Ar - Br, Di = Ai - Bi;
        int32_t c, s;
        tab.cs(uint32_t(k), c, s);
        const int32_t Xr = sat27(rnd(Er * 65536 + (c * Di - s * Dr), 17));
        const int32_t Xi = sat27(rnd(Ei * 65536 + (-c * Dr - s * Di), 17));
        stats.maxSplit = std::max({stats.maxSplit, std::abs(int64_t(Xr)), std::abs(int64_t(Xi))});
        if (dbg) {
            dbg->xr[k] = Xr;
            dbg->xi[k] = Xi;
        }
        cordicVector(Xr, Xi, mag[k], phase[k]);
        L[k] = pvLog2(uint32_t(mag[k]));
    }
    {
        // Bungee Stretch::Frequency: multiplier = round(256 / -speed), speed = hopIdeal / H
        const int64_t ah = std::abs(hopIdeal);
        const int64_t q = ah ? ((int64_t(1) << 33) + ah / 2) / ah : 32768;
        const int32_t mul = -int32_t(std::min<int64_t>(q, 32768));
        if (dbg)
            dbg->mul = mul;
        rotF[0] = 0;
        for (int m = 1; m < PV_BINS; ++m) {
            const int16_t d = int16_t(phase[m - 1] - phase[m]);
            int32_t x = d;
            x *= mul;
            x >>= 8;
            rotF[m] = int16_t(rotF[m - 1] + int16_t(x) + d);
        }
    }

    // ---- partials (Bungee Partials::enumerate on the log magnitude) ----
    auto e = [&](int m) -> int32_t { return m < PV_BINS ? int32_t(L[m]) : (m == PV_BINS ? -1 : 0); };
    partials.clear();
    int strongest = 0;
    {
        int m = 1;
        do {
            while (e(m) < e(m + 1))
                m++;
            Partial p{};
            p.peak = m++;
            while (!(e(m) < e(m + 1)))
                m++;
            p.end = m++;
            if (L[p.peak] > L[partials.empty() ? p.peak : partials[strongest].peak])
                strongest = int(partials.size());
            partials.push_back(p);
        } while (m < PV_BINS + 1);
    }
    if (binPartials) {
        partials.clear();
        for (int m = 0; m < PV_BINS; ++m)
            partials.push_back(Partial{m, m + 1, 0});
        strongest = 0;
    }
    stats.partials = int(partials.size());

    // ---- pass B: transient suppression and phase advance per partial ----
    const int P = int(partials.size());
    if (continuous && transientSuppression) {
        for (int i = 1; i < P - 1; ++i)
            if (i != strongest && int32_t(L[partials[i].peak]) > int32_t(st.pL[partials[i].peak]) + PV_TRANSIENT_T) {
                if (L[partials[i - 1].end] > L[partials[i].end])
                    partials[i - 1].end = partials[i].end;
                else
                    partials[i].end = partials[i - 1].end;
            }
    }
    if (continuous) {
        const uint32_t a = uint32_t(hop) << (32 - (PV_LOG2H + 3));
        int32_t mulT = 0;
        const int32_t dividend = int32_t(PV_H) << 16, divisor = hop * 2;
        if (divisor)
            mulT = (dividend + std::abs(divisor) / 2) / divisor;
        if (dbg)
            dbg->mulT = mulT;
        for (auto& p : partials) {
            const int pk = p.peak;
            const int16_t off = int16_t((reverse ? -phase[pk] : phase[pk]) - (st.pHop < 0 ? -st.pPhase[pk] : st.pPhase[pk]));
            const uint32_t ph = uint32_t(int32_t(phase[pk])) << 16, pph = uint32_t(int32_t(st.pPhase[pk])) << 16;
            const uint32_t da = (ph - pph) - uint32_t(pk) * a;
            const uint32_t s32 = (uint32_t(pk) << 29) + uint32_t(int32_t(da) >> 15) * uint32_t(mulT);
            const int16_t stretched = int16_t(int32_t(s32) >> 16);
            p.delta = int16_t(st.pRot[pk] - off + stretched - rotF[pk]);
        }
    } else {
        for (auto& p : partials)
            p.delta = int16_t(-rotF[p.peak]);
    }

    // ---- pass C: final rotation, resynthesis, merge for the packed inverse ----
    {
        int i = 0;
        for (int n = 0; n < PV_BINS; ++n) {
            while (i < P - 1 && n >= partials[i].end)
                ++i;
            rot[n] = int16_t(rotF[n] + partials[i].delta);
        }
        rot[PV_M] = rot[PV_M - 1];
    }
    if (dbg) {
        std::copy(mag, mag + PV_BINS, dbg->mag);
        std::copy(L, L + PV_BINS, dbg->L);
        std::copy(phase, phase + PV_BINS, dbg->phase);
        std::copy(rotF, rotF + PV_BINS, dbg->rotF);
        std::copy(rot, rot + PV_BINS, dbg->rot);
        dbg->P = P;
        dbg->strongest = strongest;
        for (int i = 0; i < P && i < 1024; ++i) {
            dbg->peak[i] = partials[i].peak;
            dbg->end[i] = partials[i].end;
            dbg->delta[i] = partials[i].delta;
        }
    }
    static int32_t xr[PV_BINS], xi[PV_BINS];
    for (int k = 0; k < PV_BINS; ++k) {
        const int16_t phOut = int16_t((reverse ? -phase[k] : phase[k]) + rot[k]);
        cordicRotate(mag[k], phOut, xr[k], xi[k]);
        stats.maxRot = std::max({stats.maxRot, std::abs(int64_t(xr[k])), std::abs(int64_t(xi[k]))});
    }
    for (int k = 0; k < PV_M; ++k) {
        const int64_t Ar = xr[k], Ai = xi[k], Br = xr[PV_M - k], Bi = -int64_t(xi[PV_M - k]);
        const int64_t Er = Ar + Br, Ei = Ai + Bi, Dr = Ar - Br, Di = Ai - Bi;
        int32_t c, s;
        tab.cs(uint32_t(k), c, s);
        zr[k] = sat27(rnd(Er * 65536 - (Dr * s + Di * c), 18));
        zi[k] = sat27(rnd(Ei * 65536 + (Dr * c - Di * s), 18));
    }
    // keep state for the next grain
    for (int k = 0; k < PV_BINS; ++k) {
        st.pL[k] = L[k];
        st.pPhase[k] = phase[k];
        st.pRot[k] = rot[k];
    }
    st.valid = true;
    st.pPos = job.pos;
    st.pErr = err;
    st.pHop = hop;

    if (dbg) {
        std::copy(xr, xr + PV_BINS, dbg->xpr);
        std::copy(xi, xi + PV_BINS, dbg->xpi);
        std::copy(zr, zr + PV_M, dbg->zr);
        std::copy(zi, zi + PV_M, dbg->zi);
    }
    fft(true);
    if (dbg) {
        std::copy(zr, zr + PV_M, dbg->ifftR);
        std::copy(zi, zi + PV_M, dbg->ifftI);
    }
    // frame y(t), t = -2H .. 2H-1 (time index j = t mod N, z[n] = y[2n] + j y[2n+1])
    for (int t = -2 * PV_H; t < 2 * PV_H; ++t) {
        const uint32_t j = uint32_t(t) & (PV_N - 1);
        const uint32_t a = bitrev(j >> 1, MB);
        frame[t + 2 * PV_H] = (j & 1) ? zi[a] : zr[a];
    }
}

void PvCore::accumulate(int32_t* ring, uint32_t c, int win) const {
    for (int t = -2 * PV_H; t < 2 * PV_H; ++t) {
        int32_t w;
        const int at = t < 0 ? -t : t;
        switch (win) {
        case PV_WIN_R0: w = t < 0 ? 0 : t < PV_H ? int32_t(tab.win[2 * PV_H + 1 + t]) : int32_t(tab.win[at]); break;
        case PV_WIN_R1: {
            const int u = t + PV_H;
            w = u < 0 ? 0 : u < PV_H ? int32_t(tab.win[3 * PV_H + 1 + u]) : int32_t(tab.win[at]);
            break;
        }
        case PV_WIN_R2: {
            const int u = t + 2 * PV_H;
            w = u < PV_H ? int32_t(tab.win[4 * PV_H + 1 + u]) : int32_t(tab.win[at]);
            break;
        }
        default: w = int32_t(tab.win[at]); break;
        }
        if (w)
            ring[(c + uint32_t(t)) & (PV_RING - 1)] += int32_t(rnd(int64_t(frame[t + 2 * PV_H]) * w, 18));
    }
}

} // namespace smp
