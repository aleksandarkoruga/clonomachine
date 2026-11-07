// SPDX-License-Identifier: AGPL-3.0-or-later
// Derived from QuadMorphFilter (https://github.com/OTODESK4193/QuadMorphFilter) by OTODESK4193,
// licensed under the GNU Affero General Public License v3; this file is therefore under the
// AGPL-3.0 as well (the rest of this repository is MIT, see LICENSE).
//
// The quad morph filter of the synth model: four filter slots in parallel, each running one of
// the plugin's models, blended by an XY position with equal-power weights. Per slot: an input
// gain that depends on the model and resonance ("comp"), the model, and an automatic gain
// control that brings the output RMS back to the input's (not for the clean SVF; ladders pass
// it by near self-oscillation). Coefficients are recomputed every 16 processing-rate samples.
//
// Ported models (index as in the plugin): 0 clean SVF, 1 Moog ladder, 2 diode (TB-303), 3 SEM,
// 5 formant, 7 MS-20, 12 CEM3320, 15 Jupiter. Other indices fall back to the clean SVF.
// Deviations from the plugin: the cutoff / resonance targets come from the synth's modulation
// and are not smoothed again (the plugin smooths them with 10 ms); the slots always run at the
// synth's processing rate (no per-model oversampler); the XY position is the synth's own.
//
// cost(): multiplies per instance-sample, from the plugin's op counts (minimal form, with the
// per-sample wrapper of 7 multiplies); fn(): nonlinear function evaluations (tanh, exp, ...).
#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace synth {

struct QmfFixed {  // word-length emulation of the filter datapath
    bool on = false;
    static double q(double x, int frac) {
        const double s = std::ldexp(1.0, frac);
        return std::nearbyint(x * s) / s;
    }
    double st(double x) const { return on ? q(std::clamp(x, -64.0, 64.0), 24) : x; }  // s32 Q6.24 states
    double cf(double x) const {                                                        // 18 significant bits
        if (!on || x == 0)
            return x;
        int e;
        std::frexp(x, &e);
        return q(x, 17 - e);
    }
    // tanh as the hardware would: a 1024-entry table over [-8, 8], linear interpolation
    double tanh(double x) const {
        if (!on)
            return std::tanh(x);
        static double T[1026];
        static bool init = false;
        if (!init) {
            for (int i = 0; i < 1026; ++i)
                T[i] = q(std::tanh(-8.0 + 16.0 * i / 1024), 17);
            init = true;
        }
        const double p = (std::clamp(x, -8.0, 8.0) + 8.0) * 64.0;
        const int i = std::min(int(p), 1023);
        const double f = p - i;
        return T[i] + f * (T[i + 1] - T[i]);
    }
};

struct QmfSlot {
    int model = 0, type = 0, slope = 0;
    int N = 1;
    double fs = 96000;
    // coefficients
    double g = 0, G = 0, R = 0.5, h = 1, k = 0, scalerSVF = 1, scalerMoog = 1;
    double diode_g = 0, diode_h = 0, msAlpha = 0.9995;
    double form_g[3]{}, form_R[3]{}, form_h[3]{};
    double res = 0.707;
    double rmsCoef = 0, agcCoef = 0;
    // state
    double s1[8]{}, s2[8]{}, z[4][4]{}, adaaPrev[4]{}, hpfS = 0;
    double sk1[8]{}, sk2[8]{}, dcx[8]{}, dcy[8]{};
    double fs1[3]{}, fs2[3]{};
    double rmsIn = 0, rmsOut = 0, agc = 1;
    int macs = 0, fns = 0;

    void set_rate(double rate) {
        fs = rate;
        rmsCoef = 1 - std::exp(-1.0 / (0.0045 * fs));
        agcCoef = 1 - std::exp(-1.0 / (0.045 * fs));
        msAlpha = std::exp(-2 * M_PI * 3.5 / fs);
    }
    void set_slope(int m, int s) {
        model = m;
        slope = s;
        if (m == 5)
            N = 1;
        else if (m == 2)
            N = 1;
        else if (is_ladder(m))
            N = s <= 1 ? 1 : s == 2 ? 2 : 4;
        else  // SVF cascades (0, 3, 7 and the fallback)
            N = s == 0 ? 1 : s == 1 ? 2 : s == 2 ? 4 : 8;
        scalerSVF = N > 1 ? std::pow(0.6, std::log2(double(N))) : 1.0;
        scalerMoog = N > 1 ? std::pow(0.7, std::log2(double(N))) : 1.0;
    }
    static bool is_ladder(int m) { return m == 1 || m == 12 || m == 15; }
    static double jmap(double v, double a, double b, double c, double d) { return c + (d - c) * (v - a) / (b - a); }

    void update(double fc, double r, const QmfFixed& F) {
        res = std::clamp(r, 0.1, 10.0);
        fc = std::clamp(fc, 20.0, 20000.0);
        const int m = model;
        if (m == 2) {
            static const double phi[4] = {1.0, 1.21, 1.56, 1.56};
            diode_g = F.cf(std::tan(M_PI * std::clamp(fc * phi[std::clamp(slope, 0, 3)], 20.0, fs * 0.45) / fs));
            diode_h = F.cf(std::tan(M_PI * 8.0 / fs));
            fns += 2;
            return;
        }
        if (m == 5) {
            const double v = std::clamp(jmap(std::log10(fc), std::log10(20.0), std::log10(20000.0), 0, 4), 0.0, 4.0);
            int idx = int(v);
            double fr = v - idx;
            if (idx >= 4)
                idx = 3, fr = 1;
            static const double f1[5] = {730, 270, 300, 530, 400}, f2[5] = {1090, 2290, 870, 1840, 840},
                                f3[5] = {2440, 3010, 2240, 2480, 2800};
            const double fa[3] = {f1[idx] + (f1[idx + 1] - f1[idx]) * fr, f2[idx] + (f2[idx + 1] - f2[idx]) * fr,
                                  f3[idx] + (f3[idx + 1] - f3[idx]) * fr};
            const double Q = 2 + 3 * res, Rf = 1 / (2 * Q);
            for (int f = 0; f < 3; ++f) {
                form_g[f] = F.cf(std::tan(M_PI * fa[f] / fs));
                form_R[f] = F.cf(Rf);
                form_h[f] = F.cf(1 / (1 + 2 * Rf * form_g[f] + form_g[f] * form_g[f]));
            }
            fns += 4;
            return;
        }
        const double fcs = std::clamp(fc, 20.0, fs * 0.45);
        g = F.cf(std::tan(M_PI * fcs / fs));
        G = F.cf(g / (1 + g));
        fns += 1;
        if (!is_ladder(m)) {
            const double adj = res * (m != 7 ? scalerSVF : 1.0);
            R = F.cf(1 / (2 * adj));
            h = F.cf(1 / (1 + 2 * R * g + g * g));
        } else {  // ladders 1, 12, 15
            k = F.cf(jmap(res, 0.1, 10, 0, 4.5) * scalerMoog);
        }
    }

    static double logcosh(double x) {
        const double ax = std::fabs(x);
        return ax > 12 ? ax - 0.6931471806 : ax + std::log1p(std::exp(-2 * ax)) - 0.6931471806;
    }

    double process(double x, const QmfFixed& F) {
        const int m = model;
        double comp = 1;
        if (m == 0 || m == 5)
            comp = 1 + 0.1 * res;
        else if (m == 1 || m == 12 || m == 15)
            comp = 1 + 0.5 * k;
        else if (m == 2)
            comp = 1 + 0.2 * std::clamp(jmap(res, 0.1, 10, 0, slope == 0 ? 4.2 : slope == 1 ? 4.6 : 5.0), 0.0, 5.0);
        else if (m == 7)
            comp = 1 + 0.15 * res;
        x *= comp;
        rmsIn += rmsCoef * (x * x - rmsIn);
        double out = x;
        macs += 1 + 7;  // comp + wrapper
        fns += 2;       // wrapper: 1 divide + 1 sqrt (could run at a lower rate in hardware)
        if (!is_ladder(m) && m != 2 && m != 5) {
            const int mm = (m == 3 || m == 7) ? m : 0;
            for (int s = 0; s < N; ++s) {
                double lp, bp, hp;
                if (mm == 0) {
                    hp = F.st((out - (2 * R + g) * s1[s] - s2[s]) * h);
                    bp = F.st(g * hp + s1[s]);
                    lp = F.st(g * bp + s2[s]);
                    s1[s] = F.st(g * hp + bp);
                    s2[s] = F.st(g * bp + lp);
                    macs += 4;
                } else if (mm == 3) {
                    const double bs = F.tanh(s1[s]);
                    hp = F.st((out - 2 * R * bs - g * s1[s] - s2[s]) * (1 / (1 + g * g)));
                    bp = F.st(g * hp + s1[s]);
                    lp = F.st(g * bp + s2[s]);
                    s1[s] = F.st(g * hp + bp);
                    s2[s] = F.st(g * bp + lp);
                    macs += 5;
                    fns += 1;
                } else {  // MS-20
                    const double msk = jmap(res, 0.1, 10, 0, 2.5);
                    const double raw = sk2[s] * msk;
                    const double blk = F.st(raw - dcx[s] + msAlpha * dcy[s]);
                    dcx[s] = raw;
                    dcy[s] = blk;
                    const double fb = blk > 0 ? F.tanh(blk * 1.5) : F.tanh(blk * 0.8);
                    const double v1 = (out - fb - sk1[s]) * G;
                    const double y1 = F.st(v1 + sk1[s]);
                    sk1[s] = F.st(y1 + v1);
                    const double v2 = (y1 - sk2[s]) * G;
                    const double y2 = F.st(v2 + sk2[s]);
                    sk2[s] = F.st(y2 + v2);
                    lp = y2, hp = out - y1, bp = y1 - y2;
                    macs += 5;
                    fns += 1;
                }
                out = type == 0 ? lp : type == 1 ? bp : type == 2 ? hp : lp + hp;
            }
        } else if (m == 5) {
            double sum = 0;
            static const double gains[3] = {1.0, 0.5, 0.2};
            for (int f = 0; f < 3; ++f) {
                const double hp = F.st((out - (2 * form_R[f] + form_g[f]) * fs1[f] - fs2[f]) * form_h[f]);
                const double bp = F.st(form_g[f] * hp + fs1[f]);
                const double lp = F.st(form_g[f] * bp + fs2[f]);
                fs1[f] = F.st(form_g[f] * hp + bp);
                fs2[f] = F.st(form_g[f] * bp + lp);
                sum += bp * gains[f];
            }
            out = sum;
            macs += 13;
        } else if (m == 2) {
            out = diode(out, F);
        } else {  // ladders 1, 12, 15
            for (int s = 0; s < N; ++s)
                out = ladder(s, out, F);
        }
        rmsOut += rmsCoef * (out * out - rmsOut);
        double target = 1;
        if (rmsOut > 1e-8)
            target = std::clamp(std::sqrt((rmsIn + 1e-8) / (rmsOut + 1e-8)), 0.1, 15.0);
        agc += agcCoef * (target - agc);
        double fin = agc;
        if (m == 0) {
            fin = 1;
        } else if (m == 1 || m == 2 || m == 12 || m == 15) {
            const double kn = std::clamp(m == 2 ? jmap(res, 0.1, 10, 0, 1) : k / 4.5, 0.0, 1.0);
            if (kn > 0.85) {
                const double by = std::clamp((kn - 0.85) * 10, 0.0, 1.0);
                agc = agc * (1 - by * 0.01) + by * 0.01;
                fin = agc * (1 - by) + by;
            }
        }
        return F.st(out * fin);
    }

    double ladder(int s, double x, const QmfFixed& F) {
        double* st = z[s];
        const double inv1pg = 1 / (1 + g);
        const double S1 = st[0] * inv1pg, S2 = st[1] * inv1pg, S3 = st[2] * inv1pg, S4 = st[3] * inv1pg;
        const double G2 = G * G, G3 = G2 * G, G4 = G3 * G;
        const double sigma = G3 * S1 + G2 * S2 + G * S3 + S4;
        const double c = x - k * sigma;
        double u;
        const int m = model;
        if (m == 1) {
            const double inv = 1 / (1 + k * G4);
            double uh = c + (k > 3.5 ? 1e-5 : 0);
            if (k <= 4.0) {
                for (int it = 0; it < 3; ++it) {
                    const double t = F.tanh(uh * inv), sech2 = 1 - t * t;
                    const double f = uh + k * G4 * t - c, fp = 1 + k * G4 * inv * sech2;
                    if (std::fabs(fp) < 1e-10)
                        break;
                    uh -= f / fp;
                }
                macs += 12;
                fns += 3 + 3;  // tanh + divide per iteration
            }
            const double ti = uh * inv;
            const double d = ti - adaaPrev[s];
            u = std::fabs(d) > 1e-4 ? (logcosh(ti) - logcosh(adaaPrev[s])) / d : F.tanh(0.5 * (ti + adaaPrev[s]));
            adaaPrev[s] = ti;
            macs += 10;
            fns += 3;  // exp + log1p (one logcosh cached) + divide
        } else if (m == 12) {
            const double up = c + (k > 3.5 ? 1e-6 : 0);
            u = F.tanh(up * 1.4) / 1.4;
            macs += 11;
            fns += 1;
        } else {  // 15 Jupiter
            const double up = c + (k > 3.5 ? 1e-6 : 0);
            u = up / (1 + std::fabs(up * 0.5));
            macs += 9;
            fns += 1 + 4;
        }
        double y[4], in = u;
        for (int i = 0; i < 4; ++i) {
            const double v = (in - st[i]) * G;
            double yy = v + st[i];
            if (m == 15)
                yy = F.tanh(yy);
            st[i] = F.st(st[i] + 2 * v);
            y[i] = F.st(yy);
            in = y[i];
        }
        const int t = type;
        if (slope == 0) {
            return t == 0 ? y[1] : t == 1 ? 2 * (y[0] - y[1]) : t == 2 ? x - 2 * y[0] + y[1] : y[1] + (x - 2 * y[0] + y[1]);
        }
        const double hp = x - 4 * y[0] + 6 * y[1] - 4 * y[2] + y[3];
        return t == 0 ? y[3] : t == 1 ? 4 * (y[1] - y[3]) : t == 2 ? hp : y[3] + hp;
    }

    double diode(double x, const QmfFixed& F) {
        double* st = z[0];
        const double gg = diode_g, inv1pg = 1 / (1 + gg), Gd = gg * inv1pg;
        const double G2 = Gd * Gd, G3 = G2 * Gd, G4 = G3 * Gd;
        const double kmax = slope == 0 ? 4.2 : slope == 1 ? 4.6 : 5.0;
        const double kk = std::clamp(jmap(res, 0.1, 10, 0, kmax), 0.0, kmax);
        const double S1 = st[0] * inv1pg, S2 = st[1] * inv1pg, S3 = st[2] * inv1pg, S4 = st[3] * inv1pg;
        const double sigma = G3 * S1 + G2 * S2 + Gd * S3 + S4;
        double u = (x - kk * sigma) / (1 + kk * G4);
        if (kk > 3.5)
            u += 1e-5;
        if (kk <= 4.0) {
            for (int it = 0; it < 3; ++it) {
                const double us = F.tanh(u), y1 = F.tanh(Gd * us + S1), y2 = F.tanh(Gd * y1 + S2),
                             y3 = F.tanh(Gd * y2 + S3), y4 = F.tanh(Gd * y3 + S4);
                const double f = u + kk * y4 - x;
                const double dch = G4 * (1 - us * us) * (1 - y1 * y1) * (1 - y2 * y2) * (1 - y3 * y3) * (1 - y4 * y4);
                const double fp = 1 + kk * dch;
                if (std::fabs(fp) < 1e-10)
                    break;
                u -= f / fp;
            }
            macs += 52;
            fns += 3 * 6;  // 5 tanh + 1 divide per iteration
        }
        const double us = F.tanh(u);
        double in = us;
        for (int i = 0; i < 4; ++i) {
            const double v = (in - st[i]) * Gd;
            const double y = F.tanh(v + st[i]);
            st[i] = F.st(st[i] + 2 * v);
            in = y;
        }
        const double hg = diode_h / (1 + diode_h);
        const double v = (in - hpfS) * hg, lp = v + hpfS;
        hpfS = F.st(lp + v);
        macs += 12;
        fns += 5;
        return (in - lp) * 0.95;
    }
};

struct QuadMorph {
    QmfSlot slot[4];
    double w[4] = {0, 0, 0, 0};  // smoothed weights
    double smooth = 0;
    int counter = 0;
    void set_rate(double fs) {
        for (auto& s : slot)
            s.set_rate(fs);
        smooth = 1 - std::exp(-1.0 / (0.005 * fs));
    }
    // equal-power corner weights for x, y in [0, 1]: slot 0 (0,0) 1 (1,0) 2 (0,1) 3 (1,1)
    static std::array<double, 4> weights(double x, double y) {
        const double cx = std::cos(x * M_PI / 2), sx = std::sin(x * M_PI / 2), cy = std::cos(y * M_PI / 2),
                     sy = std::sin(y * M_PI / 2);
        std::array<double, 4> v = {cx * cy, sx * cy, cx * sy, sx * sy};
        double s = 0;
        for (double a : v)
            s += a * a;
        if (s > 1e-8)
            for (double& a : v)
                a /= std::sqrt(s);
        return v;
    }
    // every 16 samples: new targets; per sample: process
    double process(double in, double fc, double res, const double oct[4], double x, double y, const QmfFixed& F) {
        if (counter++ % 16 == 0)
            for (int i = 0; i < 4; ++i)
                slot[i].update(fc * std::exp2(oct[i]), res, F);
        const auto tw = weights(x, y);
        double out = 0;
        for (int i = 0; i < 4; ++i) {
            w[i] += smooth * (tw[i] - w[i]);
            out += w[i] * slot[i].process(in, F);
        }
        return out;
    }
    int take_macs() {
        int m = 4 + 4;  // weights
        for (auto& s : slot)
            m += s.macs, s.macs = 0;
        return m;
    }
    int take_fns() {
        int f = 0;
        for (auto& s : slot)
            f += s.fns, s.fns = 0;
        return f;
    }
};

}  // namespace synth
