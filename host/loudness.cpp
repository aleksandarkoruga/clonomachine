// Loudness matching for the sample card: see loudness.h.
#include "loudness.h"

#include <algorithm>
#include <cmath>
#include <deque>

namespace {

struct Biquad {
    double b0, b1, b2, a1, a2; // normalised by a0
    double z1 = 0, z2 = 0;
    double run(double x) { // transposed direct form II
        const double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
};

Biquad make(double b0, double b1, double b2, double a0, double a1, double a2) {
    return Biquad{b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0};
}

// BS.1770 K-weighting at any sample rate (libebur128's analog prototypes: the standard's
// published 48 kHz coefficients exactly, the high pass with its unnormalised 1 -2 1)
void k_filters(uint32_t sr, Biquad& shelf, Biquad& hp) {
    {
        const double f0 = 1681.974450955533, G = 3.999843853973347, Q = 0.7071752369554196;
        const double K = std::tan(M_PI * f0 / sr), Vh = std::pow(10.0, G / 20), Vb = std::pow(Vh, 0.4996667741545416);
        shelf = make(Vh + Vb * K / Q + K * K, 2 * (K * K - Vh), Vh - Vb * K / Q + K * K, 1 + K / Q + K * K, 2 * (K * K - 1),
                     1 - K / Q + K * K);
    }
    {
        const double f0 = 38.13547087602444, Q = 0.5003270373238773, K = std::tan(M_PI * f0 / sr), a0 = 1 + K / Q + K * K;
        hp = Biquad{1, -2, 1, 2 * (K * K - 1) / a0, (1 - K / Q + K * K) / a0};
    }
}

Biquad butter_lp(double fc, uint32_t sr) {
    const double w = 2 * M_PI * fc / sr, c = std::cos(w), al = std::sin(w) / (2 * M_SQRT1_2);
    return make((1 - c) / 2, 1 - c, (1 - c) / 2, 1 + al, -2 * c, 1 - al);
}

// zero-phase low pass (forward and backward: 24 dB/oct, no phase shift)
std::vector<float> zp_lp(const std::vector<float>& x, double fc, uint32_t sr) {
    std::vector<float> y(x.size());
    Biquad f = butter_lp(fc, sr);
    for (size_t i = 0; i < x.size(); ++i)
        y[i] = float(f.run(x[i]));
    Biquad b = butter_lp(fc, sr);
    for (size_t i = x.size(); i-- > 0;)
        y[i] = float(b.run(y[i]));
    return y;
}

double db(double g) { return 20 * std::log10(g); }

} // namespace

double lufs(const float* x, size_t n, uint32_t sr) {
    if (!n || !sr)
        return -INFINITY;
    Biquad shelf, hp;
    k_filters(sr, shelf, hp);
    const size_t step = std::max<size_t>(1, sr / 10), blk = 4 * step; // 100 ms hop, 400 ms block
    std::vector<double> hop_sum((n + step - 1) / step, 0.0);
    for (size_t i = 0; i < n; ++i) {
        const double y = hp.run(shelf.run(x[i]));
        hop_sum[i / step] += y * y;
    }
    std::vector<double> z; // mean square per block
    if (n <= blk) {
        double s = 0;
        for (double v : hop_sum)
            s += v;
        z.push_back(s / double(n));
    } else
        for (size_t h = 0; (h + 4) * step <= n; ++h)
            z.push_back((hop_sum[h] + hop_sum[h + 1] + hop_sum[h + 2] + hop_sum[h + 3]) / double(blk));
    auto loud = [](double ms) { return -0.691 + 10 * std::log10(ms); };
    double s = 0;
    size_t k = 0;
    for (double v : z)
        if (v > 0 && loud(v) > -70) {
            s += v;
            ++k;
        }
    if (!k)
        return -INFINITY;
    const double rel = loud(s / double(k)) - 10;
    s = 0;
    k = 0;
    for (double v : z)
        if (v > 0 && loud(v) > -70 && loud(v) > rel) {
            s += v;
            ++k;
        }
    return k ? loud(s / double(k)) : -INFINITY;
}

namespace {

struct Work {
    uint32_t sr;
    size_t n, pad;                    // the file is xp[pad, pad + n), circularly padded around
    std::vector<float> band[3], lvl[3]; // low / mid / high, their level in dB
    float peak_db[3];
};

// 3-band compressor at `depth` dB below each band's peak (log-domain, smoothed gain
// reduction with attack / release, soft knee); returns the largest reduction in dB.
double compress(const Work& w, double depth, const LevelParams& p, std::vector<float>& out) {
    const size_t N = w.band[0].size();
    out.assign(N, 0.0f);
    const double aa = std::exp(-1.0 / (p.attack_ms * 1e-3 * w.sr));
    double most = 0;
    for (int b = 0; b < 3; ++b) {
        const double ar = std::exp(-1.0 / ((b == 0 ? p.low_release_ms : p.release_ms) * 1e-3 * w.sr));
        const double T = w.peak_db[b] - depth, W = p.knee, s = 1.0 / p.ratio - 1;
        double y = 0;
        for (size_t i = 0; i < N; ++i) {
            const double over = w.lvl[b][i] - T;
            double gr = 0;
            if (2 * over > W)
                gr = -s * over;
            else if (2 * over > -W)
                gr = -s * (over + W / 2) * (over + W / 2) / (2 * W);
            y = gr > y ? aa * y + (1 - aa) * gr : ar * y + (1 - ar) * gr;
            if (i >= w.pad && i < w.pad + w.n)
                most = std::max(most, y);
            out[i] += float(w.band[b][i] * std::exp(-y * (M_LN10 / 20)));
        }
    }
    return most;
}

// look-ahead peak limiter at full scale: the gain is the minimum needed over the next W
// samples, recovers with the release, averaged over W samples (so it is down by the time a
// peak arrives); returns the largest reduction in dB. y = x * gain, x padded as in Work.
double limit(const std::vector<float>& x, float g_in, const Work& w, const LevelParams& p, std::vector<float>& y) {
    const size_t N = x.size(), W = std::max<size_t>(1, size_t(p.lim_lookahead_ms * 1e-3 * w.sr));
    std::vector<float> req(N);
    for (size_t i = 0; i < N; ++i) {
        const double a = std::fabs(double(x[i]) * g_in);
        req[i] = a > 1.0 ? float(1.0 / a) : 1.0f;
    }
    std::vector<float> h(N); // min of req[i .. i + W]
    std::deque<size_t> q;
    for (size_t j = N; j-- > 0;) {
        while (!q.empty() && req[q.back()] >= req[j])
            q.pop_back();
        q.push_back(j);
        while (q.front() > j + W)
            q.pop_front();
        h[j] = req[q.front()];
    }
    const double ar = std::exp(-1.0 / (p.lim_release_ms * 1e-3 * w.sr));
    double r = 1, sum = 0, most = 1;
    std::vector<float> rr(N);
    y.resize(N);
    for (size_t i = 0; i < N; ++i) {
        r = std::min<double>(h[i], 1 - (1 - r) * ar);
        rr[i] = float(r);
        sum += r;
        if (i > W)
            sum -= rr[i - W - 1];
        const double g = sum / double(std::min(i + 1, W + 1));
        double v = double(x[i]) * g_in * g;
        v = std::clamp(v, -1.0, 1.0); // float rounding only
        y[i] = float(v);
        if (i >= w.pad && i < w.pad + w.n)
            most = std::min(most, g);
    }
    return -db(most);
}

struct Try {
    double depth, gain_db, limit, comp, after;
    std::vector<float> y;
};

// makeup gain into the limiter so the result lands on tgt (the limiter lowers it a little)
Try reach(const Work& w, const std::vector<float>& c, double depth, double comp, double tgt, const LevelParams& p) {
    Try t{depth, 0, 0, comp, 0, {}};
    t.gain_db = tgt - lufs(c.data() + w.pad, w.n, w.sr);
    for (int it = 0; it < 4; ++it) {
        t.limit = limit(c, float(std::pow(10.0, t.gain_db / 20)), w, p, t.y);
        t.after = lufs(t.y.data() + w.pad, w.n, w.sr);
        if (std::fabs(tgt - t.after) < 0.05)
            break;
        t.gain_db += tgt - t.after;
    }
    return t;
}

} // namespace

LevelResult level(std::vector<float>& x, uint32_t sr, const LevelParams& p) {
    LevelResult r;
    const size_t n = x.size();
    r.before = r.after = lufs(x.data(), n, sr);
    if (!std::isfinite(r.before) || r.before >= p.target - 0.5 || n < 64)
        return r;
    double tgt = std::min(p.target, r.before + p.max_boost);
    r.capped = tgt < p.target;

    Work w;
    w.sr = sr;
    w.n = n;
    w.pad = size_t(0.3 * sr); // detector pre-roll / filter and look-ahead edges: the file as a loop
    const size_t N = n + 2 * w.pad;
    std::vector<float> xp(N);
    for (size_t i = 0; i < N; ++i)
        xp[i] = x[(i + n - w.pad % n) % n];
    w.band[0] = zp_lp(xp, p.xover_lo, sr);
    std::vector<float> rest(N);
    for (size_t i = 0; i < N; ++i)
        rest[i] = xp[i] - w.band[0][i];
    w.band[1] = zp_lp(rest, p.xover_hi, sr);
    w.band[2].resize(N);
    for (size_t i = 0; i < N; ++i)
        w.band[2][i] = rest[i] - w.band[1][i];
    for (int b = 0; b < 3; ++b) {
        w.lvl[b].resize(N);
        float pk = 1e-9f;
        for (size_t i = 0; i < N; ++i) {
            const float a = std::fabs(w.band[b][i]);
            w.lvl[b][i] = float(db(std::max(a, 1e-9f)));
            if (i >= w.pad && i < w.pad + n)
                pk = std::max(pk, a);
        }
        w.peak_db[b] = float(db(pk));
    }

    // the least compression that keeps the limiter within lim_max; else the one it needs least
    static const double depths[] = {0, 2, 4, 6, 9, 12, 15, 18, 24};
    Try best{};
    bool have = false, fits = false;
    std::vector<float> c;
    for (double d : depths) {
        double comp = 0;
        if (d == 0)
            c = xp;
        else
            comp = compress(w, d, p, c);
        Try t = reach(w, c, d, comp, tgt, p);
        if (!have || t.limit < best.limit - 0.25) {
            best = std::move(t);
            have = true;
        }
        if (best.limit <= p.lim_max) {
            fits = true;
            break;
        }
    }
    if (!fits) { // as loud as lim_max allows: the limiter alone, or that compression if it gets 1 dB more
        auto raise = [&](double depth) {
            double comp = 0;
            if (depth == 0)
                c = xp;
            else
                comp = compress(w, depth, p, c);
            double lo = r.before, hi = tgt;
            Try ok = reach(w, c, depth, comp, lo, p);
            for (int it = 0; it < 12; ++it) {
                const double mid = (lo + hi) / 2;
                Try t = reach(w, c, depth, comp, mid, p);
                if (t.limit <= p.lim_max) {
                    ok = std::move(t);
                    lo = mid;
                } else
                    hi = mid;
            }
            return ok;
        };
        Try alone = raise(0);
        if (best.depth > 0) {
            Try comp = raise(best.depth);
            best = comp.after > alone.after + 1.0 ? std::move(comp) : std::move(alone);
        } else
            best = std::move(alone);
        r.capped = true;
    }
    float pk = 0;
    for (size_t i = 0; i < n; ++i)
        pk = std::max(pk, std::fabs(best.y[w.pad + i]));
    const float g = pk > 0 ? 1.0f / pk : 1.0f; // back to a full-scale peak (the board's rule)
    for (size_t i = 0; i < n; ++i)
        x[i] = best.y[w.pad + i] * g;
    r.after = lufs(x.data(), n, sr);
    r.boost = r.after - r.before;
    r.depth = best.depth;
    r.comp = best.comp;
    r.limit = best.limit;
    r.changed = true;
    return r;
}
