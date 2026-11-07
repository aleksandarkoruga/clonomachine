// Wavetables for the synth model, see wt_tables.h.
#include "wt_tables.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "wav.h"

namespace wt {

void fft(std::vector<std::complex<double>>& a, bool inverse) {
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        const double ang = 2 * M_PI / double(len) * (inverse ? 1 : -1);
        const std::complex<double> wl(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w(1);
            for (size_t j = 0; j < len / 2; ++j) {
                const auto u = a[i + j], v = a[i + j + len / 2] * w;
                a[i + j] = u + v;
                a[i + j + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}

Table build(const std::string& name, const std::vector<float>& frames, int n0, int nmin) {
    Table t;
    t.name = name;
    t.n0 = n0;
    t.nmin = std::max(nmin, kMinN);
    t.frames = int(frames.size() / size_t(n0));
    t.levels = 0;
    for (int N = n0; N >= kMinN; N >>= 1)
        ++t.levels;
    t.lvl.assign(t.levels, {});
    for (int k = 0; k < t.levels; ++k)
        t.lvl[k].resize(size_t(t.frames) * t.size(k));
    std::vector<std::complex<double>> X(n0), Y;
    for (int f = 0; f < t.frames; ++f) {
        for (int i = 0; i < n0; ++i)
            X[i] = frames[size_t(f) * n0 + i];
        fft(X, false);
        for (int k = 0; k < t.levels; ++k) {
            const int N = t.size(k);
            Y.assign(N, 0);
            Y[0] = X[0];
            for (int h = 1; h <= t.harm(k); ++h) {
                Y[h] = X[h];
                Y[N - h] = X[n0 - h];
            }
            fft(Y, true);
            for (int i = 0; i < N; ++i)
                t.lvl[k][size_t(f) * N + i] = float(Y[i].real() / n0);
        }
    }
    float peak = 0;
    for (auto& L : t.lvl)
        for (float v : L)
            peak = std::max(peak, std::fabs(v));
    t.qscale = peak > 1.0f ? 1.0 / peak : 1.0;  // band-limiting overshoots (Gibbs): scale, don't clip
    t.q.assign(t.levels, {});
    t.base.assign(t.levels, 0);
    uint32_t off = 0;
    for (int k = 0; k < t.levels; ++k) {
        t.base[k] = off;
        off += uint32_t(t.lvl[k].size());
        t.q[k].resize(t.lvl[k].size());
        for (size_t i = 0; i < t.lvl[k].size(); ++i)
            t.q[k][i] = int16_t(std::lrint(std::clamp(t.lvl[k][i] * t.qscale * 32767.0, -32768.0, 32767.0)));
    }
    t.words = off;
    return t;
}

bool load_clm_wav(const std::string& path, Table& t, int nmin) {
    wav::Audio a;
    if (!wav::read(path, a))
        return false;
    int n0 = 2048;
    if (FILE* f = std::fopen(path.c_str(), "rb")) {  // 'clm ' chunk: "<!>2048 ..."
        char buf[4096];
        const size_t n = std::fread(buf, 1, sizeof buf, f);
        std::fclose(f);
        for (size_t o = 12; o + 12 < n; ++o)
            if (!std::memcmp(buf + o, "clm ", 4) && !std::memcmp(buf + o + 8, "<!>", 3)) {
                n0 = std::atoi(buf + o + 11);
                break;
            }
    }
    if (n0 < kMinN || (n0 & (n0 - 1)) || a.x.size() < size_t(n0))
        return false;
    a.x.resize(a.x.size() / n0 * n0);
    std::string name = path.substr(path.find_last_of('/') + 1);
    t = build(name.substr(0, name.find_last_of('.')), a.x, n0, nmin);
    return true;
}

bool save_clm_wav(const std::string& path, const std::vector<float>& frames, int n0) {
    // 32-bit float WAV with the 'clm ' chunk (frame size) wavetable editors read
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f)
        return false;
    char clm[48];
    std::snprintf(clm, sizeof clm, "<!>%-4d 10000000 wavetable (model)", n0);
    const uint32_t clmLen = uint32_t(std::strlen(clm)) + (std::strlen(clm) & 1);
    const uint32_t bytes = uint32_t(frames.size() * 4);
    auto w32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
    auto w16 = [&](uint16_t v) { std::fwrite(&v, 2, 1, f); };
    std::fwrite("RIFF", 1, 4, f);
    w32(4 + 8 + 16 + 8 + clmLen + 8 + bytes);
    std::fwrite("WAVEfmt ", 1, 8, f);
    w32(16);
    w16(3);
    w16(1);
    w32(48000);
    w32(48000 * 4);
    w16(4);
    w16(32);
    std::fwrite("clm ", 1, 4, f);
    w32(clmLen);
    std::fwrite(clm, 1, clmLen, f);
    std::fwrite("data", 1, 4, f);
    w32(bytes);
    std::fwrite(frames.data(), 4, frames.size(), f);
    std::fclose(f);
    return true;
}

namespace {
// one frame from harmonic amplitudes (sine phase), normalised later by the caller
void additive(float* out, int n0, const std::vector<double>& amp) {
    std::vector<double> acc(n0, 0.0);
    for (size_t h = 1; h < amp.size(); ++h)
        if (amp[h] != 0)
            for (int i = 0; i < n0; ++i)
                acc[i] += amp[h] * std::sin(2 * M_PI * double(h) * i / n0);
    for (int i = 0; i < n0; ++i)
        out[i] = float(acc[i]);
}
}  // namespace

Table builtin(const std::string& name, int frames, int n0, int nmin) {
    std::vector<float> x(size_t(frames) * n0);
    const int H = n0 / 2;
    for (int f = 0; f < frames; ++f) {
        float* o = &x[size_t(f) * n0];
        const double t = frames > 1 ? double(f) / (frames - 1) : 0.0;
        if (name == "sine") {
            for (int i = 0; i < n0; ++i)
                o[i] = float(std::sin(2 * M_PI * i / n0));
        } else if (name == "saw") {
            for (int i = 0; i < n0; ++i)
                o[i] = float(1.0 - 2.0 * i / n0);  // naive: the mip build band-limits it
        } else if (name == "basic") {
            // sine -> triangle -> saw -> square, crossfaded in thirds (the usual basic-shapes table)
            const double seg = t * 3;
            const int s = std::min(2, int(seg));
            const double u = seg - s;
            for (int i = 0; i < n0; ++i) {
                const double p = double(i) / n0;
                const double sh[4] = {std::sin(2 * M_PI * p), p < 0.25 ? 4 * p : p < 0.75 ? 2 - 4 * p : 4 * p - 4,
                                      p < 0.5 ? 2 * p : 2 * p - 2, p < 0.5 ? 1.0 : -1.0};
                o[i] = float(sh[s] * (1 - u) + sh[s + 1] * u);
            }
        } else if (name == "pwm") {
            const double w = 0.5 - 0.48 * t;
            for (int i = 0; i < n0; ++i)
                o[i] = (double(i) / n0 < w ? 1.0f : -1.0f) - float(2 * w - 1);  // DC removed
        } else if (name == "formant") {
            const double fc = std::exp2(5.0 * t);  // resonance at 1 .. 32 x f
            std::vector<double> amp(H, 0.0);
            for (int h = 1; h < H; ++h) {
                const double r = std::log2(double(h) / fc);
                amp[h] = (1.0 / h) * (0.15 + 4.0 * std::exp(-r * r * 8));
            }
            additive(o, n0, amp);
        } else {
            return Table{};
        }
    }
    float peak = 0;
    for (float v : x)
        peak = std::max(peak, std::fabs(v));
    if (peak > 0)
        for (float& v : x)
            v /= peak;
    return build(name, x, n0, nmin);
}

bool get(const std::string& spec, Table& t, int nmin) {
    if (spec.find('.') == std::string::npos && spec.find('/') == std::string::npos) {
        t = builtin(spec, 256, 2048, nmin);
        return t.frames > 0;
    }
    return load_clm_wav(spec, t, nmin);
}

int level_for(const Table& t, double f, double limit) {
    for (int k = 0; k < t.levels; ++k)
        if (double(t.harm(k)) * f <= limit)
            return k;
    return t.levels - 1;
}

}  // namespace wt
