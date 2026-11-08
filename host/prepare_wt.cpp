// Wavetables for the synth, prepared on the PC: WAV wavetables (frames from the 'clm ' chunk, else
// 2048 samples) and the built-in shapes become the band-limited mip levels the oscillators play
// (model/wt_tables.cpp: levels 1..9, 1024 / 512 / 256 samples per frame, s16, normalised), one
// .wtl file each. The board copies them into the top 8 MB of its SDRAM in name order (fw/synth.c);
// the file's number is the table index (aTable / bTable).
//
//   build/host/prepare_wt [--out DIR] [--builtins] [--clean] [inputs...]    (host/prepare_wt.sh)
//
// inputs: WAV files or folders (their *.wav, sorted). Frames of another size are resampled to 2048;
// more than 256 frames: 256 of them, evenly spaced. Stops at the 8 MB the board has for tables.
//
// .wtl: "SYWT", u32 version (1), u32 frames, u32 words (frames x 3328), char name[32], then the s16
// words: level 1 (all frames), level 2, ... level 9.
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "../model/wt_tables.h"

namespace fs = std::filesystem;

namespace {
constexpr int kN0 = 2048, kMaxFrames = 256;
constexpr uint32_t kFrameWords = 3328, kBudgetWords = (8u << 20) / 2;

// one frame of n samples -> 2048 (spectrum zero-padded or truncated)
std::vector<float> resample_frame(const float* x, int n) {
    if (n == kN0)
        return std::vector<float>(x, x + n);
    std::vector<std::complex<double>> X(n), Y(kN0);
    for (int i = 0; i < n; ++i)
        X[i] = x[i];
    wt::fft(X, false);
    const int h = std::min(n, kN0) / 2;
    for (int k = 0; k < h; ++k) {
        Y[k] = X[k];
        if (k)
            Y[kN0 - k] = X[n - k];
    }
    wt::fft(Y, true);
    std::vector<float> y(kN0);
    for (int i = 0; i < kN0; ++i)
        y[i] = float(Y[i].real() / n);
    return y;
}

bool write_wtl(const fs::path& path, const std::string& name, const wt::Table& t) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f)
        return false;
    const uint32_t hdr[4] = {0x54575953u /* "SYWT" */, 1, uint32_t(t.frames), uint32_t(t.frames) * kFrameWords};
    std::fwrite(hdr, 4, 4, f);
    char nm[32] = {};
    std::snprintf(nm, sizeof nm, "%s", name.c_str());
    std::fwrite(nm, 1, 32, f);
    uint32_t words = 0;
    for (int k = 1; k <= 9; ++k) {
        std::fwrite(t.q[k].data(), 2, t.q[k].size(), f);
        words += uint32_t(t.q[k].size());
    }
    std::fclose(f);
    return words == hdr[3];
}

// the built-in shapes (the board's fallback has a few of them too: fw/synth_wt.c)
std::vector<float> builtin_frames(const std::string& name, int frames) {
    std::vector<float> x(size_t(frames) * kN0);
    for (int f = 0; f < frames; ++f) {
        const double u = frames > 1 ? double(f) / (frames - 1) : 0.0;
        for (int i = 0; i < kN0; ++i) {
            const double p = double(i) / kN0, s = std::sin(2 * M_PI * p);
            double v = 0;
            if (name == "square")
                v = p < 0.5 ? 1 : -1;
            else if (name == "fold") {  // a sine folded back at +-1, gain 1 .. 5
                v = s * (1 + 4 * u);
                for (int k = 0; k < 8 && std::fabs(v) > 1; ++k)
                    v = v > 1 ? 2 - v : -2 - v;
            }
            x[size_t(f) * kN0 + i] = float(v);
        }
    }
    return x;
}
}  // namespace

int main(int argc, char** argv) {
    fs::path out = ".";
    bool builtins = false, clean = false;
    std::vector<fs::path> inputs;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--out" && i + 1 < argc)
            out = argv[++i];
        else if (a == "--builtins")
            builtins = true;
        else if (a == "--clean")
            clean = true;
        else if (a == "-h" || a == "--help") {
            std::printf("usage: %s [--out DIR] [--builtins] [--clean] [WAV files or folders...]\n", argv[0]);
            return 0;
        } else
            inputs.push_back(a);
    }
    fs::create_directories(out);
    if (clean)
        for (auto& e : fs::directory_iterator(out))
            if (e.is_regular_file() && e.path().extension() == ".wtl")
                fs::remove(e.path());
    struct Job {
        std::string name;
        wt::Table t;
    };
    std::vector<Job> jobs;
    if (builtins) {
        const std::pair<const char*, int> bi[] = {{"basic", 64}, {"saw", 1}, {"square", 1}, {"pwm", 32},
                                                  {"sine", 1},   {"formant", 32}, {"fold", 32}};
        for (auto& [n, fr] : bi) {
            wt::Table t = (std::string(n) == "square" || std::string(n) == "fold")
                              ? wt::build(n, builtin_frames(n, fr), kN0, 256)
                              : wt::builtin(n, fr, kN0, 256);
            jobs.push_back({n, t});
        }
    }
    std::vector<fs::path> files;
    for (auto& in : inputs) {
        if (fs::is_directory(in)) {
            std::vector<fs::path> v;
            for (auto& e : fs::directory_iterator(in))
                if (e.is_regular_file()) {
                    std::string ext = e.path().extension().string();
                    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                    if (ext == ".wav")
                        v.push_back(e.path());
                }
            std::sort(v.begin(), v.end());
            files.insert(files.end(), v.begin(), v.end());
        } else
            files.push_back(in);
    }
    for (auto& p : files) {
        wt::Table src;
        if (!wt::load_clm_wav(p.string(), src) || src.frames < 1) {
            std::printf("skipped %s (not a wavetable WAV)\n", p.c_str());
            continue;
        }
        // the frames at 2048 samples, at most 256 of them
        const int n0 = src.n0, nf = std::min(src.frames, kMaxFrames);
        std::vector<float> frames;
        for (int j = 0; j < nf; ++j) {
            const int f = src.frames <= kMaxFrames ? j : int(std::lround(double(j) * (src.frames - 1) / (nf - 1)));
            std::vector<float> one(n0);
            for (int i = 0; i < n0; ++i)
                one[i] = src.at(0, f, i);
            const auto r = resample_frame(one.data(), n0);
            frames.insert(frames.end(), r.begin(), r.end());
        }
        float peak = 0;
        for (float v : frames)
            peak = std::max(peak, std::fabs(v));
        if (peak > 0)
            for (float& v : frames)
                v /= peak;
        jobs.push_back({p.stem().string(), wt::build(p.stem().string(), frames, kN0, 256)});
    }
    uint32_t used = 0;
    int idx = 0, written = 0;
    for (auto& j : jobs) {
        const uint32_t w = uint32_t(j.t.frames) * kFrameWords;
        if (used + w > kBudgetWords) {
            std::printf("%-32s %3d frames: no room left in the 8 MB (%u KB used), skipped\n", j.name.c_str(),
                        j.t.frames, used / 512);
            continue;
        }
        char fn[96];
        std::snprintf(fn, sizeof fn, "%02d_%s.wtl", idx, j.name.c_str());
        for (char* c = fn; *c; ++c)  // FAT-safe
            if (std::strchr("\\/:*?\"<>|", *c))
                *c = '_';
        if (!write_wtl(out / fn, j.name, j.t)) {
            std::printf("could not write %s\n", (out / fn).c_str());
            return 1;
        }
        std::printf("%2d %-32s %3d frames %5u KB\n", idx, j.name.c_str(), j.t.frames, w / 512);
        used += w;
        ++idx;
        ++written;
    }
    std::printf("%d tables, %u KB of 8192\n", written, used / 512);
    return 0;
}
