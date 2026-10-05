// Host check of fw/wavutil.c: wav_convert against a double-precision reference (synthetic
// edge cases + the first and a middle stretch of real files) and the split plan per file.
//
//   g++ -O2 -std=c++20 -Ifw host/test_wavutil.cpp fw/wavutil.c -o test_wavutil
//   ./test_wavutil [file.wav ...]
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

extern "C" {
#include "wavutil.h"
}

static const uint32_t kMaxFrames = (7u << 20) / 2 - 64 - 128 - 64; // as fw/samples.c

static double chan_ref(const uint8_t* p, int fmt, int bits) {
    if (fmt == WAV_FLOAT) {
        if (bits == 32) {
            float f;
            std::memcpy(&f, p, 4);
            return f * 0.5;
        }
        double d;
        std::memcpy(&d, p, 8);
        return d * 0.5;
    }
    switch (bits) {
    case 8: return (p[0] - 128) / 128.0;
    case 16: return int16_t(p[0] | p[1] << 8) / 32768.0;
    case 24: return int32_t(uint32_t(p[0]) << 8 | uint32_t(p[1]) << 16 | uint32_t(p[2]) << 24) / 2147483648.0;
    default: return int32_t(uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24) / 2147483648.0;
    }
}

// max |wav_convert - reference| in LSB over n frames
static int check(const uint8_t* src, uint32_t n, int fmt, int ch, int bits, int32_t g, const char* what) {
    std::vector<int16_t> out(n);
    wav_convert(out.data(), src, n, fmt, ch, bits, g);
    const int fb = ch * bits / 8;
    int worst = 0;
    uint32_t at = 0;
    for (uint32_t i = 0; i < n; ++i) {
        double acc = 0;
        for (int c = 0; c < ch; ++c) {
            double v = chan_ref(src + i * fb + c * (bits / 8), fmt, bits);
            if (std::isnan(v) || v > 16.0)
                v = 16.0;
            if (v < -16.0)
                v = -16.0;
            acc += v;
        }
        acc = acc / ch * 32768.0 * g / 65536.0;
        const double r = std::fmax(-32768.0, std::fmin(32767.0, std::nearbyint(acc)));
        const int d = int(std::fabs(r - out[i]));
        if (d > worst) {
            worst = d;
            at = i;
        }
    }
    std::printf("  %-34s fmt %d ch %d %2d-bit gain %.3f: max err %d LSB%s\n", what, fmt, ch, bits, g / 65536.0, worst,
                worst > 1 ? "   <-- FAIL" : "");
    (void)at;
    return worst <= 1;
}

template <class T> static std::vector<uint8_t> bytes(const std::vector<T>& v) {
    std::vector<uint8_t> b(v.size() * sizeof(T));
    std::memcpy(b.data(), v.data(), b.size());
    return b;
}

static int synthetic() {
    int ok = 1;
    std::vector<float> f;
    for (int i = 0; i < 20000; ++i)
        f.push_back(float(std::sin(i * 0.0137) * (0.2 + i / 8000.0))); // up to +2.7: clips at 2.0
    for (float v : {0.0f, -0.0f, 1e-30f, -1e-38f, 1.0f, -1.0f, 2.0f, -2.0f, 1.9999f, 15.9f, -40.0f, 1e30f, INFINITY,
                    -INFINITY, 0.49999f, 3.0517578e-05f})
        f.push_back(v);
    if (f.size() & 1)
        f.push_back(0.25f);
    std::vector<double> d(f.begin(), f.end());
    std::vector<int16_t> s16;
    std::vector<int32_t> s32;
    std::vector<uint8_t> s24, s8;
    for (float v : f) {
        const double c = std::fmax(-1.0, std::fmin(0.99999, v * 0.5));
        s16.push_back(int16_t(std::lrint(c * 32767)));
        const int32_t x = int32_t(std::lrint(c * 2147483647.0));
        s32.push_back(x);
        s24.push_back(uint8_t(x >> 8));
        s24.push_back(uint8_t(x >> 16));
        s24.push_back(uint8_t(x >> 24));
        s8.push_back(uint8_t(std::lrint(c * 127) + 128));
    }
    const uint32_t n1 = uint32_t(f.size());
    for (int32_t g : {65536, 65536 * 3 / 2, 65536 * 7}) {
        ok &= check(bytes(f).data(), n1, WAV_FLOAT, 1, 32, g, "float32 mono");
        ok &= check(bytes(f).data(), n1 / 2, WAV_FLOAT, 2, 32, g, "float32 stereo");
        ok &= check(bytes(d).data(), n1, WAV_FLOAT, 1, 64, g, "float64 mono");
        ok &= check(bytes(d).data(), n1 / 4, WAV_FLOAT, 4, 64, g, "float64 4ch");
        ok &= check(bytes(s16).data(), n1, WAV_PCM, 1, 16, g, "pcm16 mono");
        ok &= check(bytes(s16).data(), n1 / 2, WAV_PCM, 2, 16, g, "pcm16 stereo");
        ok &= check(s24.data(), n1, WAV_PCM, 1, 24, g, "pcm24 mono");
        ok &= check(s24.data(), n1 / 2, WAV_PCM, 2, 24, g, "pcm24 stereo");
        ok &= check(s24.data(), n1 / 6, WAV_PCM, 6, 24, g, "pcm24 6ch");
        ok &= check(bytes(s32).data(), n1 / 2, WAV_PCM, 2, 32, g, "pcm32 stereo");
        ok &= check(s8.data(), n1, WAV_PCM, 1, 8, g, "pcm8 mono");
    }
    const struct { const char* n; uint32_t bpm; } names[] = {
        {"loop_174bpm.wav", 174000}, {"Amen 136 BPM.wav", 136000}, {"bpm90_pad.wav", 90000},
        {"x_120_bpm.wav", 120000},   {"0014 38-Audio.wav", 0},     {"bpm.wav", 0},
        {"song 12bpm.wav", 0},       {"a1128bpm.wav", 128000},     {"BPM-95 keys.wav", 95000}};
    for (auto& t : names) {
        const uint32_t b = wav_name_bpm_milli(t.n);
        if (b != t.bpm) {
            std::printf("  name bpm %s: %u, expected %u   <-- FAIL\n", t.n, b, t.bpm);
            ok = 0;
        }
    }
    return ok;
}

static uint32_t rd32(const uint8_t* p) { return p[0] | p[1] << 8 | p[2] << 16 | uint32_t(p[3]) << 24; }

static int real_file(const char* path) {
    FILE* fp = std::fopen(path, "rb");
    if (!fp)
        return 1;
    uint8_t h[12];
    if (std::fread(h, 1, 12, fp) != 12 || std::memcmp(h, "RIFF", 4)) {
        std::fclose(fp);
        return 1;
    }
    int fmt = 0, ch = 0, bits = 0;
    uint32_t sr = 0, len = 0;
    long off = 0;
    for (;;) {
        uint8_t c[8];
        if (std::fread(c, 1, 8, fp) != 8)
            break;
        const uint32_t l = rd32(c + 4);
        if (!std::memcmp(c, "fmt ", 4)) {
            uint8_t b[40] = {};
            std::fread(b, 1, l < 40 ? l : 40, fp);
            fmt = b[0] | b[1] << 8;
            ch = b[2] | b[3] << 8;
            sr = rd32(b + 4);
            bits = b[14] | b[15] << 8;
            if (fmt == 0xFFFE && l >= 26)
                fmt = b[24] | b[25] << 8;
            std::fseek(fp, long(l) - (l < 40 ? long(l) : 40) + (l & 1), SEEK_CUR);
        } else if (!std::memcmp(c, "data", 4)) {
            off = std::ftell(fp);
            len = l;
            break;
        } else
            std::fseek(fp, long(l + (l & 1)), SEEK_CUR);
    }
    const char* base = std::strrchr(path, '/') ? std::strrchr(path, '/') + 1 : path;
    if (!wav_supported(fmt, ch, bits)) {
        std::printf("%s: format %d, %d ch, %d bit: not supported\n", base, fmt, ch, bits);
        std::fclose(fp);
        return 1;
    }
    const uint32_t fb = uint32_t(ch * bits / 8), frames = len / fb;
    const uint32_t part = wav_split_len(frames, sr, base, kMaxFrames);
    std::printf("%s: fmt %d %d ch %d bit %u Hz, %.1f s", base, fmt, ch, bits, sr, double(frames) / sr);
    if (part) {
        const uint32_t n = wav_split_count(frames, part);
        std::printf(" -> %u parts of %.2f s (last starts %.2f s)\n", n, double(part) / sr,
                    double(wav_split_start(frames, part, n - 1)) / sr);
    } else
        std::printf(" (whole)\n");
    int ok = 1;
    for (int pass = 0; pass < 2; ++pass) {
        const uint32_t start = pass ? frames / 2 : 0, n = frames - start < 200000 ? frames - start : 200000;
        std::vector<uint8_t> buf(size_t(n) * fb);
        std::fseek(fp, off + long(start) * fb, SEEK_SET);
        if (std::fread(buf.data(), 1, buf.size(), fp) != buf.size())
            break;
        ok &= check(buf.data(), n, fmt, ch, bits, 65536 * 5 / 4, pass ? "  middle" : "  start");
    }
    std::fclose(fp);
    return ok;
}

int main(int argc, char** argv) {
    int ok = synthetic();
    for (int a = 1; a < argc; ++a)
        ok &= real_file(argv[a]);
    std::printf(ok ? "ALL PASS\n" : "FAILURES\n");
    return ok ? 0 : 1;
}
