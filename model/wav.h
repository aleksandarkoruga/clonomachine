// Minimal WAV reader/writer for the offline model tools (PCM 16/24/32, float32; mono mixdown).
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace wav {

struct Audio {
    int rate = 48000;
    std::vector<float> x; // mono, full scale +-1
};

inline bool read(const std::string& path, Audio& a) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f)
        return false;
    std::vector<uint8_t> d;
    uint8_t buf[65536];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0)
        d.insert(d.end(), buf, buf + n);
    std::fclose(f);
    if (d.size() < 12 || std::memcmp(&d[0], "RIFF", 4) || std::memcmp(&d[8], "WAVE", 4))
        return false;
    auto u16 = [&](size_t o) { return uint32_t(d[o] | d[o + 1] << 8); };
    auto u32 = [&](size_t o) { return uint32_t(d[o] | d[o + 1] << 8 | d[o + 2] << 16 | uint32_t(d[o + 3]) << 24); };
    int fmt = 0, ch = 0, bits = 0;
    for (size_t o = 12; o + 8 <= d.size();) {
        const uint32_t len = u32(o + 4);
        if (!std::memcmp(&d[o], "fmt ", 4)) {
            fmt = u16(o + 8);
            ch = u16(o + 10);
            a.rate = int(u32(o + 12));
            bits = u16(o + 22);
            if (fmt == 0xFFFE)
                fmt = u16(o + 32);
        } else if (!std::memcmp(&d[o], "data", 4) && ch) {
            const size_t bps = bits / 8, frames = std::min<size_t>(len, d.size() - o - 8) / (bps * ch);
            a.x.resize(frames);
            const uint8_t* p = &d[o + 8];
            for (size_t i = 0; i < frames; ++i) {
                double s = 0;
                for (int c = 0; c < ch; ++c, p += bps) {
                    if (fmt == 3 && bits == 32) {
                        float v;
                        std::memcpy(&v, p, 4);
                        s += v;
                    } else if (bits == 16) {
                        s += int16_t(p[0] | p[1] << 8) / 32768.0;
                    } else if (bits == 24) {
                        s += (int32_t(uint32_t(p[0] << 8 | p[1] << 16 | uint32_t(p[2]) << 24)) >> 8) / 8388608.0;
                    } else if (bits == 32) {
                        s += int32_t(p[0] | p[1] << 8 | p[2] << 16 | uint32_t(p[3]) << 24) / 2147483648.0;
                    } else if (bits == 8) {
                        s += (p[0] - 128) / 128.0;
                    }
                }
                a.x[i] = float(s / ch);
            }
            return true;
        }
        o += 8 + len + (len & 1);
    }
    return false;
}

inline bool write(const std::string& path, const std::vector<float>& x, int rate) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f)
        return false;
    const uint32_t bytes = uint32_t(x.size() * 2);
    auto w32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
    auto w16 = [&](uint16_t v) { std::fwrite(&v, 2, 1, f); };
    std::fwrite("RIFF", 1, 4, f);
    w32(36 + bytes);
    std::fwrite("WAVEfmt ", 1, 8, f);
    w32(16);
    w16(1);
    w16(1);
    w32(uint32_t(rate));
    w32(uint32_t(rate) * 2);
    w16(2);
    w16(16);
    std::fwrite("data", 1, 4, f);
    w32(bytes);
    for (float v : x) {
        const double s = v * 32767.0;
        const int16_t q = int16_t(s > 32767 ? 32767 : s < -32768 ? -32768 : (s < 0 ? s - 0.5 : s + 0.5));
        w16(uint16_t(q));
    }
    std::fclose(f);
    return true;
}

} // namespace wav
