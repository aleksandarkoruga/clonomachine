#include "wavutil.h"

#include <string.h>

int wav_supported(int fmt, int ch, int bits) {
    if (ch < 1 || ch > WAV_MAX_CH)
        return 0;
    if (fmt == WAV_PCM)
        return bits == 8 || bits == 16 || bits == 24 || bits == 32;
    if (fmt == WAV_FLOAT)
        return bits == 32 || bits == 64;
    return 0;
}

#define Q_CLAMP ((1 << 27) - 1) /* +-16 full scale per channel: 8 channels still fit int32 */

/* One channel sample in Q23 (full scale 2^23). Floats at half scale: x * 2^22. The
   VexRiscv has no FPU, so floats are taken apart: x = M * 2^(e - bias - mantissa bits). */
static inline __attribute__((always_inline)) int32_t chan_q23(const uint8_t* p, int bps, int fl) {
    if (bps == 1)
        return ((int32_t)p[0] - 128) * 65536;
    if (bps == 2)
        return (int32_t)(int16_t)(p[0] | (p[1] << 8)) * 256;
    if (bps == 3)
        return (int32_t)((uint32_t)p[0] << 8 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 24) >> 8;
    if (bps == 4 && !fl)
        return (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24) >> 8;
    uint32_t u, m;
    int e, sh, maxsh;
    if (bps == 4) { /* float32: M < 2^24, x * 2^22 = M * 2^(e - 128) */
        u = (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
        e = (int)((u >> 23) & 255);
        m = (u & 0x7FFFFFu) | 0x800000u;
        sh = e - 128;
        maxsh = 3;
    } else { /* float64, high word: M < 2^21, x * 2^22 = M * 2^(e - 1021) */
        u = (uint32_t)p[4] | (uint32_t)p[5] << 8 | (uint32_t)p[6] << 16 | (uint32_t)p[7] << 24;
        e = (int)((u >> 20) & 2047);
        m = (u & 0xFFFFFu) | 0x100000u;
        sh = e - 1021;
        maxsh = 6;
    }
    int32_t v;
    if (e == 0 || sh <= -32) /* zero, denormal, below 2^-23 */
        v = 0;
    else if (sh > maxsh) /* also inf / NaN */
        v = Q_CLAMP;
    else if (sh >= 0)
        v = (int32_t)(m << sh);
    else
        v = (int32_t)(m >> -sh);
    return (u >> 31) ? -v : v;
}

static inline int16_t sat16(int64_t v) { return (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v); }

static inline __attribute__((always_inline)) void cvt(int16_t* d, const uint8_t* p, uint32_t n, int ch, int bps,
                                                      int fl, int32_t g) {
    const int32_t inv = (1 << 24) / ch;
    for (uint32_t i = 0; i < n; ++i) {
        int32_t acc = 0;
        for (int c = 0; c < ch; ++c, p += bps)
            acc += chan_q23(p, bps, fl);
        if (ch == 2)
            acc >>= 1;
        else if (ch > 2)
            acc = (int32_t)(((int64_t)acc * inv + (1 << 23)) >> 24);
        d[i] = sat16(((int64_t)acc * g + (1 << 23)) >> 24); /* Q23 * Q16 -> int16 */
    }
}

#define CVT(bps, fl)                                                                                                   \
    do {                                                                                                               \
        if (ch == 1)                                                                                                   \
            cvt(dst, src, n, 1, bps, fl, g);                                                                           \
        else if (ch == 2)                                                                                              \
            cvt(dst, src, n, 2, bps, fl, g);                                                                           \
        else                                                                                                           \
            cvt(dst, src, n, ch, bps, fl, g);                                                                          \
    } while (0)

void wav_convert(int16_t* dst, const uint8_t* src, uint32_t n, int fmt, int ch, int bits, int32_t g) {
    const int fl = fmt == WAV_FLOAT;
    if (!fl && bits == 16 && ch == 1 && g == 65536) { /* native format: little-endian int16 */
        memcpy(dst, src, n * 2);
        return;
    }
    if (!fl && bits == 16 && ch == 1) {
        const int16_t* s = (const int16_t*)src;
        for (uint32_t i = 0; i < n; ++i)
            dst[i] = sat16(((int64_t)s[i] * g + 32768) >> 16);
        return;
    }
    if (!fl && bits == 16 && ch == 2) {
        const uint32_t* s = (const uint32_t*)src; /* L | R << 16 */
        for (uint32_t i = 0; i < n; ++i) {
            const uint32_t w = s[i];
            const int32_t m = (int32_t)(int16_t)w + (int32_t)(int16_t)(w >> 16);
            dst[i] = sat16(((int64_t)m * g + 65536) >> 17);
        }
        return;
    }
    switch (fl ? -bits : bits) {
    case 8: CVT(1, 0); break;
    case 16: CVT(2, 0); break;
    case 24: CVT(3, 0); break;
    case 32: CVT(4, 0); break;
    case -32: CVT(4, 1); break;
    case -64: CVT(8, 1); break;
    default: memset(dst, 0, n * 2); break;
    }
}

static int is_sep(char c) { return c == ' ' || c == '_' || c == '-'; }

uint32_t wav_name_bpm_milli(const char* name) {
    for (const char* p = name; *p; ++p) {
        if ((p[0] | 32) != 'b' || (p[1] | 32) != 'p' || (p[2] | 32) != 'm')
            continue;
        const char* e = p; /* "<n>bpm", "<n> bpm", "<n>_bpm" */
        if (e > name && is_sep(e[-1]))
            --e;
        const char* q = e;
        while (q > name && q[-1] >= '0' && q[-1] <= '9' && e - q < 3)
            --q;
        uint32_t v = 0;
        for (const char* r = q; r < e; ++r)
            v = v * 10 + (uint32_t)(*r - '0');
        if (q < e && v >= 40 && v <= 300)
            return v * 1000;
        const char* r = p + 3; /* "bpm<n>", "bpm_<n>" */
        if (is_sep(*r))
            ++r;
        int nd = 0;
        for (v = 0; *r >= '0' && *r <= '9' && nd < 3; ++r, ++nd)
            v = v * 10 + (uint32_t)(*r - '0');
        if (nd && v >= 40 && v <= 300)
            return v * 1000;
    }
    return 0;
}

uint32_t wav_split_len(uint32_t frames, uint32_t sr, const char* name, uint32_t max_frames) {
    if ((uint64_t)frames <= (uint64_t)sr * WAV_SPLIT_SECONDS && frames <= max_frames)
        return 0;
    uint32_t bpm = wav_name_bpm_milli(name);
    if (!bpm)
        bpm = WAV_PART_BPM * 1000;
    uint64_t len = (uint64_t)sr * 60000u * WAV_PART_BEATS / bpm;
    if (len > max_frames)
        len = max_frames;
    len &= ~(uint64_t)63;
    if (len < 4096)
        len = 4096;
    return len < frames ? (uint32_t)len : 0;
}

uint32_t wav_split_count(uint32_t frames, uint32_t len) {
    uint32_t n = frames / len;
    if (frames - n * len >= len / 4)
        ++n;
    return n ? n : 1;
}

uint32_t wav_split_start(uint32_t frames, uint32_t len, uint32_t k) {
    uint64_t s = (uint64_t)k * len;
    if (s + len > frames)
        s = (frames - len) & ~3u;
    return (uint32_t)s;
}
