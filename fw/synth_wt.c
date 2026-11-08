/* Wavetable mip levels, see synth_wt.h. */
#include "synth_wt.h"

#include "dmath.h"

static int32_t g_cq[513];   /* cos(2 pi i / 2048) for a quarter, Q30 */
static int32_t g_zbuf[2048]; /* the transform's work array (interleaved re, im) unless wt_set_work */
static int32_t* g_z = g_zbuf;
static int32_t g_xr[1025], g_xi[1025];

static double d_cosr(double x) { /* cos for |x| <= pi/2 */
    double t = 1, s = 1;
    for (int i = 1; i < 14; ++i) {
        t *= -x * x / ((2 * i - 1) * (2 * i));
        s += t;
    }
    return s;
}

void wt_init(void) {
    for (int i = 0; i <= 512; ++i)
        g_cq[i] = (int32_t)(d_cosr(3.14159265358979323846 * i / 1024.0) * 1073741824.0 + 0.5);
    g_cq[512] = 0;
}
void wt_set_work(int32_t* buf) { g_z = buf ? buf : g_zbuf; }
int32_t wt_cos_q30(int t) {
    t &= 2047;
    const int r = t & 511;
    switch (t >> 9) {
    case 0: return g_cq[r];
    case 1: return -g_cq[512 - r];
    case 2: return -g_cq[r];
    default: return g_cq[512 - r];
    }
}
static int32_t sin_q30(int t) { return wt_cos_q30(t - 512); }

int wt_level_size(int k) { return k <= 2 ? 2048 >> k : 256; }

void wt_layout(uint32_t base, int frames, uint32_t lvl[SY_LEVELS]) {
    lvl[0] = base;
    uint32_t a = base;
    for (int k = 1; k < SY_LEVELS; ++k) {
        lvl[k] = a;
        a += (uint32_t)frames * (uint32_t)wt_level_size(k);
    }
}

/* radix-2 complex FFT in place on z (interleaved); forward e^-i, inverse e^+i; scale: /2 per stage */
static void fft(int32_t* z, int m, int inv, int scale) {
    for (int i = 1, j = 0; i < m; ++i) {
        int bit = m >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            int32_t t = z[2 * i]; z[2 * i] = z[2 * j]; z[2 * j] = t;
            t = z[2 * i + 1]; z[2 * i + 1] = z[2 * j + 1]; z[2 * j + 1] = t;
        }
    }
    for (int len = 2; len <= m; len <<= 1) {
        const int half = len >> 1, step = 2048 / len;
        for (int j = 0; j < half; ++j) {
            const int32_t wr = wt_cos_q30(j * step), ws = sin_q30(j * step), wi = inv ? ws : -ws;
            for (int a = j; a < m; a += len) {
                int32_t* pa = z + 2 * a;
                int32_t* pb = pa + 2 * half;
                const int32_t br = pb[0], bi = pb[1], ar = pa[0], ai = pa[1];
                const int32_t tr = (int32_t)(((int64_t)br * wr - (int64_t)bi * wi) >> 30);
                const int32_t ti = (int32_t)(((int64_t)br * wi + (int64_t)bi * wr) >> 30);
                if (scale) {
                    pb[0] = (ar - tr) >> 1;
                    pb[1] = (ai - ti) >> 1;
                    pa[0] = (ar + tr) >> 1;
                    pa[1] = (ai + ti) >> 1;
                } else {
                    pb[0] = ar - tr;
                    pb[1] = ai - ti;
                    pa[0] = ar + tr;
                    pa[1] = ai + ti;
                }
            }
        }
    }
}

static int16_t sat16(int32_t v) { return (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v); }

void wt_build_frame(const int16_t* x, int16_t* const dst[SY_LEVELS]) {
    int32_t* z = g_z;
    /* forward: z[n] = x[2n] + i x[2n+1] (Q27), Z = FFT_1024(z) / 1024, then the real spectrum
       X[k] / 2048 for k = 0..1024 */
    for (int n = 0; n < 2048; ++n)
        z[n] = (int32_t)x[n] << 12;
    fft(z, 1024, 0, 1);
    for (int k = 0; k <= 1024; ++k) {
        const int a = k & 1023, b = (1024 - k) & 1023;
        const int32_t zr = z[2 * a], zi = z[2 * a + 1], cr = z[2 * b], ci = -z[2 * b + 1];
        const int32_t er = (zr + cr) / 4, ei = (zi + ci) / 4;
        const int32_t dr = (zr - cr) / 4, di = (zi - ci) / 4;
        const int32_t orr = di, oi = -dr; /* -i d */
        const int32_t c = wt_cos_q30(k), s = sin_q30(k);
        g_xr[k] = er + (int32_t)(((int64_t)c * orr + (int64_t)s * oi) >> 30);
        g_xi[k] = ei + (int32_t)(((int64_t)c * oi - (int64_t)s * orr) >> 30);
    }
    /* each level: the harmonics it keeps, inverse real FFT of its size through a half-size one */
    for (int lv = 1; lv < SY_LEVELS; ++lv) {
        const int N = wt_level_size(lv), M = N / 2, H = (1024 >> lv) - 1, st = 2048 / N;
        for (int k = 0; k < M; ++k) {
            const int32_t yr = k <= H ? g_xr[k] : 0, yi = k <= H ? g_xi[k] : 0;
            const int j = M - k;
            const int32_t cr = (j <= H && j < M) ? g_xr[j] : 0, ci = (j <= H && j < M) ? -g_xi[j] : 0;
            const int32_t er = yr + cr, ei = yi + ci, dr = yr - cr, di = yi - ci;
            const int32_t c = wt_cos_q30(k * st), s = sin_q30(k * st);
            const int32_t orr = (int32_t)(((int64_t)dr * c - (int64_t)di * s) >> 30);
            const int32_t oi = (int32_t)(((int64_t)dr * s + (int64_t)di * c) >> 30);
            z[2 * k] = er - oi;
            z[2 * k + 1] = ei + orr;
        }
        fft(z, M, 1, 0);
        int16_t* o = dst[lv];
        for (int n = 0; n < N; ++n) /* Q27 -> Q15, x 0.78 (z holds y[0], y[1], ... in order) */
            o[n] = sat16((int32_t)(((int64_t)z[n] * 51118) >> 28));
    }
}

/* ---- built-in tables --------------------------------------------------------------------- */
static const struct {
    const char* name;
    int frames;
} g_bi[] = {{"basic", 64}, {"saw", 1}, {"square", 1}, {"pwm", 32}, {"sine", 1}, {"fold", 32}};
#define NBI ((int)(sizeof(g_bi) / sizeof(g_bi[0])))

int wt_builtin_count(void) { return NBI; }
const char* wt_builtin_name(int t) { return g_bi[t].name; }
int wt_builtin_frames(int t) { return g_bi[t].frames; }

/* the four basic shapes at sample i (Q15): sine, triangle, saw (rising from 0), square */
static int32_t shape(int s, int i) {
    switch (s) {
    case 0: return sin_q30(i) >> 15;
    case 1: return i < 512 ? i * 64 : i < 1536 ? 65536 - i * 64 : i * 64 - 131072;
    case 2: return i < 1024 ? i * 32 : i * 32 - 65536;
    default: return i < 1024 ? 32767 : -32767;
    }
}

void wt_builtin_frame(int t, int f, int16_t* x) {
    const int F = g_bi[t].frames;
    const int32_t u16 = F > 1 ? (int32_t)((int64_t)f * 65536 / (F - 1)) : 0; /* 0..1 over the table */
    for (int i = 0; i < WT_N0; ++i) {
        int32_t v;
        switch (t) {
        case 0: { /* sine -> triangle -> saw -> square, crossfaded in thirds */
            const int32_t seg = 3 * u16;
            const int s = seg >> 16 > 2 ? 2 : seg >> 16;
            const int32_t u = seg - (s << 16);
            const int32_t a = shape(s, i), b = shape(s + 1, i);
            v = a + (int32_t)(((int64_t)(b - a) * u) >> 16);
            break;
        }
        case 1: v = 32767 - (i * 65535) / WT_N0; break; /* falling saw */
        case 2: v = shape(3, i); break;
        case 3: { /* pulse width 50 % -> 2 %, DC removed, scaled by the widest excursion (1.96) */
            const int32_t w = 32768 - (int32_t)(((int64_t)u16 * 31457) >> 16); /* Q16 width */
            const int32_t hi = i * 32 < w ? 65536 : -65536;                     /* phase Q16 = i * 32 */
            v = (int32_t)(((int64_t)(hi - (2 * w - 65536)) * 16718) >> 16);     /* x 32767 / 1.96 / 65536 */
            break;
        }
        case 4: v = shape(0, i); break;
        default: { /* sine folded back at +-1 with a gain of 1 .. 5 */
            int32_t y = (int32_t)(((int64_t)shape(0, i) * (65536 + 4 * u16)) >> 16);
            for (int k = 0; k < 8 && (y > 32767 || y < -32767); ++k)
                y = y > 32767 ? 65534 - y : -65534 - y;
            v = y;
            break;
        }
        }
        x[i] = sat16(v);
    }
}
