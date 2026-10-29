// Bit-exact model of rtl/reverb_unit.v, see reverb_model.h. Shifts of negative values are
// arithmetic (floor), as Verilog's >>> on signed values.
#include "reverb_model.h"

namespace {
inline int32_t sat18(int64_t v) { return v > 131071 ? 131071 : v < -131072 ? -131072 : (int32_t)v; }
inline int32_t sat24(int64_t v) { return v > 8388607 ? 8388607 : v < -8388608 ? -8388608 : (int32_t)v; }
inline int64_t rnd(int64_t v, int n) { return (v + ((int64_t)1 << (n - 1))) >> n; } /* to nearest */
}  // namespace

// 4-point Lagrange through delays d-1 .. d+2 (PMWave's _updateLengths), f in Q15:
//   a = f(f-1), c = (f+1)(f-2), t0 = -a(f-2)/6, t1 = c(f-1)/2   (t2 t3: the mirror images)
int32_t ReverbModel::tap_table(int which, int k) {
    const int64_t f = (int64_t)k << 5, fm1 = f - 32768, fm2 = f - 65536;
    if (which == 0) {
        const int64_t a = (f * fm1) >> 15;
        return (int32_t)((((a * fm2) >> 15) * -21845) >> 17);
    }
    const int64_t c = ((f + 32768) * fm2) >> 15;
    return (int32_t)(((c * fm1) >> 15) >> 1);
}

void ReverbModel::taps(uint32_t D, int32_t t[4]) {
    static int32_t T[2][1024];
    static bool built = false;
    if (!built) {
        for (int k = 0; k < 1024; ++k)
            T[0][k] = tap_table(0, k), T[1][k] = tap_table(1, k);
        built = true;
    }
    const int k = (int)((D >> 10) & 1023);
    t[0] = T[0][k];
    t[1] = T[1][k];
    t[2] = k ? T[1][1024 - k] : 0;
    t[3] = k ? T[0][1024 - k] : 0;
}

int32_t ReverbModel::step(int32_t in) {
    if (cnt == 0) { /* block start: the soft core's registers */
        for (int r = 0; r < NR; ++r) {
            gain[r] = next.gain[r];
            pole[r] = next.pole[r];
            inc[r] = (int32_t)(((int64_t)next.target[r] - (int64_t)len[r]) >> RV_BLOCK_LOG2);
        }
    }
    for (int r = 0; r < NR; ++r) {
        const uint32_t D = len[r];
        const int d = (int)(D >> 20);
        int32_t t[4];
        taps(D, t);
        const int s0 = w - d;
        int64_t acc = 0; /* Q15 x Q2.16 */
        for (int k = 0; k < 4; ++k)
            acc += (int64_t)t[k] * ram[r][(s0 - k) & MASK];
        const int32_t x = sat24(rnd(acc, 9)); /* Q2.22 */
        const int32_t y = sat24(rnd((int64_t)gain[r] * x + (int64_t)pole[r] * lp[r], 17));
        lp[r] = wave[r] = y;
    }
    /* junctions J = i*2 + j: E = east, S = south (on a 2x2 torus also west and north) */
    for (int J = 0; J < 4; ++J) {
        const int E = J ^ 1, S = J ^ 2;
        p[J] = sat24(rnd((int64_t)wave[J * 4 + 0] + wave[J * 4 + 1] + wave[E * 4 + 2] + wave[S * 4 + 3], 1));
    }
    p[0] = sat24(p[0] + rnd((int64_t)in * 52429, 17)); /* 0.4 in */
    const int slot = (w - 1) & MASK;
    for (int J = 0; J < 4; ++J) {
        const int W = J ^ 1, N = J ^ 2;
        const int64_t out[4] = {(int64_t)p[W] - wave[J * 4 + 2], (int64_t)p[N] - wave[J * 4 + 3],
                                (int64_t)p[J] - wave[J * 4 + 0], (int64_t)p[J] - wave[J * 4 + 1]};
        for (int k = 0; k < 4; ++k) { /* stochastic rounding 24 -> 18 bits */
            rng ^= rng << 13;
            rng ^= rng >> 17;
            rng ^= rng << 5;
            ram[J * 4 + k][slot] = sat18((out[k] + (rng >> 26)) >> 6);
        }
    }
    w = (w + 1) & MASK;
    for (int r = 0; r < NR; ++r)
        len[r] += (uint32_t)inc[r];
    cnt = (cnt + 1) & ((1 << RV_BLOCK_LOG2) - 1);
    return sat24(rnd((int64_t)p[3] + p[1], 1));
}
