// Bit-exact model of rtl/reverb_unit.v: the 2x2 PMWave plate in fixed point.
//
// 16 rails (4 junctions x {W->J, N->J, J->W, J->N}), each a 2048-sample ring of s18 (Q2.16). Per
// sample, for every rail: length D (Q11.20) -> delay d and fraction f (Q15) -> 4-point Lagrange
// taps (Q15) -> x = taps . ring[w-d .. w-d-3] -> loop filter y = gain x + pole y[n-1] (Q1.17
// coefficients). Junction pressures p = (sum of the 4 arriving waves) / 2, the input joins at
// junction 0 with 0.4, the outgoing waves p - arriving go into the rings at slot w-1. Output:
// (p[3] + p[1]) / 2 (the UGen's two pickups, mixed: the plate feeds the right output only).
//
// Inside one sample everything is s24 (Q2.22), rounded to nearest. Only the rings hold 18 bits,
// stored with stochastic rounding (a random 6-bit offset from a xorshift32 before dropping 6
// bits): unbiased, so the network's loop gain near 1 can't build a DC offset or hold a limit
// cycle, and it doesn't drain the tail either (rounding towards zero would, the tail sits a few
// bits above the LSB).
//
// Every 256 samples the next block's registers (written by the soft core, fw/reverb_ctl.c) are
// taken: gains, poles and each rail's target length, which it then reaches linearly.
#pragma once
#include <cstdint>

#include "../fw/reverb_ctl.h"

struct ReverbModel {
    static constexpr int NR = 16, LEN = 2048, MASK = LEN - 1;
    int32_t ram[NR][LEN]{};
    uint32_t len[NR];        // Q20
    int32_t inc[NR]{};       // Q20 per sample
    int32_t gain[NR]{}, pole[NR]{}, lp[NR]{}, wave[NR]{}, p[4]{}; // lp, wave, p: s24
    uint32_t rng = 0x2545F491u;  // stochastic rounding of the ring writes
    rv_regs next{};          // the soft core's writes, taken at the next block start
    int w = 0, cnt = 0;

    ReverbModel() {
        for (int r = 0; r < NR; ++r)
            len[r] = next.target[r] = 1024u << 20;
    }
    int32_t step(int32_t in); // in, out: s24 Q2.22
    static void taps(uint32_t D, int32_t t[4]); // the interpolation taps for a length
};
