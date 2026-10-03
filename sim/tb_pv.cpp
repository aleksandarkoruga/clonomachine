// Verilator testbench: rtl/pv_unit.v vs model/pv_model.cpp, job by job, bit exact.
//
// Generates the job sequence the engine would issue (restart R0/R1/R2, steady grains,
// emission clearing ring entries as samples are played), runs every job on the RTL and
// the model, and compares both slots' rings after each job.
#include "Vpv_unit.h"
#include "Vpv_unit___024root.h"
#include "verilated.h"
#include "../model/pv_model.h"
#include "../model/wav.h"
#include "../include/sampler_regs.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

static Vpv_unit* top;
static uint64_t cycles = 0;
static std::vector<int16_t> mem;
static int memLatency = 40;

// SDRAM line server: in-order, `memLatency` cycles, then one word per cycle
struct Req { uint32_t addr; uint64_t due; };
static std::deque<Req> reqs;
static int wordIdx = 0;

static void tick() {
    // drive fill before the edge
    top->fill_valid = 0;
    if (!reqs.empty() && reqs.front().due <= cycles) {
        top->fill_valid = 1;
        top->fill_data = uint16_t(mem[(reqs.front().addr + wordIdx) % mem.size()]);
    }
    top->req_ready = reqs.size() < 8;
    top->eval();
    const bool req = top->req_valid && top->req_ready;
    const uint32_t raddr = top->req_addr;
    const bool fill = top->fill_valid;
    top->clk = 1;
    top->eval();
    top->clk = 0;
    top->eval();
    ++cycles;
    if (fill && ++wordIdx == 64) {
        wordIdx = 0;
        reqs.pop_front();
    }
    if (req)
        reqs.push_back({raddr, cycles + memLatency});
}

struct SlotSim {
    smp::PvSlotState st;
    std::vector<int32_t> ring = std::vector<int32_t>(smp::PV_RING, 0);
    uint32_t base = 0, frames = 0;
};

static smp::PvDebug dbg;
static int stageFails = 0;

static int64_t sx(uint64_t v, int bits) { return int64_t(v << (64 - bits)) >> (64 - bits); }
static uint32_t rev11(uint32_t v) { uint32_t r = 0; for (int i = 0; i < 11; ++i) r |= ((v >> i) & 1) << (10 - i); return r; }
static void elem(uint32_t i, int64_t& re, int64_t& im) {
    auto* r = top->rootp;
    const uint32_t p = __builtin_parity(i);
    const uint64_t w = p ? r->pv_unit__DOT__bank1[i >> 1] : r->pv_unit__DOT__bank0[i >> 1];
    re = sx(w >> 27, 27);
    im = sx(w & ((1ull << 27) - 1), 27);
}
static void check(const char* what, int64_t rtl, int64_t model, int idx) {
    if (rtl != model) {
        if (stageFails < 10)
            std::printf("    stage %s [%d]: rtl %lld model %lld\n", what, idx, (long long)rtl, (long long)model);
        ++stageFails;
    }
}
// compare the RTL state when it enters phase `ph` with the model snapshot
static void stageCompare(int ph) {
    auto* r = top->rootp;
    int64_t re, im;
    switch (ph) {
    case 10: // frame done
        check("hopi", sx(r->pv_unit__DOT__hopi, 42), dbg.hopIdeal, 0);
        check("hop", sx(r->pv_unit__DOT__hop, 28), dbg.hop, 0);
        check("err", sx(r->pv_unit__DOT__err, 18), dbg.err, 0);
        check("pos", int64_t(r->pv_unit__DOT__pos), dbg.pos, 0);
        for (uint32_t i = 0; i < 2048; ++i) {
            elem(i, re, im);
            check("frame.re", re, dbg.frameR[i], i);
            check("frame.im", im, dbg.frameI[i], i);
        }
        break;
    case 11: // FFT done
        for (uint32_t i = 0; i < 2048; ++i) {
            elem(i, re, im);
            check("fft.re", re, dbg.fftR[i], i);
            check("fft.im", im, dbg.fftI[i], i);
        }
        break;
    case 12: // A0 done: X[k] at rev(k)
        for (uint32_t k = 0; k < 2048; ++k) {
            elem(rev11(k), re, im);
            check("split.re", re, dbg.xr[k], k);
            check("split.im", im, dbg.xi[k], k);
        }
        check("nyq.re", sx(r->pv_unit__DOT__nyq_r, 27), dbg.xr[2048], 2048);
        check("nyq.im", sx(r->pv_unit__DOT__nyq_i, 27), dbg.xi[2048], 2048);
        check("fmul", sx(r->pv_unit__DOT__fmul, 17), dbg.mul, 0);
        break;
    case 13: { // A1 done
        for (uint32_t k = 0; k < 2048; ++k) {
            const auto& w = r->pv_unit__DOT__cur[k];
            const uint64_t lo = w[0] | (uint64_t(w[1]) << 32);
            const uint64_t hi = w[2];
            check("rotF", sx(lo & 0xFFFF, 16), dbg.rotF[k], k);
            check("phase", sx((lo >> 16) & 0xFFFF, 16), dbg.phase[k], k);
            check("L", int64_t((lo >> 32) & 0xFFFF), dbg.L[k], k);
            check("mag", int64_t((lo >> 48) | (hi << 16)) & ((1ll << 27) - 1), dbg.mag[k], k);
        }
        check("P", r->pv_unit__DOT__P, dbg.P, 0);
        check("strongest", r->pv_unit__DOT__strongest, dbg.strongest, 0);
        for (int i = 0; i < dbg.P && i < 1024; ++i)
            check("peak", r->pv_unit__DOT__pk_mem[i], dbg.peak[i], i);
        break;
    }
    case 15: // B done
        if (r->pv_unit__DOT__cont)
            check("tmul", sx(r->pv_unit__DOT__tmul, 26), dbg.mulT, 0);
        for (int i = 0; i < dbg.P && i < 1024; ++i) {
            check("end", r->pv_unit__DOT__en_mem[i], dbg.end[i], i);
            check("delta", sx(r->pv_unit__DOT__dl_mem[i], 16), dbg.delta[i], i);
        }
        break;
    case 16: // C1 done
        for (uint32_t k = 0; k < 2048; ++k) {
            const auto& w = r->pv_unit__DOT__cur[k];
            check("rot", sx(w[0] & 0xFFFF, 16), dbg.rot[k], k);
        }
        break;
    case 17: // C2 done
        for (uint32_t k = 0; k < 2048; ++k) {
            elem(k, re, im);
            check("merge.re", re, dbg.zr[k], k);
            check("merge.im", im, dbg.zi[k], k);
        }
        break;
    case 18: case 20: // IFFT done
        for (uint32_t i = 0; i < 2048; ++i) {
            elem(i, re, im);
            check("ifft.re", re, dbg.ifftR[i], i);
            check("ifft.im", im, dbg.ifftI[i], i);
        }
        break;
    }
}

static int64_t wrapPos(int64_t p, int64_t F) {
    p %= F;
    return p < 0 ? p + F : p;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const std::string rtl = "../rtl";
    std::string wavPath = argc > 1 ? argv[1] : "";
    top = new Vpv_unit;

    // memory: two buffers (a WAV if given, else a synthetic tone + noise)
    std::vector<int16_t> pcmA, pcmB;
    {
        wav::Audio a;
        if (!wavPath.empty() && wav::read(wavPath, a)) {
            for (size_t i = 0; i < std::min<size_t>(a.x.size(), 120000); ++i)
                pcmA.push_back(int16_t(std::lrint(std::max(-1.0f, std::min(1.0f, a.x[i])) * 32767)));
        } else {
            uint32_t x = 1;
            for (int i = 0; i < 60000; ++i) {
                x = x * 1664525u + 1013904223u;
                const double s = 0.5 * std::sin(i * 0.031) * std::exp(-(i % 6000) / 1500.0) + 0.1 * ((x >> 8) / 16777216.0 - 0.5);
                pcmA.push_back(int16_t(std::lrint(s * 32767)));
            }
        }
        for (int i = 0; i < 20000; ++i)
            pcmB.push_back(int16_t(std::lrint(9000 * std::sin(i * 0.11) + 6000 * std::sin(i * 0.0173))));
    }
    SlotSim sl[2];
    auto addBuf = [&](const std::vector<int16_t>& pcm, SlotSim& s) {
        const uint32_t base = (uint32_t(mem.size()) + 63) & ~63u;
        const uint32_t F = uint32_t(pcm.size());
        mem.resize(base + SMP_PREPAD + F + SMP_POSTPAD + 64, 0);
        for (uint32_t i = 0; i < SMP_PREPAD; ++i) mem[base + i] = pcm[(F - SMP_PREPAD + i) % F];
        for (uint32_t i = 0; i < F; ++i) mem[base + SMP_PREPAD + i] = pcm[i];
        for (uint32_t i = 0; i < SMP_POSTPAD; ++i) mem[base + SMP_PREPAD + F + i] = pcm[i % F];
        s.base = base;
        s.frames = F;
    };
    mem.resize(256, 0);
    std::printf("buffers: A %zu frames (%s), B %zu frames\n", pcmA.size(), wavPath.empty() ? "synthetic" : wavPath.c_str(), pcmB.size());
    addBuf(pcmA, sl[0]);
    addBuf(pcmB, sl[1]);

    smp::PvCore core;
    if (!core.loadTables(rtl)) {
        std::fprintf(stderr, "cannot load PV tables from %s\n", rtl.c_str());
        return 1;
    }

    // reset
    top->rst = 1;
    for (int i = 0; i < 8; ++i)
        tick();
    top->rst = 0;
    tick();

    uint32_t now = 0;
    // play (emission) up to sample e: read & clear ring entries in RTL and model
    auto playTo = [&](uint32_t e) {
        while (now < e) {
            for (int s = 0; s < 2; ++s) {
                sl[s].ring[now & (smp::PV_RING - 1)] = 0;
                top->em_re = 1;
                top->em_clr = 1;
                top->em_slot = s;
                top->em_addr = now & 4095;
                top->now = now;
                tick();
            }
            top->em_re = 0;
            top->em_clr = 0;
            ++now;
        }
        top->now = now;
    };
    int fails = 0, jobs = 0;
    auto compareRings = [&](const char* what) {
        for (int s = 0; s < 2; ++s)
            for (uint32_t a = 0; a < 4096; ++a) {
                top->em_re = 1;
                top->em_clr = 0;
                top->em_slot = s;
                top->em_addr = a;
                tick();
                top->em_re = 0;
                tick();
                const int32_t r = (int32_t(top->em_q) << 4) >> 4;
                if (r != sl[s].ring[a]) {
                    if (fails < 12)
                        std::printf("  MISMATCH after %s: slot %d ring[%u] rtl %d model %d\n", what, s, a, r, sl[s].ring[a]);
                    ++fails;
                }
            }
    };
    struct Job { int slot; uint32_t c, dl; int64_t ph; int ahead; int32_t inc; uint32_t rate; bool reset, notr; int win; };
    auto run = [&](const Job& j) {
        SlotSim& s = sl[j.slot];
        // model
        const int64_t F = int64_t(s.frames) << 32;
        const int64_t incq = int64_t(j.inc) << 8;
        if (j.win == smp::PV_WIN_R0) {
            const uint32_t c0 = j.c, e = c0 - SMP_LATENCY;
            const smp::PvTables& t = core.tables();
            for (uint32_t u = 0; u < smp::PV_FADE; ++u) {
                int32_t& v = s.ring[(c0 + u) & 4095];
                v = int32_t((int64_t(v) * t.fadeOut[u] + 32768) >> 16);
            }
            for (uint32_t tt = c0 + smp::PV_FADE; tt != e + smp::PV_RING; ++tt)
                s.ring[tt & 4095] = 0;
        }
        smp::PvJob pj;
        pj.frames = s.frames;
        pj.base = s.base;
        pj.pos = wrapPos(j.ph + int64_t(j.ahead) * incq, F);
        pj.inc = incq;
        pj.rate = j.rate;
        pj.reset = j.reset;
        pj.win = j.win;
        core.transientSuppression = !j.notr;
        core.dbg = &dbg;
        core.run(pj, s.st, mem);
        core.accumulate(s.ring.data(), j.c, j.win);
        // RTL
        top->job_valid = 1;
        top->job_slot = j.slot;
        top->job_clear = 0;
        top->job_win = j.win;
        top->job_reset = j.reset;
        top->job_notr = j.notr;
        top->job_c = j.c;
        top->job_dl = j.dl;
        top->job_ph = uint64_t(j.ph);
        top->job_ahead = uint16_t(j.ahead);
        top->job_inc = uint32_t(j.inc);
        top->job_rate = j.rate;
        top->job_base = s.base;
        top->job_frames = s.frames;
        tick();
        top->job_valid = 0;
        const uint64_t t0 = cycles;
        tick();
        int lastPh = top->rootp->pv_unit__DOT__ph;
        while (top->busy && cycles - t0 < 2000000) {
            tick();
            const int phn = top->rootp->pv_unit__DOT__ph;
            if (phn != lastPh) {
                const int before = stageFails;
                stageCompare(phn);
                if (stageFails != before && before < 10)
                    std::printf("  (job %d, entering phase %d)\n", jobs, phn);
                lastPh = phn;
            }
        }
        ++jobs;
        return cycles - t0;
    };

    // scenario: restarts (R0/R1/R2) and steady grains of both slots, issued in time order
    struct Phase { int slot; uint32_t e0; double posFrac, speed, pitch; bool notr; int grains; };
    std::vector<Phase> phases = {
        {0, 1000, 0.10, 0.5, 1.0, false, 6},
        {1, 1100, 0.30, 1.0, 1.0, false, 3},
        {0, 5000, 0.62, -0.75, 1.0, false, 5},
        {1, 5200, 0.05, 0.3, 1.5, true, 4},
        {0, 9000, 0.97, 2.0, 0.6, false, 5},
        {1, 9100, 0.50, -1.0, 0.8, false, 3},
    };
    struct Issue { uint32_t e; Job j; std::string what; };
    std::vector<Issue> issues;
    for (const Phase& p : phases) {
        SlotSim& s = sl[p.slot];
        const int64_t F = int64_t(s.frames) << 32;
        const int32_t inc = int32_t(std::lrint(p.speed * p.pitch * 16777216.0));
        const uint32_t rate = uint32_t(std::lrint(std::fabs(p.pitch) * 16777216.0));
        const int64_t P = int64_t(p.posFrac * s.frames * 4294967296.0) % F;
        const uint32_t c0 = p.e0 + SMP_LATENCY;
        for (int k = 0; k < 3; ++k)
            issues.push_back({p.e0, Job{p.slot, c0 + uint32_t(k) * smp::PV_H, k == 0 ? c0 : c0 + 256, P, k * smp::PV_H, inc,
                rate, k == 0, p.notr, smp::PV_WIN_R0 + k}, "slot " + std::to_string(p.slot) + " R" + std::to_string(k)});
        uint32_t nextC = c0 + 3 * smp::PV_H;
        for (int g = 0; g < p.grains; ++g) {
            const uint32_t e = nextC - 2 * smp::PV_H - SMP_PV_LOOKAHEAD;
            const int64_t ph = wrapPos(P + int64_t(e - p.e0) * (int64_t(inc) << 8), F);
            issues.push_back({e, Job{p.slot, nextC, nextC - 2 * smp::PV_H, ph, int(nextC - SMP_LATENCY - e), inc, rate, false,
                p.notr, smp::PV_WIN_NORMAL}, "slot " + std::to_string(p.slot) + " grain " + std::to_string(g)});
            nextC += smp::PV_H;
        }
    }
    std::stable_sort(issues.begin(), issues.end(), [](const Issue& a, const Issue& b) { return a.e < b.e; });
    uint64_t maxCycles = 0;
    for (const Issue& is : issues) {
        playTo(is.e);
        const uint64_t cyc = run(is.j);
        maxCycles = std::max(maxCycles, cyc);
        const int before = fails;
        compareRings(is.what.c_str());
        std::printf("%-16s e=%5u %6llu cycles%s\n", is.what.c_str(), is.e, (unsigned long long)cyc, fails != before ? "  FAIL" : "");
    }
    std::printf("%d jobs, max %llu cycles/job, misses %u: %s\n", jobs, (unsigned long long)maxCycles, top->misses,
        fails ? "FAIL" : "PASS");
    delete top;
    return fails ? 1 : 0;
}
