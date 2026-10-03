// Verilator testbench: rtl/sampler_engine.v vs model/sampler_model.cpp, bit exact.
//
// The SDRAM is modelled as a line server: each 64-word request is answered after
// `latency` engine cycles, one word per cycle, requests strictly in order.
#include "Vsampler_engine.h"
#include "verilated.h"
#include "../model/sampler_model.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <vector>

struct Write { uint32_t addr, data; };

struct Buffer { uint32_t base, frames; };

struct Scenario {
    const char* name;
    int samples;
    int latency;      // memory latency in engine cycles
    bool expectExact; // false: memory deliberately too slow, only report
    std::function<void(int n, std::vector<Write>& w)> script;
};

static std::vector<int16_t> gMem;
static Buffer gBufA, gBufB;

static Buffer addBuffer(const std::vector<int16_t>& pcm) {
    // firmware layout: 64-word aligned, [prepad 64][frames][postpad 128]
    uint32_t base = (static_cast<uint32_t>(gMem.size()) + 63) & ~63u;
    const uint32_t F = static_cast<uint32_t>(pcm.size());
    gMem.resize(base + SMP_PREPAD + F + SMP_POSTPAD, 0);
    for (uint32_t i = 0; i < SMP_PREPAD; ++i) gMem[base + i] = pcm[(F - SMP_PREPAD + i) % F];
    for (uint32_t i = 0; i < F; ++i) gMem[base + SMP_PREPAD + i] = pcm[i];
    for (uint32_t i = 0; i < SMP_POSTPAD; ++i) gMem[base + SMP_PREPAD + F + i] = pcm[i % F];
    return {base, F};
}

static std::vector<int16_t> makeLoop(uint32_t frames, uint32_t seed, double f0) {
    std::vector<int16_t> v(frames);
    uint32_t x = seed;
    for (uint32_t i = 0; i < frames; ++i) {
        const uint32_t hit = i % 9000;
        const double t = hit / 48000.0;
        x = x * 1664525u + 1013904223u;
        const double n = ((x >> 8) / 16777216.0 - 0.5);
        const double s = 0.6 * std::exp(-t * 25) * std::sin(2 * M_PI * f0 * (1 + (i / 9000) % 3) * t)
                       + 0.3 * std::exp(-t * 80) * n;
        v[i] = static_cast<int16_t>(std::lrint(std::clamp(s, -1.0, 1.0) * 32767));
    }
    return v;
}

static uint32_t q24(double v) { return static_cast<uint32_t>(static_cast<int32_t>(std::lrint(v * 16777216.0))); }

static void slotSetup(std::vector<Write>& w, int s, const Buffer& b, double grainSec, double overlap,
                      double rate, double pitch) {
    auto W = [&](int r, uint32_t v) { w.push_back({SMP_ADDR(s, r), v}); };
    W(SMP_R_BASE, b.base);
    W(SMP_R_FRAMES, b.frames);
    W(SMP_R_WIN_INC, static_cast<uint32_t>(4294967296.0 / (grainSec * 48000.0)));
    W(SMP_R_OVERLAP, static_cast<uint32_t>(std::lrint(overlap * 16777216.0)));
    W(SMP_R_AMP_TARGET, static_cast<uint32_t>(std::lrint(65536.0 / std::max(overlap, 1.0))));
    W(SMP_R_PHASOR_INC, q24(rate));
    W(SMP_R_PITCH, q24(pitch));
    W(SMP_R_ENV_DUR, 48000);
    W(SMP_R_CTRL, SMP_CTRL_ENABLE);
}

static int runScenario(const Scenario& sc, const char* romDir) {
    Vsampler_engine* top = new Vsampler_engine;
    smp::Engine model(2);
    model.loadTables(romDir);
    model.mem = gMem;

    std::deque<Write> cmdq;
    std::deque<uint32_t> reqq;  // pending line requests (word address)
    uint64_t cyc = 0;
    int64_t serveAt = -1;
    int wordIdx = 0;
    int reqDepth = 32;

    auto tick = [&]() {
        top->clk = 0; top->eval();
        top->clk = 1; top->eval();
        ++cyc;
    };
    top->rst = 1;
    for (int i = 0; i < 4; ++i) tick();
    top->rst = 0;

    std::vector<Write> w;
    sc.script(0, w);
    for (auto& x : w) { cmdq.push_back(x); model.write(x.addr, x.data); }

    int n = 0, mism = 0, firstBad = -1;
    double errE = 0, sigE = 0;
    while (n < sc.samples) {
        // drive inputs for this cycle (sampled at the rising edge in tick())
        top->cmd_valid = !cmdq.empty();
        if (!cmdq.empty()) { top->cmd_addr = cmdq.front().addr; top->cmd_data = cmdq.front().data; }
        top->req_ready = reqq.size() < static_cast<size_t>(reqDepth);
        // memory: serve head request
        top->fill_valid = 0;
        if (!reqq.empty()) {
            if (serveAt < 0) serveAt = static_cast<int64_t>(cyc) + sc.latency;
            if (static_cast<int64_t>(cyc) >= serveAt) {
                top->fill_valid = 1;
                top->fill_data = static_cast<uint16_t>(gMem[(reqq.front() + wordIdx) % gMem.size()]);
            }
        }
        top->eval();
        const bool cmdTaken = top->cmd_valid && top->cmd_ready;
        const bool reqTaken = top->req_valid && top->req_ready;
        const uint32_t reqAddr = top->req_addr;
        const bool fillTaken = top->fill_valid;
        tick();
        if (cmdTaken) cmdq.pop_front();
        if (reqTaken) reqq.push_back(reqAddr);
        if (fillTaken && ++wordIdx == 64) { wordIdx = 0; reqq.pop_front(); serveAt = -1; }

        if (top->out_stb) {
            const int32_t rtl = static_cast<int32_t>(top->out_sample << 8) >> 8;
            const int32_t ref = model.step();
            if (rtl != ref) {
                if (firstBad < 0) firstBad = n;
                if (mism < 5)
                    std::printf("  [%s] sample %d: rtl %d model %d\n", sc.name, n, rtl, ref);
                ++mism;
            }
            errE += double(rtl - ref) * double(rtl - ref);
            sigE += double(ref) * double(ref);
            ++n;
            w.clear();
            sc.script(n, w);
            for (auto& x : w) { cmdq.push_back(x); model.write(x.addr, x.data); }
        }
    }
    // read status words through the streaming port: run idle cycles and capture
    std::map<int, uint32_t> st;
    for (int i = 0; i < 300; ++i) {
        top->cmd_valid = 0; top->fill_valid = 0; top->req_ready = 1;
        top->eval();
        tick();
        if (top->stat_we) st[top->stat_addr] = top->stat_data;
    }
    const uint32_t miss = st[SMP_S_CACHE_MISS] + st[8 + SMP_S_CACHE_MISS];
    std::printf("%-22s rms %5.1f dBFS ", sc.name, 10 * std::log10(sigE / std::max(n, 1) / (8388608.0 * 8388608.0) + 1e-30));
    std::printf("%6d samples  mismatches %6d  SNR(rtl vs model) %s  cycles max %4u  misses %u  "
                "req drops %u  magic %08x\n",
                n, mism,
                mism ? (std::to_string(10 * std::log10(sigE / std::max(errE, 1e-9))) + " dB").c_str() : "exact",
                st[SMP_SG_CYCLES_MAX], miss, st[SMP_SG_REQ_DROPS], st[SMP_SG_MAGIC]);
    delete top;
    if (st[SMP_SG_PV_MISSES])
        std::printf("  [%s] phase-vocoder deadline misses / job drops: %u\n", sc.name, st[SMP_SG_PV_MISSES]);
    bool ok = sc.expectExact ? (mism == 0 && miss == 0 && st[SMP_SG_CYCLES_MAX] < 1024 && st[SMP_SG_PV_MISSES] == 0) : true;
    if (st[SMP_SG_MAGIC] != SMP_MAGIC) ok = false;
    return ok ? 0 : 1;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const char* romDir = "../rtl";
    std::string only = argc > 1 ? argv[1] : "";

    gBufA = addBuffer(makeLoop(72001, 1, 110));
    gBufB = addBuffer(makeLoop(38417, 7, 330));

    std::vector<Scenario> scs;

    scs.push_back({"ola_basic", 30000, 40, true, [](int n, std::vector<Write>& w) {
        if (n == 0) slotSetup(w, 0, gBufA, 0.04, 4, 1.0, 1.0);
        if (n == 0 || n == 15000) {
            w.push_back({SMP_ADDR(0, SMP_R_POS), n ? (gBufA.frames / 2) << 8 : 0});
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
    }});

    scs.push_back({"ola_stretch_reverse", 30000, 40, true, [](int n, std::vector<Write>& w) {
        if (n == 0) {
            slotSetup(w, 0, gBufA, 0.05, 3, 0.37, 1.25);
            slotSetup(w, 1, gBufB, 0.06, 2, -1.0, -0.8);
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_TRIG});
            w.push_back({SMP_ADDR(1, SMP_R_POS), 100u << 8});
            w.push_back({SMP_ADDR(1, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
    }});

    scs.push_back({"granular_random", 40000, 40, true, [](int n, std::vector<Write>& w) {
        if (n == 0) {
            slotSetup(w, 0, gBufA, 0.0071, 1.7, 0.8, 1.0);
            w.push_back({SMP_ADDR(0, SMP_R_PROB), 52000});
            w.push_back({SMP_ADDR(0, SMP_R_SEPARATION), 40000});
            w.push_back({SMP_ADDR(0, SMP_R_RANDPOS), gBufA.frames / 10});
            w.push_back({SMP_ADDR(0, SMP_R_RANDPITCH), 4096});      // +-1 octave
            w.push_back({SMP_ADDR(0, SMP_R_SEED), 12345});
            slotSetup(w, 1, gBufB, 0.03, 6, 1.0, 1.5);
            w.push_back({SMP_ADDR(1, SMP_R_RANDPITCH), static_cast<uint32_t>(-2048) & 0xFFFF});
            w.push_back({SMP_ADDR(1, SMP_R_SEPARATION), 65535});
            w.push_back({SMP_ADDR(1, SMP_R_RANDPOS), gBufB.frames});
        }
        if (n % 7000 == 0) {
            w.push_back({SMP_ADDR(0, SMP_R_POS), ((n * 977u) % gBufA.frames) << 8});
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
        if (n == 21000) w.push_back({SMP_ADDR(0, SMP_R_PITCH), q24(0.5)});
    }});

    scs.push_back({"stutter_env", 40000, 40, true, [](int n, std::vector<Write>& w) {
        if (n == 0) {
            slotSetup(w, 0, gBufA, 0.04, 4, 1.0, 1.0);
            w.push_back({SMP_ADDR(0, SMP_R_ENV_DUR), 9000});
            slotSetup(w, 1, gBufB, 0.04, 4, 1.0, 1.0);
            w.push_back({SMP_ADDR(1, SMP_R_CTRL), SMP_CTRL_ENABLE | SMP_CTRL_ENV_HOLD});
        }
        if (n % 12000 == 10) {
            w.push_back({SMP_ADDR(0, SMP_R_REPEATS), static_cast<uint32_t>((n / 12000) * 3)});
            w.push_back({SMP_ADDR(0, SMP_R_INTERVAL), 1500});
            w.push_back({SMP_ADDR(0, SMP_R_POS), ((n / 12000) * 4800u) << 8});
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
        if (n == 30000) { // live interval change: replay rate startInterval/interval
            w.push_back({SMP_ADDR(0, SMP_R_INTERVAL), 2000});
        }
        if (n == 5) {
            w.push_back({SMP_ADDR(1, SMP_R_REPEATS), 5});
            w.push_back({SMP_ADDR(1, SMP_R_INTERVAL), 777});
            w.push_back({SMP_ADDR(1, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
        if (n == 20000) w.push_back({SMP_ADDR(1, SMP_R_STROBE), SMP_STROBE_GATE_OFF});
        if (n == 26000) {
            w.push_back({SMP_ADDR(1, SMP_R_AMP), 8192});
            w.push_back({SMP_ADDR(1, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
    }});

    scs.push_back({"pin_kill_bufswitch", 30000, 40, true, [](int n, std::vector<Write>& w) {
        if (n == 0) {
            slotSetup(w, 0, gBufA, 0.08, 4, 1.0, 1.0);
            w.push_back({SMP_ADDR(0, SMP_R_CTRL), SMP_CTRL_ENABLE | SMP_CTRL_POS_PIN | SMP_CTRL_ENV_HOLD});
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
        if (n > 0 && n < 10000 && n % 50 == 0) // scrub the pinned position
            w.push_back({SMP_ADDR(0, SMP_R_POS), static_cast<uint32_t>((n * 3) << 8)});
        if (n == 10000) {
            w.push_back({SMP_ADDR(0, SMP_R_BASE), gBufB.base});
            w.push_back({SMP_ADDR(0, SMP_R_FRAMES), gBufB.frames});
            w.push_back({SMP_ADDR(0, SMP_R_CTRL), SMP_CTRL_ENABLE | SMP_CTRL_ENV_HOLD});
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_KILL});
        }
        if (n == 20000) {
            w.push_back({SMP_ADDR(7, SMP_G_MASTER), 2048});
            w.push_back({SMP_ADDR(0, SMP_R_ALGO_AMP), 16384});
        }
    }});

    scs.push_back({"seamless_switch", 30000, 40, true, [](int n, std::vector<Write>& w) {
        if (n == 0) {
            slotSetup(w, 0, gBufA, 0.05, 4, 1.0, 1.0);
            w.push_back({SMP_ADDR(0, SMP_R_CTRL), SMP_CTRL_ENABLE | SMP_CTRL_ENV_HOLD});
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_TRIG});
            slotSetup(w, 1, gBufB, 0.02, 3, 0.8, 1.2);
            w.push_back({SMP_ADDR(1, SMP_R_RANDPOS), 2000});
            w.push_back({SMP_ADDR(1, SMP_R_RANDPITCH), 2048});
            w.push_back({SMP_ADDR(1, SMP_R_CTRL), SMP_CTRL_ENABLE | SMP_CTRL_ENV_HOLD});
            w.push_back({SMP_ADDR(1, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
        if (n > 0 && n % 2500 == 0) { // buffer switch without KILL; grains keep their buffer
            const Buffer& b = (n / 2500) & 1 ? gBufB : gBufA;
            for (int s = 0; s < 2; ++s) {
                const Buffer& bs = s ? ((n / 2500) & 1 ? gBufA : gBufB) : b;
                w.push_back({SMP_ADDR(s, SMP_R_BASE), bs.base});
                w.push_back({SMP_ADDR(s, SMP_R_FRAMES), bs.frames});
            }
            if (n % 5000 == 0) { // beat-aligned seek on slot 0
                w.push_back({SMP_ADDR(0, SMP_R_POS), ((n * 7u) % b.frames) << 8});
                w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_SEEK});
            }
        }
    }});

    scs.push_back({"bg_advance", 40000, 40, true, [](int n, std::vector<Write>& w) {
        if (n == 0) {
            slotSetup(w, 0, gBufB, 0.04, 4, 1.0, 1.0);   // melody-style: runs in the background
            w.push_back({SMP_ADDR(0, SMP_R_CTRL), SMP_CTRL_ENABLE | SMP_CTRL_POS_BG});
            w.push_back({SMP_ADDR(0, SMP_R_ENV_DUR), 3000});
            w.push_back({SMP_ADDR(0, SMP_R_REPEATS), 2});
            w.push_back({SMP_ADDR(0, SMP_R_INTERVAL), 700});
            slotSetup(w, 1, gBufA, 0.05, 3, 0.7, 1.0);
            w.push_back({SMP_ADDR(1, SMP_R_CTRL), SMP_CTRL_ENABLE | SMP_CTRL_ENV_HOLD | SMP_CTRL_POS_PIN | SMP_CTRL_POS_BG});
            w.push_back({SMP_ADDR(1, SMP_R_POS), 5000u << 8});
            w.push_back({SMP_ADDR(1, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
        if (n % 5000 == 1000) {                          // note-ons: position = pos + background
            w.push_back({SMP_ADDR(0, SMP_R_POS), ((n * 13u) % gBufB.frames) << 8});
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
        if (n == 17000)
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_BG_RESET});
        if (n == 23000) {                                // background folds on a shorter buffer
            w.push_back({SMP_ADDR(1, SMP_R_BASE), gBufB.base});
            w.push_back({SMP_ADDR(1, SMP_R_FRAMES), gBufB.frames});
            w.push_back({SMP_ADDR(0, SMP_R_PHASOR_INC), q24(-0.6)});
        }
        if (n == 31000)
            w.push_back({SMP_ADDR(0, SMP_R_CTRL), SMP_CTRL_ENABLE});  // toggle off again
    }});

    // ---- phase vocoder
    scs.push_back({"pv_basic", 30000, 40, true, [](int n, std::vector<Write>& w) {
        if (n == 0) {
            slotSetup(w, 0, gBufA, 0.04, 4, 0.5, 1.0);
            w.push_back({SMP_ADDR(0, SMP_R_ALGO), SMP_ALGO_PV});
        }
        if (n == 0 || n == 15000) {
            w.push_back({SMP_ADDR(0, SMP_R_POS), n ? (gBufA.frames / 3) << 8 : 0});
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
    }});

    scs.push_back({"pv_switch_stutter", 40000, 40, true, [](int n, std::vector<Write>& w) {
        if (n == 0) {
            slotSetup(w, 0, gBufA, 0.04, 4, 0.7, 1.0);
            slotSetup(w, 1, gBufB, 0.05, 3, -1.0, 1.3);
            w.push_back({SMP_ADDR(1, SMP_R_ALGO), SMP_ALGO_PV | SMP_ALGO_NO_TRANSIENT});
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_TRIG});
            w.push_back({SMP_ADDR(1, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
        if (n == 5000 || n == 20000) w.push_back({SMP_ADDR(0, SMP_R_ALGO), SMP_ALGO_PV});
        if (n == 15000 || n == 32000) w.push_back({SMP_ADDR(0, SMP_R_ALGO), SMP_ALGO_GRAIN});
        if (n == 8000) {
            w.push_back({SMP_ADDR(1, SMP_R_POS), (gBufB.frames / 4) << 8});
            w.push_back({SMP_ADDR(1, SMP_R_REPEATS), 3});
            w.push_back({SMP_ADDR(1, SMP_R_INTERVAL), 2400});
            w.push_back({SMP_ADDR(1, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
        if (n == 26000) w.push_back({SMP_ADDR(1, SMP_R_PITCH), q24(0.7)});
    }});

    scs.push_back({"pv_pin_seek_kill", 36000, 40, true, [](int n, std::vector<Write>& w) {
        if (n == 0) {
            slotSetup(w, 0, gBufA, 0.04, 4, 1.0, 1.0);
            w.push_back({SMP_ADDR(0, SMP_R_ALGO), SMP_ALGO_PV});
            w.push_back({SMP_ADDR(0, SMP_R_POS), 9000u << 8});
            w.push_back({SMP_ADDR(0, SMP_R_CTRL), SMP_CTRL_ENABLE | SMP_CTRL_POS_PIN | SMP_CTRL_ENV_HOLD});
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
        if (n == 6000) w.push_back({SMP_ADDR(0, SMP_R_POS), 9300u << 8});        // scrub while pinned
        if (n == 10000) w.push_back({SMP_ADDR(0, SMP_R_CTRL), SMP_CTRL_ENABLE | SMP_CTRL_ENV_HOLD});
        if (n == 14000) {
            w.push_back({SMP_ADDR(0, SMP_R_POS), 30000u << 8});
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_SEEK});
        }
        if (n == 18000) {
            w.push_back({SMP_ADDR(0, SMP_R_BASE), gBufB.base});
            w.push_back({SMP_ADDR(0, SMP_R_FRAMES), gBufB.frames});
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_KILL});
        }
        if (n == 23000) w.push_back({SMP_ADDR(0, SMP_R_CTRL), 0});                  // disable
        if (n == 25000) {                                                            // enable again
            w.push_back({SMP_ADDR(0, SMP_R_CTRL), SMP_CTRL_ENABLE | SMP_CTRL_ENV_HOLD});
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
    }});

    scs.push_back({"stress_64_grains", 12000, 40, true, [](int n, std::vector<Write>& w) {
        if (n == 0) {
            for (int s = 0; s < 2; ++s) {
                slotSetup(w, s, s ? gBufB : gBufA, 0.4, 31, 1.0, 2.0);
                w.push_back({SMP_ADDR(s, SMP_R_RANDPITCH), 8192});  // +-2 octaves
                w.push_back({SMP_ADDR(s, SMP_R_RANDPOS), (s ? gBufB : gBufA).frames});
                w.push_back({SMP_ADDR(s, SMP_R_ENV_DUR), 0xFFFFFF});
                w.push_back({SMP_ADDR(s, SMP_R_STROBE), SMP_STROBE_TRIG});
            }
        }
    }});

    scs.push_back({"slow_memory_report", 8000, 12000, false, [](int n, std::vector<Write>& w) {
        if (n == 0) {
            slotSetup(w, 0, gBufA, 0.04, 4, 1.0, 1.0);
            w.push_back({SMP_ADDR(0, SMP_R_STROBE), SMP_STROBE_TRIG});
        }
    }});

    int fails = 0;
    for (auto& sc : scs)
        if (only.empty() || only == sc.name)
            fails += runScenario(sc, romDir);
    std::printf(fails ? "FAILED (%d)\n" : "ALL PASS\n", fails);
    return fails ? 1 : 0;
}
