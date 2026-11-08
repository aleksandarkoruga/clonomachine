// rtl/synth_osc.v against the synth model's floating-point read (model/synth_osc.h, synth_warp.h):
// one voice, the SDRAM emulated (line requests served after a latency, one word per cycle), the
// voice's filter sum compared sample by sample after the frames are in. Scenarios cover the plain
// read, smooth frame crossfade, unison, cross FM, sync and asym warps, and the cycle budget with all
// three voices busy.
//
//   make synthosc && ./obj_so/Vsynth_osc [scenario]
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "Vsynth_osc.h"
#include "verilated.h"
#include "../model/synth_osc.h"
#include "../model/synth_warp.h"
#include "../model/wt_tables.h"

using namespace synth;

namespace {
Vsynth_osc* top;
uint64_t cyc = 0;
std::vector<uint16_t> mem;  // SDRAM words
std::deque<uint32_t> reqq;
int64_t serveAt = -1;
int wordIdx = 0;
const int kLatency = 60;

void tick() {
    top->clk = 0;
    top->eval();
    top->clk = 1;
    top->eval();
    ++cyc;
}
void cycle() {  // one clock with the memory model
    top->req_ready = reqq.size() < 32;
    top->fill_valid = 0;
    if (!reqq.empty()) {
        if (serveAt < 0)
            serveAt = int64_t(cyc) + kLatency;
        if (int64_t(cyc) >= serveAt) {
            top->fill_valid = 1;
            top->fill_data = mem[(reqq.front() + wordIdx) % mem.size()];
        }
    }
    top->eval();
    const bool rq = top->req_valid && top->req_ready;
    const uint32_t ra = top->req_addr;
    const bool fv = top->fill_valid;
    tick();
    top->we = 0;
    top->start = 0;
    if (rq)
        reqq.push_back(ra);
    if (fv && ++wordIdx == 64) {
        wordIdx = 0;
        reqq.pop_front();
        serveAt = -1;
    }
}
void wr(int addr, uint32_t data) {
    top->we = 1;
    top->waddr = addr;
    top->wdata = data;
    cycle();
}
void wosc(int o, int r, uint32_t d) { wr((o << 5) | r, d); }
void wvoice(int v, int r, uint32_t d) { wr(0x100 | (v << 4) | r, d); }

struct OscCfg {
    bool on = false;
    double f = 110;     // Hz
    double pos = 0.5;   // 0..1
    bool smooth = false;
    int uni = 1;
    double detune = 0;  // semitones of the outer voices
    int w1 = 0, w2 = 0;
    double a1 = 0, a2 = 0;
    bool toFilter = true;
};
struct Scenario {
    const char* name;
    int samples;
    OscCfg osc[2];
    bool bFirst = false;
    bool allVoices = false;  // the same patch on all three voices (cycle budget)
};

// the warp's register values (par, parb, parc) as the soft core would write them
struct WarpRegs {
    uint32_t par = 0, parb = 0, parc = 0;
};
WarpRegs warp_regs(int w, double a) {
    a = std::clamp(a, 0.0, 1.0);
    WarpRegs r;
    auto u612 = [](double v) { return uint32_t(std::lrint(std::min(v, 63.99) * 4096)); };
    switch (w) {
    case W_SYNC: case W_SYNCWIN: r.par = u612(std::exp2(4 * a)); break;
    case W_PWM: r.par = u612(1 / (1 - 0.95 * a)); break;
    case W_ASYM_P: case W_ASYM_M: case W_ASYM_PM: case W_MIRROR: {
        const double d = synth::detail::knee_d(w == W_MIRROR ? W_ASYM_PM : w, a);
        r.par = uint32_t(std::llrint(d * 4294967296.0));
        r.parb = u612(0.5 / d);
        r.parc = u612(0.5 / (1 - d));
        break;
    }
    case W_BEND_P: r.par = uint32_t(std::lrint(a * 65536)) & 0x3FFFF; break;
    case W_BEND_M: r.par = uint32_t(std::lrint(-a * 65536)) & 0x3FFFF; break;
    case W_BEND_PM: r.par = uint32_t(std::lrint((2 * a - 1) * 65536)) & 0x3FFFF; break;
    case W_FM: case W_FM_SUB: case W_FM_NOISE: r.par = uint32_t(std::lrint(2 * a * 65536)) & 0x3FFFF; break;  // s2.16
    default: break;
    }
    return r;
}
}  // namespace

static int run(const Scenario& sc, const wt::Table& T) {
    top = new Vsynth_osc;
    cyc = 0;
    reqq.clear();
    serveAt = -1;
    wordIdx = 0;
    top->rst = 1;
    for (int i = 0; i < 4; ++i)
        cycle();
    top->rst = 0;
    const double fsH = 96000.0;
    // per oscillator: level, registers
    int lev[2];
    for (int v = 0; v < 3; ++v) {
        const bool act = v == 0 || sc.allVoices;
        for (int k = 0; k < 2; ++k) {
            const int o = 2 * v + k;
            const OscCfg& c = sc.osc[k];
            double slope = 1;
            if (c.w1 != W_OFF && c.w1 < W_FM)
                warp_phase(c.w1, c.a1, 0.5, &slope);
            int L = wt::level_for(T, c.f * slope, 43200);
            while (T.size(L) > 1024)
                ++L;
            lev[k] = L;
            const int logn = __builtin_ctz(T.size(L));
            for (int u = 0; u < 4; ++u) {
                const double sp = c.uni > 1 ? 2.0 * u / (c.uni - 1) - 1 : 0;
                wosc(o, u, uint32_t(std::llrint(c.f * std::exp2(sp * c.detune / 12) / fsH * 4294967296.0)));
                wosc(o, 4 + u, uint32_t(std::lrint(65536 / std::sqrt(double(c.uni)))) & 0x3FFFF);
            }
            wosc(o, 8, uint32_t(std::lrint(c.pos * (T.frames - 1) * 65536)) | (c.smooth ? 1u << 24 : 0));
            wosc(o, 9, T.base[L] | uint32_t(logn) << 25);
            const WarpRegs r1 = warp_regs(c.w1, c.a1), r2 = warp_regs(c.w2, c.a2);
            wosc(o, 11, r1.par);
            wosc(o, 12, r2.par);
            wosc(o, 13, r1.parb);
            wosc(o, 14, r2.parb);
            wosc(o, 17, r1.parc);
            wosc(o, 18, r2.parc);
            wosc(o, 15, 65536);
            wosc(o, 16, T.frames - 1);
            wosc(o, 10, c.w1 | c.w2 << 5 | (c.uni - 1) << 10 | (c.toFilter ? 1 << 12 : 0) | 1 << 13 |
                        ((c.on && act) ? 1 << 14 : 0));
        }
        wvoice(v, 1, 0);
        wvoice(v, 2, 0);
        wvoice(v, 3, sc.bFirst ? 4 : 0);
    }
    // the reference (voice 0)
    std::vector<double> ref, rtl;
    std::vector<int32_t> raw;
    double ph[2][4] = {}, prev[2] = {0, 0};
    std::vector<double> out;
    int startCyc = 0, warm = 0;
    unsigned cmax = 0;
    for (int n = 0; n < sc.samples; ++n) {
        top->start = 1;
        cycle();
        int c = 0;
        while (top->busy && c < 4000)
            cycle(), ++c;
        cmax = std::max<unsigned>(cmax, c);
        // read voice 0, both sub-samples (Q8.15)
        for (int s = 0; s < 2; ++s) {
            top->out_ra = s << 2;
            cycle();
            cycle();
            ++c;
            ++c;
            rtl.push_back((int32_t(top->out_filt << 8) >> 8) / 32768.0);  // s24
        }
        if (std::getenv("OSRAW"))  // every voice's sums, raw, for bit-exact comparisons of RTL changes
            for (int a = 0; a < 8; ++a) {
                top->out_ra = a;
                cycle();
                cycle();
                c += 2;
                raw.push_back(int32_t(top->out_filt));
                raw.push_back(int32_t(top->out_direct));
            }
        while (c < 1023)  // the rest of the sample period
            cycle(), ++c;
        // reference: two sub-samples
        for (int s = 0; s < 2; ++s) {
            double vsum = 0;
            const int order[2] = {sc.bFirst ? 1 : 0, sc.bFirst ? 0 : 1};
            for (int oi = 0; oi < 2; ++oi) {
                const int k = order[oi];
                const OscCfg& cfg = sc.osc[k];
                if (!cfg.on) {
                    prev[k] = 0;
                    continue;
                }
                double sum = 0;
                for (int u = 0; u < cfg.uni; ++u) {
                    const double sp = cfg.uni > 1 ? 2.0 * u / (cfg.uni - 1) - 1 : 0;
                    const uint32_t inc = uint32_t(std::llrint(cfg.f * std::exp2(sp * cfg.detune / 12) / fsH * 4294967296.0));
                    double p = ph[k][u];
                    ph[k][u] = std::fmod(ph[k][u] + inc / 4294967296.0, 1.0);
                    for (int sl = 0; sl < 2; ++sl) {
                        const int w = sl ? cfg.w2 : cfg.w1;
                        const double a = sl ? cfg.a2 : cfg.a1;
                        if (w == W_FM)
                            p += 2 * a * prev[k ^ 1];
                        else if (w == W_SYNC)  // the ratio as the register holds it (u5.12)
                            p = warp_phase(w, std::log2(warp_regs(w, a).par / 4096.0) / 4, p);
                        else if (w != W_OFF)
                            p = warp_phase(w, a, p);
                        p -= std::floor(p);
                    }
                    double pos = cfg.pos;
                    if (!cfg.smooth)
                        pos = std::round(pos * (T.frames - 1)) / (T.frames - 1);
                    sum += read_table(T, lev[k], pos, p, Interp::SINC8) * T.qscale / std::sqrt(double(cfg.uni));
                }
                prev[k] = sum;
                vsum += sum;
            }
            ref.push_back(vsum);
        }
        if (warm == 0 && n > 0 && std::fabs(rtl.back()) > 0)
            warm = n;
    }
    (void)startCyc;
    // compare after the frames are in (+ 4 samples)
    const size_t from = size_t(warm + 4) * 2;
    double se = 0, ss = 0, maxe = 0;
    for (size_t i = from; i < ref.size(); ++i) {
        const double e = rtl[i] - ref[i];
        se += e * e;
        ss += ref[i] * ref[i];
        maxe = std::max(maxe, std::fabs(e));
    }
    if (std::getenv("SODUMP")) {
        size_t wi = from;
        for (size_t i = from; i < ref.size(); ++i)
            if (std::fabs(rtl[i] - ref[i]) > std::fabs(rtl[wi] - ref[wi]))
                wi = i;
        for (size_t i = wi - 3; i < wi + 4 && i < ref.size(); ++i)
            std::printf("  worst %zu rtl %.5f ref %.5f\n", i, rtl[i], ref[i]);
    }
    if (std::getenv("SODUMP"))
        for (size_t i = from; i < std::min(ref.size(), from + 12); ++i)
            std::printf("  %zu rtl %.5f ref %.5f\n", i, rtl[i], ref[i]);
    if (const char* d = std::getenv("OSRAW"))
        if (FILE* f = std::fopen((std::string(d) + "/" + sc.name + ".raw").c_str(), "wb")) {
            std::fwrite(raw.data(), 4, raw.size(), f);
            std::fclose(f);
        }
    const double snr = se > 0 ? 10 * std::log10(ss / se) : 200;
    std::printf("%-18s first sound at sample %3d  SNR %6.1f dB  max error %.2e  cycles max %4u  lag %u\n", sc.name,
                warm, snr, maxe, cmax, top->lag);
    const bool ok = snr > 65 && cmax < 1024 && warm < 40;
    delete top;
    return ok ? 0 : 1;
}


// A lvl write drops the cached frames (another table at the same frame size must not play the
// old one's frames), and mode [15] with fresh restarts the phases from ph0. Voice 0, oscillator A:
// table T, then T2 at the same level (same frame size, other words), then a phase restart.
static int run_switch(const wt::Table& T, const wt::Table& T2, uint32_t base2) {
    top = new Vsynth_osc;
    cyc = 0;
    reqq.clear();
    serveAt = -1;
    wordIdx = 0;
    top->rst = 1;
    for (int i = 0; i < 4; ++i)
        cycle();
    top->rst = 0;
    const double f = 330, fsH = 96000.0;
    int L = wt::level_for(T, f, 43200);
    while (T.size(L) > 1024)
        ++L;
    const int logn = __builtin_ctz(T.size(L));
    const uint32_t inc = uint32_t(std::llrint(f / fsH * 4294967296.0));
    const double pos = 0.5;
    wosc(0, 0, inc);
    wosc(0, 4, 65536);
    wosc(0, 8, uint32_t(std::lrint(pos * (T.frames - 1) * 65536)));
    wosc(0, 9, T.base[L] | uint32_t(logn) << 25);
    wosc(0, 15, 65536);
    wosc(0, 16, T.frames - 1);
    wosc(0, 10, 1 << 12 | 1 << 13 | 1 << 14);
    for (int v = 0; v < 3; ++v) {
        wvoice(v, 1, 0);
        wvoice(v, 2, 0);
        wvoice(v, 3, 0);
    }
    const int nSwitch = 150, nReset = 300, N = 450;
    const uint32_t ph0 = 0x10000;   // a quarter cycle (x 2^14)
    uint32_t p32 = 0;
    const wt::Table* cur = &T;
    std::vector<double> rtl, ref;
    int firstSound = -1;
    for (int n = 0; n < N; ++n) {
        if (n == nSwitch) {   // the other table, same level and frame size: no fresh
            wosc(0, 9, (base2 + T2.base[L]) | uint32_t(logn) << 25);
            cur = &T2;
        }
        if (n == nReset) {    // a note's restart: fresh + phase restart from ph0
            wosc(0, 20, ph0);
            wosc(0, 10, 1 << 12 | 1 << 13 | 1 << 14 | 1 << 15);
            p32 = ph0 << 14;
        }
        top->start = 1;
        cycle();
        int c = 0;
        while (top->busy && c < 4000)
            cycle(), ++c;
        for (int s = 0; s < 2; ++s) {
            top->out_ra = s << 2;
            cycle();
            cycle();
            c += 2;
            rtl.push_back((int32_t(top->out_filt << 8) >> 8) / 32768.0);
            const double fpos = std::round(pos * (cur->frames - 1)) / (cur->frames - 1);
            ref.push_back(read_table(*cur, L, fpos, p32 / 4294967296.0, Interp::SINC8) * cur->qscale);
            p32 += inc;
        }
        while (c < 1023)
            cycle(), ++c;
        if (firstSound < 0 && std::fabs(rtl.back()) > 0)
            firstSound = n;
    }
    // compare: before the switch (after the start-up), after it (once the frame is in), after the reset
    auto snr = [&](int a, int b) {
        double se = 0, ss = 0;
        for (int i = 2 * a; i < 2 * b; ++i) {
            se += (rtl[i] - ref[i]) * (rtl[i] - ref[i]);
            ss += ref[i] * ref[i];
        }
        return se > 0 ? 10 * std::log10(ss / se) : 200.0;
    };
    const double s0 = snr(firstSound + 4, nSwitch), s1 = snr(nSwitch + 12, nReset), s2 = snr(nReset, N);
    int gap = 0;   // silent sub-samples while the new table's frame loads (none: the old one plays on)
    for (int i = 2 * nSwitch; i < 2 * (nSwitch + 12); ++i)
        gap += rtl[i] == 0;
    std::printf("%-18s before %.1f dB, after the table switch %.1f dB (silent sub-samples %d), after the phase "
                "restart %.1f dB\n", "lvl_switch_phase", s0, s1, gap, s2);
    delete top;
    return (s0 > 65 && s1 > 65 && s2 > 65 && gap == 0) ? 0 : 1;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const std::string only = argc > 1 ? argv[1] : "";
    wt::Table T = wt::builtin("basic", 64, 2048, 256);
    wt::Table T2 = wt::builtin("pwm", 64, 2048, 256);   // a second table behind it (lvl_switch_phase)
    // SDRAM: the table's levels at word 0 (scaled like the s16 copy)
    mem.assign(T.words + T2.words + 4096, 0);
    for (int k = 0; k < T.levels; ++k)
        for (size_t i = 0; i < T.q[k].size(); ++i)
            mem[T.base[k] + i] = uint16_t(T.q[k][i]);
    for (int k = 0; k < T2.levels; ++k)
        for (size_t i = 0; i < T2.q[k].size(); ++i)
            mem[T.words + T2.base[k] + i] = uint16_t(T2.q[k][i]);
    std::vector<Scenario> scs;
    {
        Scenario s{"plain_saw", 300};
        s.osc[0].on = true;
        s.osc[0].f = 110;
        s.osc[0].pos = 0.66;
        scs.push_back(s);
    }
    {
        Scenario s{"smooth_low", 300};
        s.osc[0].on = true;
        s.osc[0].f = 41.2;
        s.osc[0].pos = 0.437;
        s.osc[0].smooth = true;
        scs.push_back(s);
    }
    {
        Scenario s{"unison4_high", 300};
        s.osc[0].on = true;
        s.osc[0].f = 880;
        s.osc[0].pos = 0.8;
        s.osc[0].uni = 4;
        s.osc[0].detune = 0.3;
        scs.push_back(s);
    }
    {
        Scenario s{"fm_b_from_a", 300};
        s.osc[0].on = true;
        s.osc[0].f = 110;
        s.osc[0].pos = 0.2;
        s.osc[1].on = true;
        s.osc[1].f = 220;
        s.osc[1].pos = 0.5;
        s.osc[1].w1 = W_FM;
        s.osc[1].a1 = 0.3;
        scs.push_back(s);
    }
    {
        Scenario s{"sync", 300};
        s.osc[0].on = true;
        s.osc[0].f = 98;
        s.osc[0].pos = 0.66;
        s.osc[0].w1 = W_SYNC;
        s.osc[0].a1 = 0.4;
        scs.push_back(s);
    }
    {
        Scenario s{"asym_bend", 300};
        s.osc[0].on = true;
        s.osc[0].f = 130;
        s.osc[0].pos = 0.3;
        s.osc[0].w1 = W_ASYM_P;
        s.osc[0].a1 = 0.5;
        s.osc[0].w2 = W_BEND_M;
        s.osc[0].a2 = 0.3;
        scs.push_back(s);
    }
    {
        Scenario s{"budget_3v_u4", 200};
        s.allVoices = true;
        for (int k = 0; k < 2; ++k) {
            s.osc[k].on = true;
            s.osc[k].f = 110 * (k + 1);
            s.osc[k].pos = 0.5;
            s.osc[k].smooth = true;
            s.osc[k].uni = 4;
            s.osc[k].detune = 0.2;
            s.osc[k].w1 = W_ASYM_P;
            s.osc[k].a1 = 0.3;
            s.osc[k].w2 = W_BEND_P;
            s.osc[k].a2 = 0.2;
        }
        scs.push_back(s);
    }
    int fails = 0;
    for (auto& sc : scs)
        if (only.empty() || only == sc.name)
            fails += run(sc, T);
    if (only.empty() || only == "lvl_switch_phase")
        fails += run_switch(T, T2, T.words);
    std::printf(fails ? "%d FAILED\n" : "ALL PASS\n", fails);
    return fails ? 1 : 0;
}
