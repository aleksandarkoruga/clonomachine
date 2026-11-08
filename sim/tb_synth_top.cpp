// rtl/synth_top.v end to end: one voice playing a band-limited saw through the oscillator unit, the
// filter engine (slot 0: clean SVF low pass, other slots silent by weight), the mixer and the
// half-band decimator; the output is checked for level and for energy off the note's harmonics
// (aliasing / noise) and written to build/synth/top_<scenario>.wav.
//
//   make synthtop && ./obj_st/Vsynth_top
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <string>
#include <vector>

#include "Vsynth_top.h"
#include "verilated.h"
#include "../model/synth_analysis.h"
#include "../model/synth_ucode.h"
#include "../model/wav.h"
#include "../model/wt_tables.h"

using namespace synth_ucode;

namespace {
Vsynth_top* top;
std::vector<uint16_t> mem;
std::deque<uint32_t> reqq;
int64_t serveAt = -1;
int wordIdx = 0;
uint64_t cyc = 0;

void cycle(bool tick = false) {
    top->tick = tick;
    top->req_ready = reqq.size() < 32;
    top->fill_valid = 0;
    if (!reqq.empty()) {
        if (serveAt < 0)
            serveAt = int64_t(cyc) + 60;
        if (int64_t(cyc) >= serveAt) {
            top->fill_valid = 1;
            top->fill_data = mem[(reqq.front() + wordIdx) % mem.size()];
        }
    }
    top->eval();
    const bool rq = top->req_valid && top->req_ready;
    const uint32_t ra = top->req_addr;
    const bool fv = top->fill_valid;
    top->clk = 0;
    top->eval();
    top->clk = 1;
    top->eval();
    ++cyc;
    if (rq)
        reqq.push_back(ra);
    if (fv && ++wordIdx == 64) {
        wordIdx = 0;
        reqq.pop_front();
        serveAt = -1;
    }
}
void cmd(int addr, uint32_t data) {
    top->cmd_valid = 1;
    top->cmd_addr = addr;
    top->cmd_data = data;
    do
        cycle();
    while (!top->cmd_ready);
    top->cmd_valid = 0;
}
int32_t q24(double v) { return int32_t(std::lrint(v * 16777216.0)); }
}  // namespace

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vsynth_top;
    wt::Table T = wt::builtin("saw", 4, 2048, 256);
    mem.assign(T.words + 4096, 0);
    for (int k = 0; k < T.levels; ++k)
        for (size_t i = 0; i < T.q[k].size(); ++i)
            mem[T.base[k] + i] = uint16_t(T.q[k][i]);
    top->rst = 1;
    for (int i = 0; i < 8; ++i)
        cycle();
    top->rst = 0;
    const double f = 110.0, fsH = 96000.0;
    // oscillator A of voice 0
    int L = wt::level_for(T, f, 43200);
    while (T.size(L) > 1024)
        ++L;
    cmd(0, uint32_t(std::llrint(f / fsH * 4294967296.0)));           // inc[0]
    cmd(4, 65536);                                                      // ugain[0]
    cmd(8, 0);                                                          // pos: frame 0
    cmd(9, T.base[L] | uint32_t(__builtin_ctz(T.size(L))) << 25);
    cmd(15, 65536);                                                     // level
    cmd(16, T.frames - 1);
    cmd(10, 1 << 12 | 1 << 13 | 1 << 14);                               // to filter, fresh, on
    for (int v = 0; v < 3; ++v) {
        cmd(0x100 | v << 4 | 1, 0);
        cmd(0x100 | v << 4 | 2, 0);
        cmd(0x100 | v << 4 | 3, 0);
    }
    // filter engine: constants, slot programs, slot 0 = clean SVF low pass at 12 kHz, Q 0.707
    const double cm = 1 - std::exp(-1 / (0.0045 * fsH)), ca = 1 - std::exp(-1 / (0.045 * fsH));
    const std::pair<int, double> cst[] = {{C_ZERO, 0}, {C_ONE, 1}, {C_TWO, 2}, {C_HALF, 0.5}, {C_EPS, 1e-6},
                                          {C_GMIN, 0.1}, {C_GMAX, 15}, {C_CM, cm}, {C_CA, ca}, {C_C95, 0.95}};
    for (auto& [r, v] : cst)
        cmd(1 << 11 | 1 << 9 | (r - 32), uint32_t(q24(v)));
    for (int t = 0; t < 4; ++t)
        cmd(1 << 11 | 2 << 9 | t, SVF_START);
    const double g = std::tan(M_PI * 12000 / fsH), R = 1 / (2 * 0.707), h = 1 / (1 + 2 * R * g + g * g);
    const std::pair<int, double> sv[] = {{SVF_comp, 1.0}, {SVF_k1, 2 * R + g}, {SVF_g, g}, {SVF_h, h},
                                         {SVF_cl, 1}, {SVF_cb, 0}, {SVF_ch, 0}};
    for (int t = 0; t < 4; ++t)
        for (int l = 0; l < 3; ++l)
            for (auto& [r, v] : sv)
                cmd(1 << 11 | t << 7 | l << 5 | r, uint32_t(q24(v)));
    // mixer: voice 0 slot 0 weight 1, the rest 0; voice 0 gain 1
    for (int i = 0; i < 12; ++i)
        cmd(2 << 11 | i, i == 0 ? 65536 : 0);
    cmd(2 << 11 | 16, 65536);
    // run
    const int N = 1 << 15, pre = 4000;
    std::vector<float> out;
    unsigned lagStart = 0;
    for (int n = 0; n < N + pre; ++n) {
        cycle(true);
        for (int c = 1; c < 1024; ++c)
            cycle();
        out.push_back(float((int32_t(top->out << 8) >> 8) / 8388608.0));
        if (n == pre)
            lagStart = top->lag;
    }
    std::vector<float> steady(out.begin() + pre, out.end());
    double rms = 0;
    for (float v : steady)
        rms += double(v) * v;
    rms = std::sqrt(rms / steady.size());
    const auto r = synth::inharmonic(steady, 0, size_t(1) << 14, 48000, f);  // a power of two
    std::system("mkdir -p ../build/synth");
    wav::write("../build/synth/top_saw110.wav", out, 48000);
    if (const char* p = std::getenv("STDUMP")) {   // the raw s24 output, for bit-exact comparisons
        if (FILE* f = std::fopen(p, "wb")) {
            for (float v : out) {
                const int32_t s = int32_t(std::lrint(double(v) * 8388608.0));
                std::fwrite(&s, 4, 1, f);
            }
            std::fclose(f);
        }
    }
    // a saw of amplitude A has RMS A / sqrt(3); the table's band-limited saw peaks near 1 x qscale
    std::printf("synth_top saw 110 Hz: rms %.3f (%.1f dBFS), off-harmonic energy %.1f dB, worst bin %.1f dB at %.0f Hz, "
                "osc cycles max %u, filter cycles max %u, lag after start %u\n",
                rms, 20 * std::log10(rms + 1e-12), r.ratioDb, r.worstDb, r.worstHz, top->osc_cyc_max,
                top->filt_cyc_max, top->lag - lagStart);
    const bool ok = rms > 0.1 && r.ratioDb < -60 && top->osc_cyc_max < 1024 && top->filt_cyc_max < 1024;
    std::printf(ok ? "PASS\n" : "FAIL\n");
    delete top;
    return ok ? 0 : 1;
}
