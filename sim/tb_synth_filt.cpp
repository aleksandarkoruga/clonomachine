// SPDX-License-Identifier: AGPL-3.0-or-later
// The coefficient formulas (coefs()) follow model/synth_qmf.h, derived from QuadMorphFilter
// (https://github.com/OTODESK4193/QuadMorphFilter), GNU AGPL v3; this file is under the AGPL-3.0
// as well (the rest of this repository is MIT, see LICENSE).
//
// rtl/synth_filt.v against a floating-point interpreter of the same microcode
// (scripts/synth_asm.py -> rtl/sy_ucode.hex): three voices of saw input, the four slots' programs,
// coefficients and constants written as the soft core would, outputs compared after the start-up.
// The interpreter uses exact tanh / 1/x and no saturation, so the comparison measures the engine's
// fixed point (Q8.24, table functions) as well as its correctness.
//
//   make synthfilt && ./obj_sf/Vsynth_filt [scenario]
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "Vsynth_filt.h"
#include "verilated.h"
#include "../model/synth_ucode.h"

using namespace synth_ucode;

namespace {
Vsynth_filt* top;
const double fsP = 96000.0;
std::vector<uint64_t> rom;

void tick() {
    top->clk = 0;
    top->eval();
    top->clk = 1;
    top->eval();
}
void wr(int a, uint32_t d) {
    top->we = 1;
    top->waddr = a;
    top->wdata = d;
    tick();
    top->we = 0;
}
int32_t q24(double v) {
    const double s = std::nearbyint(v * 16777216.0);
    return int32_t(std::fmax(-2147483648.0, std::fmin(2147483647.0, s)));
}

struct Interp {  // one instance per (thread, lane)
    double r[64] = {};
    double run(int start, double x, double& y) {
        for (int pc = start;; ++pc) {
            const uint64_t w = rom[pc];
            const int op = int(w >> 32 & 15), d = int(w >> 26 & 63), a = int(w >> 20 & 63), b = int(w >> 14 & 63),
                      c = int(w >> 8 & 63), imm = int(w & 255);
            auto R = [&](int i) { return r[i]; };
            switch (op) {
            case 1: r[d] = R(c) + R(a) * R(b); break;
            case 2: r[d] = R(c) - R(a) * R(b); break;
            case 3: r[d] = R(a) + R(b); break;
            case 4: r[d] = R(a) - R(b); break;
            case 5: r[d] = imm == 0 ? std::tanh(R(a)) : (R(a) > 0 ? std::fmin(128.0, 1 / R(a)) : 128.0); break;
            case 6: r[d] = std::fmax(R(a), R(b)); break;
            case 7: r[d] = std::fmin(R(a), R(b)); break;
            case 8: r[d] = x; break;
            case 9: y = R(a); break;
            case 10: return y;
            case 12: r[d] = std::fabs(R(a)); break;
            default: break;
            }
        }
    }
};

struct Slot {
    int model;  // 0 svf, 1 moog, 2 tb303, 8 tanh test (x * 16 in), 9 1/x test (|x| * 4 + 0.05)
    double fc, res;
};

// the instance coefficients (per program register map), from cutoff and resonance like synth_qmf.h
std::vector<std::pair<int, double>> coefs(const Slot& s) {
    std::vector<std::pair<int, double>> v;
    const double g = std::tan(M_PI * std::fmin(s.fc, 0.45 * fsP) / fsP), G = g / (1 + g);
    auto jmap = [](double x, double a, double b, double c, double d) { return c + (d - c) * (x - a) / (b - a); };
    if (s.model == 8)
        return {{T_TANH_sc, 16.0}};
    if (s.model == 9)
        return {{T_RECIP_sc, 4.0}, {T_RECIP_of, 2.05}};
    if (s.model == 0) {
        const double R = 1 / (2 * s.res), h = 1 / (1 + 2 * R * g + g * g);
        v = {{SVF_comp, 1 + 0.1 * s.res}, {SVF_k1, 2 * R + g}, {SVF_g, g}, {SVF_h, h},
             {SVF_cl, 1}, {SVF_cb, 0}, {SVF_ch, 0}};
    } else if (s.model == 2) {
        const double k = std::clamp(jmap(s.res, 0.1, 10, 0, 4.2), 0.0, 4.2), G4 = G * G * G * G;
        const double hgt = std::tan(M_PI * 8 / fsP);
        const double kn = std::clamp(jmap(s.res, 0.1, 10, 0, 1), 0.0, 1.0);
        v = {{TB303_comp, 1 + 0.2 * k}, {TB303_inv1pg, 1 / (1 + g)}, {TB303_G, G}, {TB303_G4, G4}, {TB303_k, k},
             {TB303_invn, 1 / (1 + k * G4)}, {TB303_hg, hgt / (1 + hgt)},
             {TB303_byp, kn > 0.85 ? std::clamp((kn - 0.85) * 10, 0.0, 1.0) : 0.0}, {TB303_agc, 1.0}};
    } else {
        const double k = jmap(s.res, 0.1, 10, 0, 4.5), G4 = G * G * G * G, inv = 1 / (1 + k * G4);
        const double kn = k / 4.5;
        v = {{MOOG_comp, 1 + 0.5 * k}, {MOOG_inv1pg, 1 / (1 + g)}, {MOOG_G, G}, {MOOG_G2, G * G}, {MOOG_G3, G * G * G},
             {MOOG_k, k}, {MOOG_inv, inv}, {MOOG_kg4, k * G4}, {MOOG_kg4i, k * G4 * inv},
             {MOOG_byp, kn > 0.85 ? std::clamp((kn - 0.85) * 10, 0.0, 1.0) : 0.0}, {MOOG_agc, 1.0}};
    }
    return v;
}
int start_of(int m) {
    return m == 0 ? SVF_START : m == 1 ? MOOG_START : m == 8 ? T_TANH_START : m == 9 ? T_RECIP_START : TB303_START;
}

struct Scenario {
    const char* name;
    std::array<Slot, 4> slot;
    int samples;
};
}  // namespace

static int run(const Scenario& sc) {
    top = new Vsynth_filt;
    top->rst = 1;
    for (int i = 0; i < 4; ++i)
        tick();
    top->rst = 0;
    // constants
    const double cm = 1 - std::exp(-1 / (0.0045 * fsP)), ca = 1 - std::exp(-1 / (0.045 * fsP));
    const std::pair<int, double> cst[] = {{C_ZERO, 0}, {C_ONE, 1}, {C_TWO, 2}, {C_HALF, 0.5}, {C_EPS, 1e-6},
                                          {C_GMIN, 0.1}, {C_GMAX, 15}, {C_CM, cm}, {C_CA, ca}, {C_C95, 0.95}};
    Interp ip[4][3];
    for (auto& [r, v] : cst) {
        wr(1 << 9 | (r - 32), uint32_t(q24(v)));
        for (auto& t : ip)
            for (auto& l : t)
                l.r[r] = v;
    }
    for (int t = 0; t < 4; ++t) {
        wr(2 << 9 | t, start_of(sc.slot[t].model));
        for (int l = 0; l < 3; ++l)
            for (auto& [r, v] : coefs(sc.slot[t])) {
                wr(t << 7 | l << 5 | r, uint32_t(q24(v)));
                ip[t][l].r[r] = v;
            }
    }
    // saw inputs per voice, 0.5 peak
    const double f0[3] = {55.0, 82.4, 146.8};
    double ph[3] = {0, 0, 0};
    std::vector<double> rtl, ref;
    unsigned cmax = 0;
    std::vector<std::array<double, 24>> refq;
    for (int n = 0; n < sc.samples; ++n) {
        std::array<double, 24> r{};
        for (int sub = 0; sub < 2; ++sub)
            for (int l = 0; l < 3; ++l) {
                const double x = 0.5 * (1 - 2 * ph[l]);
                ph[l] += f0[l] / fsP;
                ph[l] -= std::floor(ph[l]);
                top->in_we = 1;
                top->in_a = sub << 2 | l;
                top->in_d = uint32_t(q24(x));
                tick();
                top->in_we = 0;
                for (int t = 0; t < 4; ++t) {
                    double y = 0;
                    ip[t][l].run(start_of(sc.slot[t].model), x, y);
                    r[sub * 12 + t * 3 + l] = y;
                }
            }
        refq.push_back(r);
        top->start = 1;
        tick();
        top->start = 0;
        int c = 1;
        while (top->busy && c < 3000)
            tick(), ++c;
        cmax = std::max<unsigned>(cmax, c);
        // the finished run is readable now that a new one hasn't started: it sits in the other half
        // after the next start, so read after one more start. Here: start toggles half, the run
        // wrote yb[half]; y_ra reads yb[~half] = the run before -> compare with refq[n - 1].
        if (n >= 1) {
            for (int i = 0; i < 24; ++i) {
                const int sub = i / 12, t = (i % 12) / 3, l = i % 3;
                top->y_ra = sub << 4 | t << 2 | l;
                tick();
                tick();
                rtl.push_back(int32_t(top->y_q) / 16777216.0);
                ref.push_back(refq[n - 1][i]);
            }
        }
        while (c < 1024)
            tick(), ++c;
    }
    if (std::getenv("SFDUMP"))
        for (size_t i = 24 * 10; i < 24 * 12; ++i)
            std::printf("  [%zu] sub %zu slot %zu lane %zu rtl %.6f ref %.6f\n", i / 24, (i % 24) / 12, (i % 12) / 3, i % 3,
                        rtl[i], ref[i]);
    if (const char* d = std::getenv("SFRAW"))  // the RTL's outputs, raw, for bit-exact comparisons
        if (FILE* f = std::fopen((std::string(d) + "/" + sc.name + ".raw").c_str(), "wb")) {
            for (double v : rtl) {
                const int32_t q = int32_t(std::llrint(v * 16777216.0));
                std::fwrite(&q, 4, 1, f);
            }
            std::fclose(f);
        }
    // per slot SNR after 20 ms
    const size_t from = size_t(sc.samples / 4) * 24;
    double worst = 1e9;
    std::printf("%-16s cycles max %4u (budget 1024)  SNR per slot:", sc.name, cmax);
    for (int t = 0; t < 4; ++t) {
        double s = 0, e = 0;
        for (size_t i = from; i < ref.size(); ++i)
            if (int((i % 24) % 12 / 3) == t) {
                s += ref[i] * ref[i];
                e += (rtl[i] - ref[i]) * (rtl[i] - ref[i]);
            }
        const double snr = e > 0 ? 10 * std::log10(s / e) : 200;
        worst = std::min(worst, snr);
        const int m = sc.slot[t].model;
        std::printf("  %s %5.1f dB", m == 0 ? "svf" : m == 1 ? "moog" : m == 8 ? "tanh" : m == 9 ? "1/x" : "303", snr);
    }
    std::printf("\n");
    delete top;
    return (worst > 50 && cmax < 1024) ? 0 : 1;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const std::string only = argc > 1 ? argv[1] : "";
    FILE* f = std::fopen("../rtl/sy_ucode.hex", "r");
    char line[64];
    while (f && std::fgets(line, sizeof line, f))
        rom.push_back(std::stoull(line, nullptr, 16));
    if (f)
        std::fclose(f);
    std::vector<Scenario> scs = {
        {"fn", {{{8, 0, 0}, {9, 0, 0}, {8, 0, 0}, {9, 0, 0}}}, 400},
        {"svf", {{{0, 800, 2}, {0, 600, 5}, {0, 1200, 4}, {0, 3000, 0.707}}}, 400},
        {"mixed", {{{0, 800, 2}, {2, 600, 5}, {1, 1200, 4}, {0, 3000, 0.707}}}, 2000},
        {"all_tb303", {{{2, 400, 6}, {2, 900, 3}, {2, 2000, 8}, {2, 250, 1}}}, 2000},
        {"all_moog_res", {{{1, 500, 9}, {1, 1500, 7}, {1, 4000, 5}, {1, 300, 2}}}, 2000},
    };
    int fails = 0;
    for (auto& sc : scs)
        if (only.empty() || only == sc.name)
            fails += run(sc);
    std::printf(fails ? "%d FAILED\n" : "ALL PASS\n", fails);
    return fails ? 1 : 0;
}
