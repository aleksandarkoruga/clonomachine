// The synth firmware (fw/synth_ctl.c, synth_filt_ctl.c, synth_wt.c, scales.c) driving rtl/synth_top.v:
// the built-in tables built by the firmware's integer FFT (checked against model/wt_tables.cpp),
// put in the simulated SDRAM where the board keeps them, then notes and parameter changes as the
// MIDI / OSC handlers would make them, with a control tick every SY_TICK samples. Checks pitch,
// level, the release, the filter, scales and degree notes, glide; writes build/synth/fw_*.wav.
//
//   make synthfw && ./obj_sfw/Vsynth_top [scenario]
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "Vsynth_top.h"
#include "verilated.h"
#include "../model/wav.h"
#include "../model/wt_tables.h"
#include "../fw/scales.h"
#include "../fw/synth_ctl.h"
#include "../fw/synth_wt.h"

namespace {
Vsynth_top* top;
std::vector<uint16_t> mem;  // SDRAM words (32 MB)
std::deque<uint32_t> reqq;
int64_t serveAt = -1;
int wordIdx = 0;
uint64_t cyc = 0;
std::vector<float> out;
const uint32_t kBase = (32u << 20) / 2 - (8u << 20) / 2;  // the synth's 8 MB at the top (word address)
const uint32_t kFsQ16 = 48000u << 16;

void cycle() {
    const bool tick = cyc % 1024 == 0;
    top->tick = tick;
    top->req_ready = reqq.size() < 32;
    top->fill_valid = 0;
    if (!reqq.empty()) {
        if (serveAt < 0)
            serveAt = int64_t(cyc) + 60;
        if (int64_t(cyc) >= serveAt) {
            top->fill_valid = 1;
            top->fill_data = mem[(reqq.front() + wordIdx) & (mem.size() - 1)];
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
    if (tick)
        out.push_back(float((int32_t(top->out << 8) >> 8) / 8388608.0));
}
void hw_write(uint32_t addr, uint32_t data) {
    top->cmd_valid = 1;
    top->cmd_addr = addr;
    top->cmd_data = data;
    do
        cycle();
    while (!top->cmd_ready);
    top->cmd_valid = 0;
}
void samples(sy_state& s, int n) {  // n samples, a control tick every SY_TICK
    for (int i = 0; i < n; ++i) {
        const uint64_t until = (cyc / 1024 + 1) * 1024;
        while (cyc < until)
            cycle();
        if (out.size() % SY_TICK == 0)
            sy_tick(&s);
    }
}

// f0 of x[a, b) from the FFT peak, refined by the neighbours (Hann window)
double f0_of(const std::vector<float>& x, size_t a, size_t n) {
    std::vector<std::complex<double>> X(n);
    for (size_t i = 0; i < n; ++i)
        X[i] = x[a + i] * (0.5 - 0.5 * std::cos(2 * M_PI * i / n));
    wt::fft(X, false);
    size_t k = 2;
    for (size_t i = 2; i < n / 2; ++i)
        if (std::abs(X[i]) > std::abs(X[k]))
            k = i;
    const double l = std::abs(X[k - 1]), c = std::abs(X[k]), r = std::abs(X[k + 1]);
    const double d = 0.5 * (l - r) / (l - 2 * c + r);
    return (k + d) * 48000.0 / n;
}
// f0 of a sine-like x[a, a + n) from its rising zero crossings (interpolated)
double f0_zc(const std::vector<float>& x, size_t a, size_t n) {
    double first = -1, last = -1;
    int count = 0;
    for (size_t i = a + 1; i < a + n; ++i)
        if (x[i - 1] < 0 && x[i] >= 0) {
            const double t = (i - 1) + x[i - 1] / (x[i - 1] - x[i]);
            if (first < 0)
                first = t;
            else
                ++count;
            last = t;
        }
    return count > 0 ? count * 48000.0 / (last - first) : 0;
}
double rms(const std::vector<float>& x, size_t a, size_t n) {
    double s = 0;
    for (size_t i = a; i < a + n && i < x.size(); ++i)
        s += double(x[i]) * x[i];
    return std::sqrt(s / n);
}
double hf_ratio(const std::vector<float>& x, size_t a, size_t n, double fc) {  // energy above fc / total
    std::vector<std::complex<double>> X(n);
    for (size_t i = 0; i < n; ++i)
        X[i] = x[a + i] * (0.5 - 0.5 * std::cos(2 * M_PI * i / n));
    wt::fft(X, false);
    double hi = 0, all = 0;
    for (size_t i = 1; i < n / 2; ++i) {
        const double e = std::norm(X[i]);
        all += e;
        if (i * 48000.0 / n > fc)
            hi += e;
    }
    return hi / (all + 1e-30);
}

int fails = 0;
void check(bool ok, const char* what) {
    std::printf("  %-58s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok)
        ++fails;
}
}  // namespace

static sy_table g_tabs[SY_MAX_TABLES];
static int g_ntab;

// the firmware's built-in tables into mem, and a check of the builder against the model
static void build_tables() {
    wt_init();
    uint32_t a = kBase;
    std::vector<int16_t> x(WT_N0);
    for (int t = 0; t < wt_builtin_count(); ++t) {
        sy_table& T = g_tabs[g_ntab++];
        std::snprintf(T.name, sizeof T.name, "%s", wt_builtin_name(t));
        T.frames = uint16_t(wt_builtin_frames(t));
        wt_layout(a, T.frames, T.lvl);
        for (int f = 0; f < T.frames; ++f) {
            wt_builtin_frame(t, f, x.data());
            int16_t* dst[SY_LEVELS] = {};
            for (int k = 1; k < SY_LEVELS; ++k)
                dst[k] = reinterpret_cast<int16_t*>(&mem[T.lvl[k] + uint32_t(f) * wt_level_size(k)]);
            wt_build_frame(x.data(), dst);
        }
        a += wt_words(T.frames);
    }
    // the model's "basic" (same shapes, 64 frames): every level of a few frames
    wt::Table M = wt::builtin("basic", 64, 2048, 256);
    double worst = 1e9;
    for (int f : {0, 13, 21, 42, 63})
        for (int k = 1; k < SY_LEVELS; ++k) {
            const int N = wt_level_size(k);
            double se = 0, ss = 0;
            for (int i = 0; i < N; ++i) {
                const double ref = M.at(k, f, i) * 0.78;  // the firmware's headroom
                const double got = int16_t(mem[g_tabs[0].lvl[k] + uint32_t(f) * N + i]) / 32768.0;
                se += (got - ref) * (got - ref);
                ss += ref * ref;
            }
            const double snr = 10 * std::log10(ss / (se + 1e-30));
            if (std::getenv("WTDBG") && snr < 70)
                std::printf("  frame %d level %d: %.1f dB (ref[1] %.4f got %.4f, ref[5] %.4f got %.4f)\n", f, k, snr,
                            M.at(k, f, 1) * 0.78, int16_t(mem[g_tabs[0].lvl[k] + uint32_t(f) * N + 1]) / 32768.0,
                            M.at(k, f, 5) * 0.78, int16_t(mem[g_tabs[0].lvl[k] + uint32_t(f) * N + 5]) / 32768.0);
            worst = std::min(worst, snr);
        }
    std::printf("tables: %d built-in, %u words; firmware levels vs the model: worst %.1f dB\n", g_ntab,
                a - kBase, worst);
    check(worst > 70, "the integer FFT's levels match the model's (> 70 dB)");
}

static void fresh(sy_state& s) {
    sy_init(&s, kFsQ16, hw_write);
    sy_set_tables(&s, g_tabs, g_ntab);
    sy_hw_reset(&s);
}

static void set(sy_state& s, const char* name, double v) {
    const int p = sy_param_of(name);
    if (p < 0) {
        std::printf("unknown param %s\n", name);
        ++fails;
        return;
    }
    sy_set_natural(&s, p, int32_t(std::lrint(v * 65536)));
}

static void save(const char* name, size_t from) {
    std::system("mkdir -p ../build/synth");
    std::vector<float> y(out.begin() + from, out.end());
    wav::write(std::string("../build/synth/fw_") + name + ".wav", y, 48000);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const std::string only = argc > 1 ? argv[1] : "";
    top = new Vsynth_top;
    mem.assign(size_t(16) << 20, 0);
    scales_init();
    build_tables();
    top->rst = 1;
    for (int i = 0; i < 8; ++i)
        cycle();
    top->rst = 0;
    {   // scales and tunings (pure arithmetic): a maqam in quarter tones, Bohlen-Pierce, Partch, harmonic
        auto st = [](const char* sc, const char* tu, int deg) {
            int a = -1, b = -1;
            for (int i = 0; i < scales_count(); ++i)
                if (!std::strcmp(scale_name(i), sc)) a = i;
            for (int i = 0; i < tunings_count(); ++i)
                if (!std::strcmp(tuning_name(i), tu)) b = i;
            return (a < 0 || b < 0) ? -1e9 : scale_pitch_q16(a, b, 0, deg) / 65536.0 - 60;
        };
        std::printf("scales: %d scales, %d tunings; rast 2 = %.3f, bp chromatic 13 = %.3f, partch_o1 1 = %.3f,"
                    " harmonic chromatic 2 = %.3f, major -1 = %.3f\n", scales_count(), tunings_count(),
                    st("rast", "et12", 2), st("chromatic", "bp", 13), st("partch_o1", "partch", 1),
                    st("chromatic", "harmonic", 2), st("major", "et12", -1));
        check(std::fabs(st("rast", "et12", 2) - 3.5) < 1e-3, "rast (24 steps, et12 selected): degree 2 = 3.5 semitones");
        check(std::fabs(st("chromatic", "bp", 13) - 19.0196) < 1e-3, "Bohlen-Pierce: 13 steps = a 3:1");
        check(std::fabs(st("partch_o1", "partch", 1) - 12 * std::log2(9.0 / 8)) < 1e-3, "partch_o1 in Partch's 43: 9/8");
        check(std::fabs(st("chromatic", "harmonic", 2) - 12 * std::log2(3.0)) < 1e-3, "harmonic series: key 2 = the 3rd harmonic");
        check(std::fabs(st("major", "et12", -1) + 1) < 1e-3, "below the root: major degree -1 = B");
    }
    static sy_state s;
    fresh(s);
    samples(s, 64);

    auto run = [&](const char* name, std::function<void()> f) {
        if (!only.empty() && only != name)
            return;
        std::printf("%s\n", name);
        fresh(s);
        samples(s, 256);
        f();
    };

    run("saw_a3", [&] {  // the saw table, filter open: pitch, level, release to silence
        set(s, "aTable", 1);
        const size_t a = out.size();
        sy_note_on(&s, 57, 100, 0);
        samples(s, 24000);
        sy_note_off(&s, 57, 0);
        samples(s, 9600);
        save("saw_a3", a);
        const double f = f0_of(out, a + 4000, 16384), r = rms(out, a + 4000, 16384), tail = rms(out, out.size() - 2000, 2000);
        std::printf("  f0 %.2f Hz (220), rms %.3f, tail rms %.2e, writes %u\n", f, r, tail, s.writes);
        check(std::fabs(f - 220) < 0.3, "pitch of MIDI 57 = 220 Hz");
        check(r > 0.05 && r < 0.5, "level");
        check(tail < 1e-4, "silent after the release");
    });

    run("filter_lp", [&] {  // a low cutoff takes the highs away; res up stays bounded
        set(s, "aTable", 1);
        const size_t a = out.size();
        sy_note_on(&s, 45, 127, 0);
        samples(s, 8000);
        const double open = hf_ratio(out, a + 2000, 4096, 2000);
        set(s, "cutoff", 400);
        samples(s, 8000);
        const double closed = hf_ratio(out, out.size() - 4096, 4096, 2000);
        set(s, "res", 8);
        samples(s, 8000);
        const double r = rms(out, out.size() - 4096, 4096);
        save("filter_lp", a);
        std::printf("  energy above 2 kHz: open %.3f, 400 Hz %.5f; resonant rms %.3f\n", open, closed, r);
        check(closed < open * 0.05, "400 Hz low-pass removes the highs");
        check(r > 0.01 && r < 1.0, "resonant SVF stays bounded");
    });

    run("ladders", [&] {  // Moog and TB-303 slots: sound, bounded, with resonance
        set(s, "aTable", 1);
        const size_t a = out.size();
        for (int m = 1; m <= 2; ++m) {
            set(s, "slot1", m);
            set(s, "cutoff", 800);
            set(s, "res", 5);
            sy_note_on(&s, 48, 110, 0);
            samples(s, 12000);
            const double r = rms(out, out.size() - 6000, 6000);
            std::printf("  %s rms %.3f\n", m == 1 ? "moog" : "tb303", r);
            check(r > 0.005 && r < 1.0 && std::isfinite(r), m == 1 ? "Moog slot sounds, bounded" : "TB-303 slot sounds, bounded");
            sy_note_off(&s, 48, 0);
            samples(s, 4800);
        }
        save("ladders", a);
    });

    run("scale_degree", [&] {  // keys through the major scale; the harmony generator's degrees
        set(s, "aTable", 4);  // sine
        set(s, "scaleMap", 1);
        set(s, "scale", 0);
        const size_t a = out.size();
        sy_note_on(&s, 62, 100, 0);  // key 62 = degree 2 = E4
        samples(s, 12000);
        const double f1 = f0_zc(out, out.size() - 8192, 8192);
        sy_note_off(&s, 62, 0);
        samples(s, 2400);
        set(s, "scaleMap", 0);
        sy_note_on(&s, 64 + 4, 100, 1);  // degree 4 = G4, whatever scaleMap says
        samples(s, 12000);
        const double f2 = f0_zc(out, out.size() - 8192, 8192);
        sy_note_off(&s, 68, 1);
        set(s, "tuning", 1);  // just: degree 4 = 3/2 over C4
        sy_note_on(&s, 64 + 4, 100, 1);
        samples(s, 12000);
        const double f3 = f0_zc(out, out.size() - 8192, 8192);
        save("scale_degree", a);
        std::printf("  key 62 -> %.2f Hz (329.63), degree 4 -> %.2f Hz (392.00), just -> %.2f Hz (392.44)\n", f1, f2, f3);
        check(std::fabs(f1 - 329.63) < 0.5, "scaleMap: key 62 = degree 2 = E4");
        check(std::fabs(f2 - 392.00) < 0.5, "degree note 4 = G4");
        check(std::fabs(f3 - 261.626 * 1.5) < 0.5, "just tuning: the fifth is 3/2");
    });

    run("arp_offset", [&] {  // the note-offset CC moves a held note (scale degrees with scaleMap)
        set(s, "aTable", 4);
        set(s, "scaleMap", 1);
        sy_note_on(&s, 60, 100, 0);
        samples(s, 6000);
        const double f0 = f0_zc(out, out.size() - 4096, 4096);
        set(s, "noteOffset", 4);  // 4 degrees up: G4
        samples(s, 6000);
        const double f1 = f0_zc(out, out.size() - 4096, 4096);
        std::printf("  held C4 %.2f Hz, offset +4 degrees %.2f Hz\n", f0, f1);
        check(std::fabs(f0 - 261.63) < 0.5 && std::fabs(f1 - 392.0) < 0.5, "noteOffset steps the held note in degrees");
    });

    run("harm_offset", [&] {  // harmony offset mode: the chord's root on the MIDI notes
        set(s, "aTable", 4);
        set(s, "scaleMap", 1);
        sy_note_on(&s, 60, 100, 0);
        samples(s, 6000);
        sy_set_harm_offset(&s, 4);  // degree 4: G4, the held note follows
        samples(s, 6000);
        const double f1 = f0_zc(out, out.size() - 4096, 4096);
        sy_note_off(&s, 60, 0);
        samples(s, 2400);
        set(s, "scaleMap", 0);
        set(s, "scale", 1);  // minor: degree 2 = 3 semitones above the root
        sy_set_harm_offset(&s, 2);
        sy_note_on(&s, 57, 100, 0);  // A3 chromatic, + a minor third = C4
        samples(s, 6000);
        const double f2 = f0_zc(out, out.size() - 4096, 4096);
        sy_note_off(&s, 57, 0);
        samples(s, 2400);
        sy_set_harm_offset(&s, 0);
        set(s, "porta", 0.5);  // a long glide: a note that came with the trig jumps anyway
        set(s, "portaAlways", 1);
        sy_note_on(&s, 57, 100, 0);
        samples(s, 64);           // one tick into the note: the trig's offset arrives
        sy_set_harm_offset(&s, 2);
        samples(s, 2400);
        const double f3 = f0_zc(out, out.size() - 2048, 2048);
        sy_set_harm_offset(&s, -2); // held a while: glides (an old note), A3 - 4 semitones = F3
        samples(s, 960);
        const double f4 = f0_zc(out, out.size() - 960, 960);
        std::printf("  C4 + 4 degrees %.2f Hz (392.00), A3 + minor degree 2 %.2f Hz (261.63), "
                    "young note jumps %.2f Hz (261.63), old note glides %.2f Hz (between 174.6 and 261.6)\n",
                    f1, f2, f3, f4);
        check(std::fabs(f1 - 392.0) < 0.5, "offset 4 degrees moves the held note (scaleMap)");
        check(std::fabs(f2 - 261.63) < 0.5, "without scaleMap: the degree's interval");
        check(std::fabs(f3 - 261.63) < 0.8, "a note started with the trig takes the offset at once");
        check(f4 > 180 && f4 < 255, "a held note glides to the new chord");
        sy_note_off(&s, 57, 0);
        set(s, "porta", 0);
        set(s, "portaAlways", 0);
        sy_set_harm_offset(&s, 0);
    });

    run("glide_legato", [&] {  // mono legato: the second note glides, no new attack
        set(s, "aTable", 4);
        set(s, "mono", 1);
        set(s, "legato", 1);
        set(s, "porta", 0.2);
        const size_t a = out.size();
        sy_note_on(&s, 57, 100, 0);
        samples(s, 9600);
        sy_note_on(&s, 69, 100, 0);
        samples(s, 960);  // 20 ms into a 200 ms glide: in between
        const double mid = f0_zc(out, out.size() - 1024, 1024);
        samples(s, 19200);
        const double end = f0_zc(out, out.size() - 8192, 8192);
        save("glide_legato", a);
        std::printf("  20 ms in %.1f Hz, after %.2f Hz (440)\n", mid, end);
        check(mid > 230 && mid < 420, "gliding");
        check(std::fabs(end - 440) < 0.5, "arrives at 440 Hz");
    });

    run("poly_unison", [&] {  // three voices, unison 4 on both oscillators, warps; cycles stay in budget
        set(s, "aTable", 0);
        set(s, "bOn", 1);
        set(s, "bTable", 3);
        set(s, "aUnison", 4);
        set(s, "bUnison", 4);
        set(s, "aDetune", 0.3);
        set(s, "aWarp", 7);
        set(s, "aWarpAmt", 0.4);
        set(s, "bWarp", 14);  // FM from A
        set(s, "bWarpAmt", 0.2);
        set(s, "mod3Amt", 0.5);  // lfo2 -> aPos
        set(s, "aPos", 0.5);
        set(s, "filtOn", 1);
        set(s, "morphX", 0.5);
        set(s, "cutoff", 3000);
        const size_t a = out.size();
        const uint32_t w0 = s.writes;
        for (int n : {48, 55, 60})
            sy_note_on(&s, n, 100, 0);
        samples(s, 24000);
        const double r = rms(out, out.size() - 12000, 12000);
        save("poly_unison", a);
        const double wpt = double(s.writes - w0) / (24000 / SY_TICK);
        std::printf("  rms %.3f, osc cycles max %u, filter %u, %.1f register writes per tick\n", r, top->osc_cyc_max,
                    top->filt_cyc_max, wpt);
        check(r > 0.02 && r < 1.0, "three voices sound");
        check(top->osc_cyc_max < 1024 && top->filt_cyc_max < 1024, "within the cycle budget");
    });

    run("reese", [&] {  // the power-up patch: a bass line (legato, glide), level without clipping
        sy_patch_reese(&s);
        set(s, "aTable", 1);  // saw
        set(s, "bTable", 0);  // basic (the card's reese table isn't in the simulation)
        if (const char* e = std::getenv("REESE_SET")) {  // debugging: name=value,name=value
            std::string all = e;
            for (size_t p0 = 0; p0 < all.size();) {
                size_t c = all.find(',', p0);
                if (c == std::string::npos) c = all.size();
                const std::string kv = all.substr(p0, c - p0);
                const size_t eq = kv.find('=');
                if (eq != std::string::npos)
                    set(s, kv.substr(0, eq).c_str(), std::atof(kv.c_str() + eq + 1));
                p0 = c + 1;
            }
        }
        const size_t a = out.size();
        const int line[] = {40, 40, 43, 38, 40, 47, 45, 40};
        for (int i = 0; i < 8; ++i) {
            sy_note_on(&s, line[i], 110, 0);
            samples(s, 9000);
            if (i == 3 || i == 7) {  // a gap: a new attack
                sy_note_off(&s, line[i], 0);
                samples(s, 3000);
            } else
                sy_note_off(&s, line[i], 0), samples(s, 0);
        }
        samples(s, 12000);
        save("reese", a);
        double peak = 0;
        for (size_t i = a; i < out.size(); ++i)
            peak = std::max(peak, double(std::fabs(out[i])));
        const double r = rms(out, a, out.size() - a - 12000), hf = hf_ratio(out, a + 2000, 16384, 3000);
        std::printf("  rms %.3f, peak %.3f, energy above 3 kHz %.4f, voices active %d -> build/synth/fw_reese.wav\n",
                    r, peak, hf, sy_active_voices(&s));
        check(r > 0.05 && peak < 0.98, "reese: loud enough, no clipping");
        check(hf < 0.05, "reese: low-passed (a bass)");
    });

    std::printf(fails ? "%d FAILED\n" : "ALL PASS\n", fails);
    delete top;
    return fails ? 1 : 0;
}
