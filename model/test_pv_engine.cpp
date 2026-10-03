// Engine-level phase-vocoder scenarios on the golden model (no RTL): renders WAVs for
// listening and checks levels / restart behaviour.
//
//   test_pv_engine in.wav outdir [rtl_dir]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "sampler_model.h"
#include "wav.h"

static constexpr double FS = 48014.32;

struct Rig {
    smp::Engine eng{2};
    uint32_t base = 0, frames = 0;
    double bufRate = 44100;
    void load(const wav::Audio& a) {
        bufRate = a.rate;
        frames = uint32_t(a.x.size());
        base = 64;
        eng.mem.assign(base + SMP_PREPAD + frames + SMP_POSTPAD + 64, 0);
        auto q = [](float v) { return int16_t(std::lrint(std::clamp(v * 32767.0, -32768.0, 32767.0))); };
        for (uint32_t i = 0; i < frames; ++i)
            eng.mem[base + SMP_PREPAD + i] = q(a.x[i]);
        for (uint32_t i = 0; i < SMP_PREPAD; ++i)
            eng.mem[base + i] = eng.mem[base + SMP_PREPAD + frames - SMP_PREPAD + i];
        for (uint32_t i = 0; i < SMP_POSTPAD; ++i)
            eng.mem[base + SMP_PREPAD + frames + i] = eng.mem[base + SMP_PREPAD + (i % frames)];
    }
    void w(int s, int r, uint32_t v) { eng.write(SMP_ADDR(s, r), v); }
    static uint32_t q24(double v) { return uint32_t(int32_t(std::lrint(v * 16777216.0))); }
    void setup(int s, double rate, double pitch, uint32_t algo) {
        w(s, SMP_R_BASE, base);
        w(s, SMP_R_FRAMES, frames);
        w(s, SMP_R_WIN_INC, uint32_t(4294967296.0 / (0.04 * FS)));
        w(s, SMP_R_OVERLAP, 4u << 24);
        w(s, SMP_R_AMP_TARGET, 16384);
        w(s, SMP_R_PHASOR_INC, q24(rate * bufRate / FS));
        w(s, SMP_R_PITCH, q24(pitch * bufRate / FS));
        w(s, SMP_R_ENV_DUR, uint32_t(FS * 30));
        w(s, SMP_R_ALGO, algo);
        w(s, SMP_R_CTRL, SMP_CTRL_ENABLE | SMP_CTRL_ENV_HOLD);
    }
    void trig(int s, double posFrac, int repeats = 0, int interval = 4800) {
        w(s, SMP_R_POS, uint32_t(posFrac * frames * 256.0));
        w(s, SMP_R_REPEATS, uint32_t(repeats));
        w(s, SMP_R_INTERVAL, uint32_t(interval));
        w(s, SMP_R_STROBE, SMP_STROBE_TRIG);
    }
};

struct Result {
    double rms = 0, peak = 0, maxStep = 0;
};

static Result render(Rig& rig, int n, const std::function<void(int)>& script, std::vector<float>& out) {
    Result r;
    out.resize(n);
    double sum = 0;
    int32_t prev = 0;
    for (int i = 0; i < n; ++i) {
        script(i);
        const int32_t y = rig.eng.step();
        out[i] = float(y / 256.0 / 32768.0);
        sum += double(out[i]) * out[i];
        r.peak = std::max(r.peak, std::abs(double(out[i])));
        r.maxStep = std::max(r.maxStep, std::abs(double(y - prev)) / 256.0 / 32768.0);
        prev = y;
    }
    r.rms = std::sqrt(sum / n);
    return r;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s in.wav outdir [rtl_dir]\n", argv[0]);
        return 1;
    }
    const std::string outDir = argv[2], rtl = argc > 3 ? argv[3] : "rtl";
    wav::Audio a;
    if (!wav::read(argv[1], a)) {
        std::fprintf(stderr, "cannot read %s\n", argv[1]);
        return 1;
    }
    struct Case {
        const char* name;
        double rate, pitch;
        uint32_t algo;
        std::function<void(Rig&, int)> script;
    };
    const int bar = int(FS * 60 / 172 * 4); // one bar at 172 bpm
    std::vector<Case> cases = {
        {"grain_half", 0.5, 1, SMP_ALGO_GRAIN, [](Rig& r, int i) { if (i == 0) r.trig(0, 0); }},
        {"pv_half", 0.5, 1, SMP_ALGO_PV, [](Rig& r, int i) { if (i == 0) r.trig(0, 0); }},
        {"pv_quarter_pitch+5", 0.25, 1.3348, SMP_ALGO_PV, [](Rig& r, int i) { if (i == 0) r.trig(0, 0); }},
        {"pv_cut_stutter", 1, 1, SMP_ALGO_PV, [bar](Rig& r, int i) {
             if (i % (bar / 16) == 0) {
                 const int k = (i / (bar / 16)) * 7 % 16;
                 r.trig(0, k / 16.0, (i / (bar / 16)) % 5 == 3 ? 3 : 0, bar / 64);
             }
         }},
        {"switch_live", 0.5, 1, SMP_ALGO_GRAIN, [bar](Rig& r, int i) {
             if (i == 0)
                 r.trig(0, 0);
             if (i > 0 && i % (bar / 2) == 0)
                 r.w(0, SMP_R_ALGO, (i / (bar / 2)) & 1 ? SMP_ALGO_PV : SMP_ALGO_GRAIN);
         }},
    };
    int fails = 0;
    {
        // clicks: 440 Hz sine, PV retriggers at random positions every 1/16 bar plus
        // algorithm switches; energy above 5 kHz per 512-sample block vs the sine
        wav::Audio sine;
        sine.rate = 48000;
        sine.x.resize(48000 * 2);
        for (size_t i = 0; i < sine.x.size(); ++i)
            sine.x[i] = float(0.5 * std::sin(2 * M_PI * 440.0 * double(i) / 48000.0));
        Rig rig;
        rig.eng.loadTables(rtl);
        rig.load(sine);
        rig.setup(0, 0.5, 1, SMP_ALGO_PV);
        std::vector<float> out;
        uint32_t rs = 12345;
        render(rig, 4 * bar, [&](int i) {
            if (i % (bar / 16) == 0) {
                rs = rs * 1664525u + 1013904223u;
                rig.trig(0, (rs >> 8) / 16777216.0);
            }
            if (i % bar == bar / 3)
                rig.w(0, SMP_R_ALGO, SMP_ALGO_GRAIN);
            if (i % bar == 2 * bar / 3)
                rig.w(0, SMP_R_ALGO, SMP_ALGO_PV);
        }, out);
        wav::write(outDir + "/engine_sine_clicks.wav", out, 48000);
        // per block: HF energy via a 4th-order difference (strongly attenuates 440 Hz)
        double worst = -200, sig = 0;
        for (size_t i = 0; i < out.size(); ++i)
            sig += double(out[i]) * out[i];
        sig /= out.size();
        for (size_t b = 4 * 512; b + 512 < out.size(); b += 512) {
            double e = 0;
            for (size_t i = b; i < b + 512; ++i) {
                const double d = out[i] - 4.0 * out[i - 1] + 6.0 * out[i - 2] - 4.0 * out[i - 3] + out[i - 4];
                e += d * d;
            }
            worst = std::max(worst, 10 * std::log10(e / 512 / sig + 1e-30));
        }
        // reference: the 4th difference of a 440 Hz sine is (2 sin(pi*440/48000))^4 = -61.3 dB
        std::printf("%-22s worst block 4th-difference energy %.1f dB re signal (pure sine -61.3 dB)\n", "sine_clicks", worst);
        if (worst > -40)
            ++fails;
    }
    for (auto& c : cases) {
        Rig rig;
        if (!rig.eng.loadTables(rtl)) {
            std::fprintf(stderr, "cannot load tables from %s\n", rtl.c_str());
            return 1;
        }
        rig.load(a);
        rig.setup(0, c.rate, c.pitch, c.algo);
        std::vector<float> out;
        const int n = 4 * bar;
        const Result r = render(rig, n, [&](int i) { c.script(rig, i); }, out);
        const std::string path = outDir + "/engine_" + c.name + ".wav";
        wav::write(path, out, 48000);
        std::printf("%-22s rms %.4f peak %.3f max step %.3f  pv grains %llu\n", c.name, r.rms, r.peak, r.maxStep,
            (unsigned long long)rig.eng.pvJobs);
        if (!(r.rms > 0.01))
            ++fails;
    }
    return fails ? 1 : 0;
}
