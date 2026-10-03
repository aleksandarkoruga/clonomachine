// Offline phase-vocoder renderer: runs the float reference (and the fixed-point model)
// over a WAV file at a constant speed/pitch, like Bungee's command-line tool, and
// optionally compares the result with another WAV (best lag, SNR).
//
//   pv_render in.wav out.wav [--speed S] [--pitch SEMITONES] [--start SEC] [--dur SEC]
//             [--loop] [--no-transient] [--compare ref.wav]
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "pv_float.h"
#include "pv_model.h"
#include "../include/sampler_regs.h"
#include "wav.h"

static void compare(const std::vector<float>& a, const std::vector<float>& b, int maxLag) {
    const int n = int(std::min(a.size(), b.size())) - 2 * maxLag;
    if (n <= 0) {
        std::printf("compare: too short\n");
        return;
    }
    double best = -1e30;
    int bestLag = 0;
    for (int lag = -maxLag; lag <= maxLag; ++lag) {
        double c = 0;
        for (int i = maxLag; i < maxLag + n; i += 4)
            c += double(a[i]) * b[i + lag];
        if (c > best) {
            best = c;
            bestLag = lag;
        }
    }
    double sig = 0, err = 0;
    for (int i = maxLag; i < maxLag + n; ++i) {
        const double d = double(a[i]) - b[i + bestLag];
        sig += double(b[i + bestLag]) * b[i + bestLag];
        err += d * d;
    }
    std::printf("compare: lag %d, SNR %.2f dB over %d samples\n", bestLag, 10 * std::log10(sig / (err + 1e-30)), n);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s in.wav out.wav [--speed S] [--pitch ST] [--start SEC] [--dur SEC] [--loop] [--no-transient] [--compare ref.wav]\n", argv[0]);
        return 1;
    }
    double speed = 1, pitchSt = 0, start = 0, dur = 0;
    bool loop = false, noTransient = false, fixed = false, binPartials = false;
    std::string fixedOut;
    std::string ref;
    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() { return i + 1 < argc ? argv[++i] : (char*)"0"; };
        if (a == "--speed")
            speed = std::atof(next());
        else if (a == "--pitch")
            pitchSt = std::atof(next());
        else if (a == "--start")
            start = std::atof(next());
        else if (a == "--dur")
            dur = std::atof(next());
        else if (a == "--loop")
            loop = true;
        else if (a == "--no-transient")
            noTransient = true;
        else if (a == "--bin-partials")
            binPartials = true;
        else if (a == "--fixed") {
            fixed = true;
            fixedOut = next();
        } else if (a == "--compare")
            ref = next();
    }
    wav::Audio in;
    if (!wav::read(argv[1], in)) {
        std::fprintf(stderr, "cannot read %s\n", argv[1]);
        return 1;
    }
    const int64_t s0 = int64_t(start * in.rate);
    int64_t len = int64_t(in.x.size()) - s0;
    if (dur > 0)
        len = std::min<int64_t>(len, int64_t(dur * in.rate));
    std::vector<float> x(in.x.begin() + s0, in.x.begin() + s0 + len);

    pvf::Pv pv;
    pv.transientSuppression = !noTransient;
    pv.binPartials = binPartials;
    const int H = pv.hop();
    const double rate = std::pow(2.0, pitchSt / 12.0);
    pvf::Source src{x.data(), int64_t(x.size()), loop};
    std::vector<float> out;
    std::vector<float> chunk(H);
    const double hop = speed * H * rate;
    double pos = (speed < 0 ? double(x.size() - 1) : 0.0) - hop; // Bungee restart + preroll
    bool reset = true;
    const int64_t outLen = int64_t(std::abs(double(x.size()) / speed / rate)) + 8 * H;
    while (int64_t(out.size()) < outLen) {
        pv.grain(src, pos, speed, rate, reset, chunk.data());
        out.insert(out.end(), chunk.begin(), chunk.end());
        pos += hop;
        reset = false;
    }
    // drop the 2H output latency so that output sample 0 = input position 0
    out.erase(out.begin(), out.begin() + std::min<size_t>(out.size(), size_t(2 * H)));

    if (fixed) {
        // fixed-point model: buffer in a 16-bit memory image with the engine's pads,
        // looping reads, same grain positions as the float run above
        const int64_t fr = int64_t(x.size());
        std::vector<int16_t> mem(fr + SMP_PREPAD + SMP_POSTPAD);
        auto q = [](float v) { double s = v * 32768.0; return int16_t(s > 32767 ? 32767 : s < -32768 ? -32768 : std::lround(s)); };
        for (int64_t i = 0; i < fr; ++i)
            mem[SMP_PREPAD + i] = q(x[i]);
        for (int i = 0; i < SMP_PREPAD; ++i)
            mem[i] = mem[SMP_PREPAD + ((fr - SMP_PREPAD + i) % fr + fr) % fr];
        for (int i = 0; i < SMP_POSTPAD; ++i)
            mem[SMP_PREPAD + fr + i] = mem[SMP_PREPAD + (i % fr)];
        smp::PvCore core;
#ifndef PV_RTL_DIR
#define PV_RTL_DIR "../rtl"
#endif
        if (!core.loadTables(PV_RTL_DIR) && !core.loadTables("rtl")) {
            std::fprintf(stderr, "cannot load rtl/pv_*.hex\n");
            return 1;
        }
        core.transientSuppression = !noTransient;
        core.binPartials = binPartials;
        smp::PvSlotState st;
        std::vector<int32_t> ring(smp::PV_RING, 0);
        std::vector<float> fo;
        const int64_t F = fr << 32;
        double p = (speed < 0 ? double(fr - 1) : 0.0) - hop;
        for (int64_t k = 0; int64_t(fo.size()) < int64_t(out.size()) + 2 * H; ++k) {
            smp::PvJob j;
            double pw = std::fmod(p, double(fr));
            if (pw < 0)
                pw += double(fr);
            j.pos = int64_t(std::llround(pw * 4294967296.0)) % F;
            j.inc = int64_t(std::llround(speed * rate * 4294967296.0));
            j.rate = uint32_t(std::lround(rate * 16777216.0));
            j.base = 0;
            j.frames = uint32_t(fr);
            j.reset = k == 0;
            core.run(j, st, mem);
            const uint32_t c = uint32_t(k * H);
            core.accumulate(ring.data(), c, smp::PV_WIN_NORMAL);
            // samples before c - H are complete
            for (int i = 0; i < H; ++i) {
                const uint32_t t = c - 2 * H + i;
                int32_t& v = ring[t & (smp::PV_RING - 1)];
                if (k >= 2)
                    fo.push_back(float(v / 256.0 / 32768.0));
                v = 0;
            }
            p += hop;
        }
        fo.resize(out.size());
        wav::write(fixedOut, fo, in.rate);
        std::printf("fixed: %s; max fwd %lld inv %lld split %lld rot %lld, saturations %llu\n", fixedOut.c_str(),
            (long long)core.stats.maxFwd, (long long)core.stats.maxInv, (long long)core.stats.maxSplit,
            (long long)core.stats.maxRot, (unsigned long long)core.stats.sat);
        std::printf("fixed vs float ");
        compare(fo, out, 1024);
    }
    wav::write(argv[2], out, in.rate);
    std::printf("%s: %zu samples, speed %.3f, pitch %.2f st\n", argv[2], out.size(), speed, pitchSt);
    if (!ref.empty()) {
        wav::Audio r;
        if (!wav::read(ref, r)) {
            std::fprintf(stderr, "cannot read %s\n", ref.c_str());
            return 1;
        }
        compare(out, r.x, 4096);
    }
    return 0;
}
