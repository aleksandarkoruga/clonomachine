// Aliasing of the synth model's wavetable read: every MIDI note from 24 to 108 (or 96 with a
// warp) played steadily, the energy off the note's harmonic grid measured (20 Hz .. 20 kHz,
// relative to the note), worst and median over the notes, for several oscillator designs.
//
//   test_synth_osc [table] [warp amount (asym+)] [pm index (cycles, sine at the note)]
//   (table: a built-in name or a wavetable WAV; default saw)
//
// Build: g++ -O3 -std=c++20 model/test_synth_osc.cpp model/wt_tables.cpp -o build/synth/test_synth_osc
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <map>

#include "synth_analysis.h"
#include "synth_dsp.h"
#include "synth_osc.h"
#include "synth_warp.h"

using namespace synth;

int main(int argc, char** argv) {
    const char* spec = argc > 1 ? argv[1] : "saw";
    const double warp = argc > 2 ? std::atof(argv[2]) : 0, pm = argc > 3 ? std::atof(argv[3]) : 0;
    struct Design {
        const char* name;
        int rate, nmin;
        Interp in;
        bool aware;
    } ds[] = {
        {"1x hermite, levels down to 4", 1, 4, Interp::HERM, false},
        {"1x 8-tap sinc, levels >= 256", 1, 256, Interp::SINC8, false},
        {"2x linear, levels down to 4", 2, 4, Interp::LIN, false},
        {"2x hermite, levels >= 256", 2, 256, Interp::HERM, false},
        {"2x 8-tap sinc, levels >= 256", 2, 256, Interp::SINC8, false},
        {"2x 8-tap sinc, warp-aware levels", 2, 256, Interp::SINC8, true},
        {"4x 8-tap sinc, warp-aware levels", 4, 256, Interp::SINC8, true},
    };
    std::map<int, wt::Table> T;
    for (auto& d : ds)
        if (!T.count(d.nmin) && !wt::get(spec, T[d.nmin], d.nmin)) {
            std::fprintf(stderr, "can't load %s\n", spec);
            return 1;
        }
    const Halfband hb(71);
    std::printf("%s, warp asym+ %.2f, pm %.2f cycles: off-harmonic energy (dB re the note), MIDI 24..%d\n", spec, warp,
                pm, warp > 0 || pm > 0 ? 96 : 108);
    for (auto& d : ds) {
        const wt::Table& t = T[d.nmin];
        const double fs = 48000.0 * d.rate;
        std::vector<double> res;
        double worst = -999;
        int wn = 0;
        for (int note = 24; note <= (warp > 0 || pm > 0 ? 96 : 108); ++note) {
            const double f = 440 * std::exp2((note - 69) / 12.0);
            double slope = 1;
            warp_phase(W_ASYM_P, warp, 0.5, &slope);
            const double fmax = d.aware ? f * slope + pm * 2 * M_PI * f : f;
            const int k = wt::level_for(t, fmax, d.rate == 1 ? 24000 : 0.45 * fs);
            const int n = 1 << 15;
            std::vector<double> hi((n + 2048) * d.rate);
            double ph = 0;
            for (size_t i = 0; i < hi.size(); ++i) {
                double p = warp > 0 ? warp_phase(W_ASYM_P, warp, ph) : ph;
                p += pm * std::sin(2 * M_PI * ph);
                p -= std::floor(p);
                hi[i] = 0.5 * read_table(t, k, 0.5, p, d.in);
                ph += f / fs;
                ph -= std::floor(ph);
            }
            // decimate with the model's half-band chain
            std::vector<double> x(hi);
            for (int r = d.rate; r > 1; r /= 2) {
                Decim2 dec(hb);
                std::vector<double> y(x.size() / 2);
                for (size_t i = 0; i < y.size(); ++i)
                    y[i] = dec.push2(x[2 * i], x[2 * i + 1]);
                x.swap(y);
            }
            std::vector<float> out(x.begin(), x.end());
            const auto r = inharmonic(out, 512, n, 48000, f);
            res.push_back(r.ratioDb);
            if (r.ratioDb > worst)
                worst = r.ratioDb, wn = note;
        }
        std::sort(res.begin(), res.end());
        std::printf("  %-34s worst %6.1f (note %3d)  median %6.1f   %.2f MiB per 256-frame table\n", d.name, worst, wn,
                    res[res.size() / 2], t.words * 2 / 1048576.0 * 256 / t.frames);
    }
    return 0;
}
