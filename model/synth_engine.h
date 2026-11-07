// Model of the wavetable synth: 3 voices, 2 wavetable oscillators per voice (+ sub and noise),
// table position, warps, cross modulation, a quad morph filter per voice, envelopes, LFOs and a
// mod matrix. One implementation, two ways to run it:
//
//   reference  the sound it should have: double precision, oscillators and filter at 8 x 48 kHz,
//              full-size tables, modulation every sample, no cache
//   hardware   what the FPGA would do: an oscillator / filter rate of R x 48 kHz, s16 tables
//              with mip levels in SDRAM played from block-RAM frame caches, fixed-point word
//              lengths emulated by rounding at every register, modulation at a control rate
//
// and it counts what the hardware version costs (multiplies per sample, SDRAM lines, cache
// misses) as it runs. The filter (synth_qmf.h) is an AGPL-3.0 port: a binary built from these
// sources is under the AGPL.
#pragma once
#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "synth_cache.h"
#include "synth_dsp.h"
#include "synth_osc.h"
#include "synth_patch.h"
#include "wt_tables.h"

namespace synth {

struct Config {
    int rate = 2;               // oscillator + filter rate, x 48 kHz (1, 2, 4, 8)
    Interp interp = Interp::SINC8;
    int nmin = 256;             // smallest stored mip level
    int maxFrame = 1024;        // largest level played (caps the cache size); 2048 = all
    bool warpAwareLevels = true;
    int ctrlDiv = 16;           // modulation computed every ctrlDiv output samples (1 = every sample)
    bool fixed = true;          // emulate word lengths
    bool cache = true;          // frames through the block-RAM cache (else read directly)
    int cacheBufs = 3;
    double sdramLines = 6.0;    // lines per sample the grains leave free
    int voices = 3;
    bool stereo = false;        // stereo voices (unison width, pan, a filter per side); the board
                                // output is mono: the synth shares the melody channel
    uint32_t seed = 1;
    static Config reference() {
        Config c;
        c.rate = 8;
        c.nmin = 2048;
        c.maxFrame = 2048;
        c.ctrlDiv = 1;
        c.fixed = false;
        c.cache = false;
        return c;
    }
};

struct Stats {
    uint64_t samples = 0;
    double macsPerSample = 0;     // multiply-accumulates per output sample (all units)
    double tableReadsPerSample = 0;
    double fnsPerSample = 0;      // nonlinear function evaluations (tanh, divide, sqrt, ...)
    uint64_t frameLoads = 0, lagSamples = 0, oscSamples = 0;
    double maxLagFrames = 0, lagSum = 0;
    double sdramMean = 0, sdramPeak = 0;  // lines per sample
    uint64_t clips = 0;
    double peak = 0;
    std::map<std::string, uint64_t> macs;  // per block
};

class Engine {
public:
    Engine(const Config& cfg, const Song& song);
    ~Engine();
    // renders the whole song (+ release tail) at 48 kHz, stereo
    void render(std::vector<float>& L, std::vector<float>& R);
    const Stats& stats() const { return st_; }
    static std::vector<std::string> param_names();
    struct Impl;

private:
    std::unique_ptr<Impl> p_;
    Stats st_;
};

}  // namespace synth
