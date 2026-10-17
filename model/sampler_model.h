// Bit-exact fixed-point golden model of the sampler engine (rtl/sampler_engine.v).
//
// Derived from BufGrainPlay.cpp with the HDL number formats of
// include/sampler_regs.h. This file *is* the arithmetic spec: the RTL must produce
// identical output samples for identical register write sequences.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "../include/sampler_regs.h"
#include "pv_model.h"

namespace smp {

struct Tables {
    uint32_t hann[1025];
    uint32_t exp2[257];
    bool load(const std::string& dir); // reads hann.hex / exp2.hex from rtl/
};

struct Grain {
    uint32_t winPhase = 0, winInc = 0;
    int64_t readPhase = 0; // Q32 samples, [0, frames<<32)
    int64_t readInc = 0;   // Q32 samples/sample
    uint32_t base = 0, frames = 128; // buffer the grain was started in
};

struct Slot {
    // registers
    uint32_t r[SMP_NUM_SLOT_REGS] = {};
    // pending strobes
    uint32_t strobe = 0;

    // state
    int64_t phasor = 0;     // Q32 samples
    uint32_t trigPhase = 0;
    uint32_t hop = 65536;   // u1.16
    uint32_t amp = 1u << 24; // Q24
    uint32_t rng = 0;
    uint32_t active = 0;
    Grain g[SMP_GRAINS_PER_SLOT];

    // gate env
    bool gate = false;
    uint32_t envLeft = 0;
    uint32_t level = 0; // 0..65536

    // stutter: counted retrigger pulses
    int64_t posLatchQ = 0;  // effective trig position, Q32 samples
    int64_t bgPhasor = 0;   // background phasor, Q32 samples (always advances)
    uint32_t stutLeft = 0, stutPh = 0;

    uint32_t gateState = 0;
    int32_t lastOut = 0;  // slot output after the latency delay and the algorithm mix
    int32_t gOut = 0;     // grain path output of this sample (before the delay)

    // for the phase vocoder (set by step)
    int64_t pvPh = 0;     // playhead used by this sample's onsets (after trig/seek/pin)
    bool pvRestart = false; // trig / retrig / SEEK / KILL this sample

    void reset(int idx);
    int32_t step(const std::vector<int16_t>& mem, const Tables& t);
    uint32_t xorshift();
};

// Phase-vocoder path of one slot (see SMP_LATENCY / SMP_R_ALGO).
struct PvSlot {
    bool active = false;
    uint32_t algo = 0;      // last ALGO value seen
    uint32_t xfStart = 0;   // crossfade starts at this sample
    uint32_t x = 0;         // crossfade position u0.16 (0 grain, 65536 PV)
    uint32_t nextC = 0;     // centre (output sample) of the next steady grain
    PvSlotState st;
    std::vector<int32_t> ring = std::vector<int32_t>(PV_RING, 0);
    std::vector<int32_t> dlOut = std::vector<int32_t>(SMP_LATENCY, 0);
    std::vector<uint32_t> dlLevel = std::vector<uint32_t>(SMP_LATENCY, 0);
};

class Engine {
public:
    explicit Engine(int nSlots);
    bool loadTables(const std::string& dir) { return tab.load(dir) && pv.loadTables(dir); }

    void write(uint32_t addr, uint32_t data);
    uint32_t status(uint32_t idx) const;
    // Produces one output sample (s24, Q8 sample units; mono, both DAC channels get it).
    int32_t step();       // left output; the right one is in outR
    int32_t outR = 0;

    std::vector<int16_t> mem; // SDRAM image, 16-bit words
    int nSlots;
    std::vector<Slot> slots;
    uint32_t master = 4096;
    uint64_t beat = 0;
    uint32_t beatInc = 0;
    uint32_t samples = 0;
    std::vector<PvSlot> pvs;
    PvCore pv;
    uint64_t pvJobs = 0;

private:
    Tables tab;
    int32_t mixSlot(int s);
    void pvIssue(int s, uint32_t c, int64_t pos, int64_t inc, bool reset, int win);
};

} // namespace smp
