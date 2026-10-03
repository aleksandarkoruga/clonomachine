// Bit-exact fixed-point model of the phase-vocoder unit (rtl/pv_unit.v).
//
// The algorithm is Bungee "Basic" (see model/pv_float.cpp, validated against the
// library): per synthesis hop H one analysis frame of N = 8H samples centred on the
// playhead is transformed, the phases are advanced per spectral partial with
// transient suppression and vertical (group delay) scaling, and the inverse
// transform is overlap-added with a 4H synthesis window.
//
// Hardware formulation (this file is the arithmetic spec):
//   - frame read like a grain: 16-bit samples, linear interpolation at READ RATE
//     (= |PITCH|, pitch by resampling the input), zero-phase Hann window (Q16)
//   - real N-point FFT as N/2-point complex radix-2 DIF, 27-bit data, /2 per stage
//   - split / merge passes for the packed real transform, Q16 twiddles
//   - CORDIC (vectoring: magnitude + 16-bit phase; rotation: resynthesis)
//   - Mitchell log2 magnitude (Q11) for the partial / transient comparisons
//   - Bungee's partial enumeration, transient suppression and phase rules on int16
//   - synthesis weights from rtl/pv_win.hex (gain folded in) into a per-slot
//     overlap-add ring
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace smp {

constexpr int PV_LOG2H = 9;
constexpr int PV_H = 1 << PV_LOG2H;       // synthesis hop (output samples per grain)
constexpr int PV_N = 8 * PV_H;            // real transform length
constexpr int PV_M = PV_N / 2;            // complex FFT length
constexpr int PV_BINS = PV_M + 1;         // bins 0..N/2
constexpr int PV_CORDIC_ITER = 18;
constexpr int PV_INV_SCALE = 0;           // inverse FFT stages scaled by 1/2 (from stage 0)
constexpr int PV_FADE = 64;
constexpr int PV_DATA_BITS = 27;
constexpr int PV_TRANSIENT_T = 599;       // log2(sqrt(1.5)) * 2048: Bungee's energy ratio 1.5
constexpr int PV_RING = 4096;             // overlap-add ring per slot (power of two)

enum PvWin { PV_WIN_NORMAL = 0, PV_WIN_R0 = 1, PV_WIN_R1 = 2, PV_WIN_R2 = 3 };

struct PvTables {
    uint32_t sin[PV_N / 4 + 1];
    uint32_t atan[PV_CORDIC_ITER];
    uint32_t win[2 * PV_H + 1 + 3 * PV_H]; // ws | W0 | W1 | W2  (u0.18, gain folded in)
    uint32_t fadeOut[PV_FADE];             // u0.16
    bool load(const std::string& dir);
    // cos/sin of 2*pi*a/N, Q16, a taken modulo N
    void cs(uint32_t a, int32_t& c, int32_t& s) const;
};

// Per-slot state kept between grains (bins 0..N/2).
struct PvSlotState {
    uint16_t pL[PV_BINS + 2] = {};
    int16_t pPhase[PV_BINS] = {}, pRot[PV_BINS] = {};
    bool valid = false;
    int64_t pPos = 0;    // Q32 input samples, position of the previous grain
    int64_t pErr = 0;    // Q16 frame samples (Bungee positionError)
    int32_t pHop = 0;    // previous analysis hop, frame samples
};

struct PvJob {
    int64_t pos = 0;        // Q32 playhead at the grain centre, [0, frames << 32)
    int64_t inc = 0;        // Q32 playhead increment per output sample (hop after a reset)
    uint32_t rate = 1u << 24; // u8.24 read rate, (0, 4]
    uint32_t base = 0, frames = 128;
    bool reset = false;
    int win = PV_WIN_NORMAL;
};

struct PvStats {
    int64_t maxFwd = 0, maxInv = 0, maxSplit = 0, maxRot = 0;
    uint64_t sat = 0;
    int partials = 0;
};

class PvCore {
public:
    bool loadTables(const std::string& dir) { return tab.load(dir); }
    // Runs one grain: analysis, phase processing, resynthesis. Returns the synthesised
    // frame y(t), t = -2H .. 2H-1 (index t + 2H), before the synthesis window.
    void run(const PvJob& job, PvSlotState& st, const std::vector<int16_t>& mem);
    // Overlap-adds the last frame into a ring at output time c (ring index c + t).
    void accumulate(int32_t* ring, uint32_t c, int win) const;

    int32_t frame[4 * PV_H] = {};
    PvStats stats;
    bool transientSuppression = true;
    bool binPartials = false; // test: every bin its own partial (plain phase vocoder)
    const PvTables& tables() const { return tab; }

private:
    PvTables tab;
    int32_t zr[PV_M], zi[PV_M];
    int32_t mag[PV_BINS];
    uint16_t L[PV_BINS + 2];
    int16_t phase[PV_BINS], rotF[PV_BINS], rot[PV_BINS];
    struct Partial { int peak, end; int16_t delta; };
    std::vector<Partial> partials;

    int32_t sat27(int64_t v);
    void fft(bool inverse);
    void cordicVector(int32_t x, int32_t y, int32_t& m, int16_t& ph) const;
    void cordicRotate(int32_t m, int16_t ph, int32_t& x, int32_t& y) const;
};

uint16_t pvLog2(uint32_t m);

} // namespace smp
