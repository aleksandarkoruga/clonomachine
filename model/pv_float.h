// Floating-point reference of the phase-vocoder engine (Bungee "Basic" algorithm,
// rearranged the way the hardware runs it). Used to validate the algorithm against
// the Bungee library and as the accuracy reference for the fixed-point model.
//
// One call to grain() analyses one input frame centred on the playhead, modifies the
// spectrum and overlap-adds the synthesised grain; it returns H new output samples.
//   N = 8H analysis/transform length, Hann analysis window (length N, zero phase),
//   Hann synthesis window of length 4H, synthesis hop H.
// Pitch is applied by reading the input at `rate` input samples per frame sample
// (linear interpolation), the same way the grain engine reads.
#pragma once

#include <cstdint>
#include <vector>

namespace pvf {

struct Source {
    const float* x = nullptr;
    int64_t frames = 0;
    bool loop = false; // true: wrap around (sampler buffers), false: silence outside
    double at(double pos) const;
};

class Pv {
public:
    explicit Pv(int log2Hop = 9);
    int hop() const { return H; }
    void reset();
    // position: playhead (input samples) at the grain centre; speed: used only after a
    // reset (otherwise the hop is position - previous position); rate: read rate (pitch).
    void grain(const Source& src, double position, double speed, double rate, bool reset, float* out);

    bool transientSuppression = true;
    bool binPartials = false; // test: every bin its own partial (plain phase vocoder)

private:
    int log2H, H, N, n2;
    std::vector<double> wa, ws;
    std::vector<double> lapped;
    // previous grain
    std::vector<int16_t> pPhase, pRot;
    std::vector<float> pEnergy;
    bool pValid = false;
    double pPosition = 0, pErr = 0;
    int pHop = 0;
    // work
    std::vector<double> re, im;
    std::vector<int16_t> phase, rot;
    std::vector<float> energy;
    struct Partial { int peak, end; };
    std::vector<Partial> partials;
    void fft(bool inverse);
};

} // namespace pvf
