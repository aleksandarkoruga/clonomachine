// Float reference of the PMWave UGen (github.com/aleksandarkoruga/pmwave) as the
// 2x2 plate the FPGA gets: the same algorithm without SuperCollider (no SIMD, own RNG), for
// listening against the SC original and for checking the fixed-point model (reverb_model.h).
#pragma once
#include <cstdint>
#include <vector>

namespace pmw {

struct Params {
    float decay = 4.0f;    // T60 at DC, s
    float time = 0.3f;     // size 0..1 (0: 40 ms rails, 1: 0.1 ms)
    float glide = 0.2f;    // s
    float hfDecay = 1.0f;  // T60 at Nyquist, s
    float rand1 = 0.1f;    // length spread
    float rand2 = 0.2f;    // glide time spread
    float wander = 0.0f;   // slow length modulation depth
    int glideMode = 0;     // 0 lag, 1 constant rate
};

class Ref {
public:
    static constexpr int NX = 2, NY = 2, NJ = NX * NY, NR = NJ * 4;
    // starts at p (like a synth created with these arguments: no glide from the defaults)
    explicit Ref(double fs = 48000.0, uint32_t seed = 1, const Params& p = Params());
    void set(const Params& p);  // like the UGen's control inputs, any time
    void process(const float* in, float* outL, float* outR, int n);

private:
    struct Rng {
        uint32_t s;
        float frand() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return (s >> 8) * (1.0f / 16777216.0f); }
        float frand2() { return frand() * 2 - 1; }
    } rng_;
    double fs_;
    Params p_{};
    int bufLength_, mask_, w_ = 0;
    std::vector<float> rails_;  // [NR][bufLength]
    float lp_[NR]{}, wave_[NR]{}, p_j_[NJ]{};
    float logMin_, logMax_, minLength_, maxLength_, lossScale_;
    float logTarget_[NR]{}, targetLength_[NR]{}, logRemain_[NR]{}, lagCoef_[NR]{};
    float logRead_[NR]{}, length_[NR]{};
    int tapDelay_[NR]{};
    float tap_[4][NR]{}, gain_[NR]{}, pole_[NR]{};
    static constexpr int kLossTab = 256;
    float gainTab_[kLossTab + 1]{}, poleTab_[kLossTab + 1]{};
    float rand1_[NR]{}, rand2_[NR]{}, wPhase_[NR]{}, wInc_[NR]{}, wFrom_[NR]{}, wTo_[NR]{};
    float spread1_ = 0, wanderOct_ = 0;
    bool retarget_ = true;

    void updateTargets();
    void updateLag();
    void updateLoss();
    void updateGains();
    void updateLengths(const float* offset);
    float wanderValue(int R) const;
    void render(float in, float& l, float& r);
};

}  // namespace pmw
