// Float reference of PMWave as a 2x2 plate, see pmwave_ref.h. A transcription of
// PMWave::_render and its helpers (plain loops), GRID 2, 40 ms rails.
#include "pmwave_ref.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace pmw {

namespace {
constexpr double kMaxRailTime = 0.040, kMinRailTime = 0.0001;
constexpr float kMinRailSamples = 4.f, kMaxSlew = 1.f;
constexpr float kWanderRateMin = 0.1f, kWanderRateMax = 0.4f;
// pickups and input as in the UGen: input at (0,0), outputs at (1, NY/2) and (0, NY/2)
constexpr int kInput = 0, kPickupL = 1 * Ref::NY + Ref::NY / 2, kPickupR = 0 * Ref::NY + Ref::NY / 2;
inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
}  // namespace

Ref::Ref(double fs, uint32_t seed) : fs_(fs) {
    rng_.s = seed ? seed : 1;
    const double longest = kMaxRailTime * fs;
    bufLength_ = 1;
    while (bufLength_ < longest + 4) bufLength_ <<= 1;
    mask_ = bufLength_ - 1;
    logMax_ = (float)std::log2(longest);
    logMin_ = (float)std::log2(std::fmax(kMinRailTime * fs, (double)kMinRailSamples));
    maxLength_ = std::exp2(logMax_);
    minLength_ = std::exp2(logMin_);
    lossScale_ = kLossTab / (logMax_ - logMin_);
    rails_.assign((size_t)NR * bufLength_, 0.f);
    for (int R = 0; R < NR; ++R) {
        rand1_[R] = rng_.frand() - 0.5f;
        rand2_[R] = rng_.frand() - 0.5f;
        wPhase_[R] = rng_.frand();
        wInc_[R] = (float)((kWanderRateMin + (kWanderRateMax - kWanderRateMin) * rng_.frand()) / fs);
        wFrom_[R] = rng_.frand2();
        wTo_[R] = rng_.frand2();
    }
    spread1_ = 2 * std::log2(1 + std::fmax(p_.rand1, 0.f));
    wanderOct_ = std::log2(1 + std::fmax(p_.wander, 0.f));
    updateTargets();
    updateLag();
    updateLoss();
    // the plate starts at its target size
    float offset[NR];
    for (int R = 0; R < NR; ++R) {
        logRemain_[R] = 0;
        offset[R] = wanderOct_ * wanderValue(R);
        logRead_[R] = logTarget_[R];
        length_[R] = clampf(targetLength_[R] * std::exp2(offset[R]), minLength_, maxLength_);
    }
    updateLengths(offset);
    retarget_ = false;
}

void Ref::set(const Params& p) {
    const bool loss = p.decay != p_.decay || p.hfDecay != p_.hfDecay;
    const bool lag = p.glide != p_.glide || p.rand2 != p_.rand2 || p.glideMode != p_.glideMode;
    const bool tgt = p.time != p_.time || p.rand1 != p_.rand1;
    p_ = p;
    spread1_ = 2 * std::log2(1 + std::fmax(p.rand1, 0.f));
    wanderOct_ = std::log2(1 + std::fmax(p.wander, 0.f));
    if (tgt) updateTargets();
    if (lag) updateLag();
    if (loss) updateLoss();
}

void Ref::updateTargets() {
    const float logTime = logMax_ + clampf(p_.time, 0, 1) * (logMin_ - logMax_);
    for (int R = 0; R < NR; ++R) {
        const float target = clampf(logTime + spread1_ * rand1_[R], logMin_, logMax_);
        logRemain_[R] += target - logTarget_[R];
        logTarget_[R] = target;
        targetLength_[R] = std::exp2(target);
    }
    retarget_ = true;
}

void Ref::updateLag() {
    const float spread = 2 * std::log2(1 + std::fmax(p_.rand2, 0.f));
    for (int R = 0; R < NR; ++R) {
        const double seconds = std::fmax(p_.glide, 0.f) * std::exp2(spread * rand2_[R]);
        if (p_.glideMode)
            lagCoef_[R] = seconds > 0 ? (float)(1 / (seconds * fs_)) : 1e30f;
        else
            lagCoef_[R] = seconds > 0 ? (float)-std::expm1(-6.907755278982137 / (seconds * fs_)) : 1.f;
    }
}

void Ref::updateLoss() {
    const double kLo = -6.907755278982137 / (fs_ * clampf(p_.decay, 0.01f, 1000.f));
    const double kHi = -6.907755278982137 / (fs_ * clampf(p_.hfDecay, 0.001f, 1000.f));
    for (int i = 0; i <= kLossTab; ++i) {
        const double D = std::exp2(logMin_ + i / (double)lossScale_);
        const double r = std::exp((kHi - kLo) * D);
        const double b = (1 - r) / (1 + r);
        gainTab_[i] = (float)(std::exp(kLo * D) * (1 - b));
        poleTab_[i] = (float)b;
    }
    updateGains();
}

void Ref::updateGains() {
    for (int R = 0; R < NR; ++R) {
        const float x = (logRead_[R] - logMin_) * lossScale_;
        const int i = x < 0 ? 0 : (x >= kLossTab ? kLossTab - 1 : (int)x);
        const float fx = clampf(x - i, 0, 1);
        gain_[R] = gainTab_[i] + (gainTab_[i + 1] - gainTab_[i]) * fx;
        pole_[R] = poleTab_[i] + (poleTab_[i + 1] - poleTab_[i]) * fx;
    }
}

float Ref::wanderValue(int R) const {
    const float ph = wPhase_[R];
    return wFrom_[R] + (wTo_[R] - wFrom_[R]) * (ph * ph * (3 - 2 * ph));
}

void Ref::updateLengths(const float* offset) {
    float D[NR];
    for (int R = 0; R < NR; ++R) {
        logRead_[R] = clampf(logTarget_[R] + offset[R], logMin_, logMax_);
        D[R] = clampf(targetLength_[R] * std::exp2(offset[R]), minLength_, maxLength_);
    }
    for (int R = 0; R < NR; ++R) {
        const float prev = length_[R];
        if (std::abs(D[R] - prev) > kMaxSlew) {
            D[R] = D[R] > prev ? prev + kMaxSlew : prev - kMaxSlew;
            logRead_[R] = std::log2(D[R]);
            logRemain_[R] += offset[R] - (logRead_[R] - logTarget_[R]);
        }
    }
    for (int R = 0; R < NR; ++R) {
        const int d = (int)D[R];
        const float f = D[R] - d;
        const float a = f * (f - 1), c = (f + 1) * (f - 2);
        tap_[0][R] = a * (f - 2) * (-1.f / 6);
        tap_[1][R] = c * (f - 1) * 0.5f;
        tap_[2][R] = c * f * -0.5f;
        tap_[3][R] = a * (f + 1) * (1.f / 6);
        tapDelay_[R] = d;
        length_[R] = D[R];
    }
    updateGains();
}

void Ref::render(float in, float& outL, float& outR) {
    bool moving = retarget_ || wanderOct_ > 0.f;
    for (int R = 0; R < NR; ++R) {
        const float remain = logRemain_[R];
        if (remain == 0.f)
            continue;
        moving = true;
        if (p_.glideMode) {
            const float step = lagCoef_[R];
            logRemain_[R] = remain > step ? remain - step : (remain < -step ? remain + step : 0.f);
        } else {
            const float left = remain - remain * lagCoef_[R];
            logRemain_[R] = std::abs(left) < 1e-6f ? 0.f : left;
        }
    }
    if (moving) {
        float offset[NR];
        for (int R = 0; R < NR; ++R) {
            if (wanderOct_ > 0.f) {
                float ph = wPhase_[R] + wInc_[R];
                if (ph >= 1.f) {
                    ph -= 1.f;
                    wFrom_[R] = wTo_[R];
                    wTo_[R] = rng_.frand2();
                }
                wPhase_[R] = ph;
                offset[R] = wanderOct_ * wanderValue(R) - logRemain_[R];
            } else
                offset[R] = -logRemain_[R];
        }
        updateLengths(offset);
        retarget_ = false;
    }
    // read every rail at its length (delay k is slot w - 1 - k; taps at d-1 .. d+2), loop filter
    for (int R = 0; R < NR; ++R) {
        const float* rail = &rails_[(size_t)R * bufLength_];
        const int s0 = w_ - tapDelay_[R];
        const float x = tap_[0][R] * rail[s0 & mask_] + tap_[1][R] * rail[(s0 - 1) & mask_] +
                        tap_[2][R] * rail[(s0 - 2) & mask_] + tap_[3][R] * rail[(s0 - 3) & mask_];
        wave_[R] = lp_[R] = gain_[R] * x + pole_[R] * lp_[R];
    }
    // junction pressures (rails of J: 0 = W->J, 1 = N->J, 2 = J->W, 3 = J->N)
    for (int i = 0; i < NX; ++i)
        for (int j = 0; j < NY; ++j) {
            const int J = i * NY + j, E = i * NY + ((j + 1) & (NY - 1)), S = ((i + 1) & (NX - 1)) * NY + j;
            p_j_[J] = 0.5f * (wave_[J * 4 + 0] + wave_[J * 4 + 1] + wave_[E * 4 + 2] + wave_[S * 4 + 3]);
        }
    p_j_[kInput] += 0.4f * in;
    const int slot = (w_ - 1) & mask_;
    for (int i = 0; i < NX; ++i)
        for (int j = 0; j < NY; ++j) {
            const int J = i * NY + j, W = i * NY + ((j - 1) & (NY - 1)), N = ((i - 1) & (NX - 1)) * NY + j;
            rails_[(size_t)(J * 4 + 0) * bufLength_ + slot] = p_j_[W] - wave_[J * 4 + 2];
            rails_[(size_t)(J * 4 + 1) * bufLength_ + slot] = p_j_[N] - wave_[J * 4 + 3];
            rails_[(size_t)(J * 4 + 2) * bufLength_ + slot] = p_j_[J] - wave_[J * 4 + 0];
            rails_[(size_t)(J * 4 + 3) * bufLength_ + slot] = p_j_[J] - wave_[J * 4 + 1];
        }
    w_ = (w_ + 1) & mask_;
    outL = p_j_[kPickupL];
    outR = p_j_[kPickupR];
}

void Ref::process(const float* in, float* outL, float* outR, int n) {
    for (int i = 0; i < n; ++i)
        render(in[i], outL[i], outR[i]);
}

}  // namespace pmw
