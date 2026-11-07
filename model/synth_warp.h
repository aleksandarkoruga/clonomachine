// Oscillator warps for the synth model, after the warp menus of software wavetable synths (two
// slots per oscillator). Manuals describe what each mode does, not the formulas; these are
// reconstructions from third-party measurements, with the unknown constants picked by ear.
//
//   phase warps   the read position inside the frame is remapped (sync, bend, asym, PWM, mirror,
//                 quantize); warp_phase() also reports the map's largest slope, which raises the
//                 oscillator's instantaneous frequency (for the warp-aware mip level choice)
//   output warps  flip (polarity), odd/even
//   cross mod     FM (phase modulation by the other oscillator, the sub or noise: the classic
//                 "FM from B"), AM, RM, and thru-zero FM (the phase increment)
#pragma once
#include <algorithm>
#include <cmath>

namespace synth {

enum Warp {
    W_OFF = 0,
    W_SYNC,      // hard self-sync: the frame runs 2^(4 w) times per period (up to 16x)
    W_SYNCWIN,   // sync with a sin(pi phi) window (no click at the reset)
    W_BEND_P,    // each half bent towards the middle (u^p, p = 2^(3w))
    W_BEND_M,    // each half bent towards the edges (p = 2^(-3w))
    W_BEND_PM,   // 50 % = unchanged
    W_PWM,       // the frame squeezed into (1 - 0.95 w) of the period, its end value held after
    W_ASYM_P,    // the whole cycle bent (phi^p): a phase-distortion knee
    W_ASYM_M,
    W_ASYM_PM,
    W_FLIP,      // polarity flips at 1 - w
    W_MIRROR,    // forward then backward, each half asym-bent by w
    W_QUANT,     // the phase held in 2048 * 2^(-10 w) steps
    W_ODDEVEN,   // 0 odd harmonics only, 0.5 unchanged, 1 even only
    W_FM,        // phase modulation by the other oscillator: index 2 w cycles
    W_FM_SUB,    // ... by the sub
    W_FM_NOISE,  // ... by noise
    W_AM,        // amplitude modulation by the other oscillator (unipolar)
    W_RM,        // ring modulation by the other oscillator
    W_TZFM,      // thru-zero FM by the other oscillator: increment x (1 + 4 w m)
    W_COUNT
};

inline const char* warp_name(int w) {
    static const char* n[] = {"off",   "sync",   "sync window", "bend+", "bend-", "bend+-", "pwm",
                              "asym+", "asym-",  "asym+-",      "flip",  "mirror", "quantize", "odd/even",
                              "fm",    "fm sub", "fm noise",    "am",    "rm",    "thru-zero fm"};
    return w >= 0 && w < W_COUNT ? n[w] : "?";
}
inline bool warp_is_xmod(int w) { return w == W_FM || w == W_AM || w == W_RM || w == W_TZFM; }

namespace detail {
inline double bend(double u, double p) { return std::pow(std::clamp(u, 0.0, 1.0), p); }
// slope bound of u^p over u in [eps, 1] (unbounded at 0 for p < 1: the level choice uses the
// slope at 1/64 of the cycle, beyond that the aliasing is accepted)
inline double bend_slope(double p) { return p >= 1 ? p : p * std::pow(1.0 / 64, p - 1); }
}  // namespace detail

// phase warp: phi in [0, 1) -> [0, 1]; slope = the largest dphi'/dphi
inline double warp_phase(int w, double amt, double phi, double* slope = nullptr) {
    using namespace detail;
    amt = std::clamp(amt, 0.0, 1.0);
    double s = 1, r = phi;
    switch (w) {
    case W_SYNC:
    case W_SYNCWIN: {
        const double k = std::exp2(4 * amt);
        r = phi * k;
        r -= std::floor(r);
        s = k;
        break;
    }
    case W_BEND_P:
    case W_BEND_M:
    case W_BEND_PM: {
        const double e = w == W_BEND_P ? 3 * amt : w == W_BEND_M ? -3 * amt : 3 * (2 * amt - 1);
        const double p = std::exp2(e);
        r = phi < 0.5 ? 0.5 * bend(2 * phi, p) : 1 - 0.5 * bend(2 - 2 * phi, p);
        s = bend_slope(p);
        break;
    }
    case W_PWM: {
        const double width = 1 - 0.95 * amt;
        r = std::min(phi / width, 1.0);
        s = 1 / width;
        break;
    }
    case W_ASYM_P:
    case W_ASYM_M:
    case W_ASYM_PM: {
        const double e = w == W_ASYM_P ? 3 * amt : w == W_ASYM_M ? -3 * amt : 3 * (2 * amt - 1);
        const double p = std::exp2(e);
        r = bend(phi, p);
        s = bend_slope(p);
        break;
    }
    case W_MIRROR: {
        const double u = phi < 0.5 ? 2 * phi : 2 - 2 * phi;
        const double p = std::exp2(3 * (2 * amt - 1));
        r = bend(u, p);
        s = 2 * bend_slope(p);
        break;
    }
    case W_QUANT: {
        const double q = 2048 * std::exp2(-10 * amt);
        r = std::floor(phi * q) / q;
        break;
    }
    default:
        break;
    }
    if (slope)
        *slope = s;
    return r;
}

// output-side factor of a warp at phase phi (flip; sync window)
inline double warp_gain(int w, double amt, double phi) {
    if (w == W_FLIP)
        return phi < 1 - std::clamp(amt, 0.0, 1.0) ? 1.0 : -1.0;
    if (w == W_SYNCWIN)
        return std::sin(M_PI * phi);
    return 1.0;
}

}  // namespace synth
