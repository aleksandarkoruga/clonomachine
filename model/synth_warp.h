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
    W_BEND_P,    // each half bent: u + a u (1 - u), a = w (towards the middle of the half)
    W_BEND_M,    // a = -w (towards its edges)
    W_BEND_PM,   // a = 2w - 1 (50 % = unchanged)
    W_PWM,       // the frame squeezed into (1 - 0.95 w) of the period, its end value held after
    W_ASYM_P,    // phase-distortion knee over the whole cycle: the first half of the frame in
    W_ASYM_M,    //   d = 0.5 - 0.49 w of the period (asym-: 0.5 + 0.49 w; asym+-: 50 % = unchanged)
    W_ASYM_PM,
    W_FLIP,      // polarity flips at 1 - w
    W_MIRROR,    // forward then backward, each half with the asym+- knee (50 % = plain mirror)
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

// (The reference curves aren't published; third-party measurements map bend to a piecewise-linear
// squeeze and asym to a smooth whole-cycle bend. These forms take one or two multiplies in the
// FPGA: rtl/synth_osc.v.)
namespace detail {
inline double knee(double u, double d) { return u < d ? u * 0.5 / d : 0.5 + (u - d) * 0.5 / (1 - d); }
inline double knee_slope(double d) { return std::max(0.5 / d, 0.5 / (1 - d)); }
inline double knee_d(int w, double amt) {
    return w == W_ASYM_P ? 0.5 - 0.49 * amt : w == W_ASYM_M ? 0.5 + 0.49 * amt : 0.5 + 0.49 * (1 - 2 * amt);
}
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
        const double a = w == W_BEND_P ? amt : w == W_BEND_M ? -amt : 2 * amt - 1;
        const double u = phi < 0.5 ? 2 * phi : 2 - 2 * phi, b = u + a * u * (1 - u);
        r = phi < 0.5 ? 0.5 * b : 1 - 0.5 * b;
        s = 1 + std::fabs(a);
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
        const double d = knee_d(w, amt);
        r = knee(phi, d);
        s = knee_slope(d);
        break;
    }
    case W_MIRROR: {
        const double u = phi < 0.5 ? 2 * phi : 2 - 2 * phi, d = knee_d(W_ASYM_PM, amt);
        r = std::min(knee(u, d), 0.999999);
        s = 2 * knee_slope(d);
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
