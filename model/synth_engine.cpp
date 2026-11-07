// Model of the wavetable synth, see synth_engine.h.
#include "synth_engine.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "synth_qmf.h"
#include "synth_warp.h"

namespace synth {
namespace {

constexpr double kFs = 48000.0;

struct ParamDef {
    const char* name;
    double def;
};
// Everything a patch can set and a mod route can target (in these units).
const ParamDef kParams[] = {
    {"volume", 0.5}, {"mono", 0}, {"legato", 0}, {"porta", 0}, {"porta_always", 0},
    // oscillators (a_ / b_): level 0..1, pan -1..1, oct/semi/fine (cents), pos 0..1 over the
    // frames, warp mode (synth_warp.h) and amount 0..1 (two warp slots), start phase 0..1,
    // phase randomisation 0..1, unison voices / detune (semitones at the outer pair) / blend
    {"a_on", 1}, {"a_level", 0.75}, {"a_pan", 0}, {"a_oct", 0}, {"a_semi", 0}, {"a_fine", 0},
    {"a_pos", 0}, {"a_warp", 0}, {"a_warp_amt", 0}, {"a_warp2", 0}, {"a_warp2_amt", 0},
    {"a_phase", 0}, {"a_rand", 0}, {"a_unison", 1}, {"a_detune", 0.25}, {"a_range", 2}, {"a_blend", 0.75}, {"a_width", 1},
    {"a_smooth", 0}, {"a_filt", 1},
    {"b_on", 0}, {"b_level", 0.75}, {"b_pan", 0}, {"b_oct", 0}, {"b_semi", 0}, {"b_fine", 0},
    {"b_pos", 0}, {"b_warp", 0}, {"b_warp_amt", 0}, {"b_warp2", 0}, {"b_warp2_amt", 0},
    {"b_phase", 0}, {"b_rand", 0}, {"b_unison", 1}, {"b_detune", 0.25}, {"b_range", 2}, {"b_blend", 0.75}, {"b_width", 1},
    {"b_smooth", 0}, {"b_filt", 1},
    // sub (shape 0 sine 1 triangle 2 saw 3 square, octave relative to the note) and noise (white)
    {"sub_on", 0}, {"sub_level", 0.75}, {"sub_oct", -1}, {"sub_shape", 0}, {"sub_filt", 0},
    {"noise_on", 0}, {"noise_level", 0.25}, {"noise_filt", 1},
    // filter (quad morph, synth_qmf.h): cutoff in Hz (mods in octaves via "cutoff_oct"), res 0.1..10
    // (the plugin's Res / Ctrl), drive 0..1 (input gain 1..8), morph x/y 0..1; per slot: model
    // (plugin index), type 0 LP 1 BP 2 HP 3 notch, slope 0..3, cutoff offset in octaves
    {"filt_on", 0}, {"cutoff", 20000}, {"cutoff_oct", 0}, {"res", 0.707}, {"drive", 0}, {"morph_x", 0}, {"morph_y", 0},
    {"slot1", 0}, {"slot2", 0}, {"slot3", 0}, {"slot4", 0},
    {"slot1_type", 0}, {"slot2_type", 1}, {"slot3_type", 2}, {"slot4_type", 3},
    {"slot1_slope", 0}, {"slot2_slope", 0}, {"slot3_slope", 0}, {"slot4_slope", 0},
    {"slot1_oct", 0}, {"slot2_oct", 0}, {"slot3_oct", 0}, {"slot4_oct", 0}, {"filt_key", 0}, {"filt_mix", 1},
    // envelopes: attack hold decay (s), sustain 0..1, release (s), curves -1..1 (0 = linear
    // attack, exponential-like decay / release)
    {"env1_a", 0.0005}, {"env1_h", 0}, {"env1_d", 1}, {"env1_s", 1}, {"env1_r", 0.015},
    {"env2_a", 0.0005}, {"env2_h", 0}, {"env2_d", 1}, {"env2_s", 0}, {"env2_r", 0.015},
    {"env3_a", 0.0005}, {"env3_h", 0}, {"env3_d", 1}, {"env3_s", 0}, {"env3_r", 0.015},
    // LFOs: shape (0 sine 1 tri 2 saw down 3 saw up 4 square 5 random S&H; fractions crossfade),
    // rate (Hz, or cycles per beat when sync), mode (0 trig: restarts per note, 1 env: one cycle,
    // 2 free), rise (s), delay (s), start phase
    {"lfo1_shape", 1}, {"lfo1_rate", 1}, {"lfo1_sync", 1}, {"lfo1_mode", 0}, {"lfo1_rise", 0}, {"lfo1_delay", 0}, {"lfo1_phase", 0},
    {"lfo2_shape", 1}, {"lfo2_rate", 1}, {"lfo2_sync", 1}, {"lfo2_mode", 0}, {"lfo2_rise", 0}, {"lfo2_delay", 0}, {"lfo2_phase", 0},
    {"lfo3_shape", 1}, {"lfo3_rate", 1}, {"lfo3_sync", 1}, {"lfo3_mode", 0}, {"lfo3_rise", 0}, {"lfo3_delay", 0}, {"lfo3_phase", 0},
    {"lfo4_shape", 1}, {"lfo4_rate", 1}, {"lfo4_sync", 1}, {"lfo4_mode", 0}, {"lfo4_rise", 0}, {"lfo4_delay", 0}, {"lfo4_phase", 0},
    {"macro1", 0}, {"macro2", 0}, {"macro3", 0}, {"macro4", 0}, {"modwheel", 0}, {"pitch", 0},
};
constexpr int kNP = int(sizeof kParams / sizeof kParams[0]);

int pidx(const std::string& n) {
    for (int i = 0; i < kNP; ++i)
        if (n == kParams[i].name)
            return i;
    return -1;
}

inline double qz(double x, int frac) {
    const double s = std::ldexp(1.0, frac);
    return std::nearbyint(x * s) / s;
}
inline double sat(double x, double lim) { return x > lim ? lim : x < -lim ? -lim : x; }

enum Src { S_ENV1, S_ENV2, S_ENV3, S_LFO1, S_LFO2, S_LFO3, S_LFO4, S_VEL, S_NOTE, S_RAND, S_MACRO1, S_MACRO2,
           S_MACRO3, S_MACRO4, S_MODWHEEL, S_COUNT };
int src_index(std::string s, bool& bip) {
    bip = !s.empty() && s.back() == '~';
    if (bip)
        s.pop_back();
    static const char* n[] = {"env1", "env2", "env3", "lfo1", "lfo2", "lfo3", "lfo4", "vel", "note", "rand",
                              "macro1", "macro2", "macro3", "macro4", "modwheel"};
    for (int i = 0; i < S_COUNT; ++i)
        if (s == n[i])
            return i;
    return -1;
}

struct Env {
    enum { IDLE, ATK, HOLD, DEC, SUS, REL } st = IDLE;
    double v = 0, t = 0, relFrom = 0;
    void gate_on(bool retrig) {
        if (retrig || st == IDLE || st == REL) {
            st = ATK;
            t = 0;
            if (retrig)
                v = v;  // restart from the current level (no click)
        }
    }
    void gate_off() {
        if (st != IDLE) {
            st = REL;
            t = 0;
            relFrom = v;
        }
    }
    // advance by dt seconds with attack/hold/decay/sustain/release
    double step(double dt, double a, double h, double d, double s, double r) {
        t += dt;
        switch (st) {
        case IDLE: v = 0; break;
        case ATK:
            v = a <= 0 ? 1 : std::min(1.0, v + dt / a);
            if (v >= 1) st = HOLD, t = 0;
            break;
        case HOLD:
            v = 1;
            if (t >= h) st = DEC, t = 0;
            break;
        case DEC: {
            // exponential-like: reaches the sustain level in d seconds (-60 dB of the distance)
            const double k = d > 0 ? std::exp(-6.9 * dt / d) : 0;
            v = s + (v - s) * k;
            if (t >= d) st = SUS, v = s;
            break;
        }
        case SUS: v = s; break;
        case REL: {
            const double k = r > 0 ? std::exp(-6.9 * dt / r) : 0;
            v *= k;
            if (t >= r || v < 1e-5) st = IDLE, v = 0;
            break;
        }
        }
        return v;
    }
};

struct Lfo {
    double ph = 0, held = 0, age = 0;
    bool done = false;
    uint32_t rng = 1;
    double rnd() {
        rng ^= rng << 13, rng ^= rng >> 17, rng ^= rng << 5;
        return (rng >> 8) * (1.0 / 16777216.0);
    }
    static double shape(double s, double p, double held) {
        auto one = [&](int k) {
            switch (k) {
            case 0: return 0.5 - 0.5 * std::cos(2 * M_PI * p);
            case 1: return 1 - std::fabs(2 * p - 1);
            case 2: return 1 - p;
            case 3: return p;
            case 4: return p < 0.5 ? 1.0 : 0.0;
            default: return held;
            }
        };
        const int a = std::clamp(int(s), 0, 5), b = std::min(a + 1, 5);
        const double u = std::clamp(s - a, 0.0, 1.0);
        return one(a) * (1 - u) + one(b) * u;
    }
};

struct Osc {
    double ph[16] = {};  // unison phases
    double prev = 0;     // last output, before level and pan (the cross-modulation signal)
    FrameCache cache;
    // control-rate state
    double inc = 0, pos = 0, level = 0, w1amt = 0, w2amt = 0, pan = 0;
    int w1 = 0, w2 = 0, lev = 0, uni = 1, dir = 1;
    double detune = 0, blend = 0, width = 0;
    bool smooth = false, on = false;
};

struct Voice {
    bool active = false, gate = false;
    int note = 60;
    double vel = 1, age = 0, rnd = 0;
    double pitch = 60, pitchTarget = 60, portaRate = 0;  // semitones
    Env env[3];
    Lfo lfo[4];
    Osc osc[2];
    double subPh = 0, subInc = 0, sub = 0, noise = 0;
    QuadMorph filt, filtR;  // filtR: the right channel when the voice is stereo
    double fc = 1000, res = 0.707, drive = 1, mx = 0, my = 0, oct[4] = {0, 0, 0, 0};
    double amp0 = 0, amp1 = 0;  // gain at the start / end of the control block
    double mod[kNP];            // modulated parameter values for this block
    uint64_t startSample = 0;
};

}  // namespace

struct Engine::Impl {
    Config cfg;
    Song song;
    wt::Table tab[2];
    double base[kNP];
    struct Route {
        int src, dst;
        bool bip;
        double amt;
    };
    std::vector<Route> routes;
    std::vector<Voice> voices;
    std::vector<Halfband> hb;
    std::vector<std::unique_ptr<Decim2>> decL, decR;  // a chain of 2x stages
    Sdram mem;
    uint32_t rng = 0x9E3779B9u;
    uint64_t n = 0;
    Stats* st = nullptr;
    std::map<std::string, uint64_t> macs;
    uint64_t reads = 0, fns = 0;
    QmfFixed qf;

    double frand() {
        rng ^= rng << 13, rng ^= rng >> 17, rng ^= rng << 5;
        return (rng >> 8) * (1.0 / 16777216.0);
    }
    double P(const Voice& v, const char* name) const { return v.mod[pidx(name)]; }

    void note_on(int note, double vel) {
        const bool mono = base[pidx("mono")] > 0.5;
        const int nv = mono ? 1 : std::clamp(cfg.voices, 1, int(voices.size()));
        Voice* v = nullptr;
        if (mono) {
            v = &voices[0];
        } else {
            for (int i = 0; i < nv && !v; ++i)
                if (!voices[i].active)
                    v = &voices[i];
            if (!v) {  // steal: a released voice first, else the oldest
                double best = -1;
                for (int i = 0; i < nv; ++i) {
                    const double score = voices[i].age + (voices[i].gate ? 0 : 1e9);
                    if (score > best)
                        best = score, v = &voices[i];
                }
            }
        }
        const bool legato = mono && base[pidx("legato")] > 0.5 && v->active && v->gate;
        const double porta = base[pidx("porta")];
        const bool glide = porta > 0 && v->active && (legato || base[pidx("porta_always")] > 0.5 || mono);
        v->pitchTarget = note;
        if (!glide)
            v->pitch = note;
        v->portaRate = porta > 0 ? 1.0 / porta : 0;
        v->note = note;
        v->vel = vel;
        if (!legato) {
            const bool fresh = !v->active;
            v->active = true;
            v->age = 0;
            v->rnd = frand();
            for (auto& e : v->env)
                e.gate_on(true);
            for (int i = 0; i < 4; ++i) {
                const int mode = int(base[pidx(("lfo" + std::to_string(i + 1) + "_mode").c_str())]);
                if (mode != 2) {
                    v->lfo[i].ph = base[pidx(("lfo" + std::to_string(i + 1) + "_phase").c_str())];
                    v->lfo[i].age = 0;
                    v->lfo[i].done = false;
                }
            }
            for (int o = 0; o < 2; ++o) {
                const char* pre = o ? "b_" : "a_";
                const double ph0 = base[pidx(std::string(pre).append("phase"))];
                const double rnd = base[pidx(std::string(pre).append("rand"))];
                for (double& p : v->osc[o].ph) {
                    p = ph0 + rnd * frand();
                    p -= std::floor(p);
                }
            }
            if (fresh) {
                v->filt = v->filtR = QuadMorph();
                v->filt.set_rate(kFs * cfg.rate);
                v->filtR.set_rate(kFs * cfg.rate);
            }
            for (auto& o : v->osc)  // play nothing until this note's frames are in (no stale frames)
                o.cache.fresh = true;
            v->subPh = 0;
            v->startSample = n;
        }
        v->gate = true;
    }
    void note_off(int note) {
        for (auto& v : voices)
            if (v.active && v.gate && v.note == note) {
                v.gate = false;
                for (auto& e : v.env)
                    e.gate_off();
            }
    }

    // control block: modulation sources -> parameter values -> per-oscillator settings
    void control(Voice& v, double dtBlock) {
        v.age += dtBlock;
        // pitch glide (linear in semitones at 1 / porta semitones... per second x distance)
        if (v.pitch != v.pitchTarget) {  // portamento: exponential, ~99 % of the way in 'porta' seconds
            const double k = v.portaRate > 0 ? std::exp(-4.6 * dtBlock * v.portaRate) : 0;
            v.pitch = v.pitchTarget + (v.pitch - v.pitchTarget) * k;
            if (std::fabs(v.pitch - v.pitchTarget) < 1e-4)
                v.pitch = v.pitchTarget;
        }
        double src[S_COUNT] = {};
        for (int e = 0; e < 3; ++e) {
            const std::string p = "env" + std::to_string(e + 1) + "_";
            src[S_ENV1 + e] = v.env[e].step(dtBlock, base[pidx(p + "a")], base[pidx(p + "h")], base[pidx(p + "d")],
                                            base[pidx(p + "s")], base[pidx(p + "r")]);
        }
        for (int l = 0; l < 4; ++l) {
            const std::string p = "lfo" + std::to_string(l + 1) + "_";
            Lfo& L = v.lfo[l];
            const double rate = base[pidx(p + "rate")] * (base[pidx(p + "sync")] > 0.5 ? song.bpm / 60.0 : 1.0);
            L.age += dtBlock;
            const double delay = base[pidx(p + "delay")], rise = base[pidx(p + "rise")];
            const int mode = int(base[pidx(p + "mode")]);
            double g = 1;
            if (L.age < delay)
                g = 0;
            else if (rise > 0)
                g = std::min(1.0, (L.age - delay) / rise);
            if (!(mode == 1 && L.done) && L.age >= delay) {
                const double np = L.ph + rate * dtBlock;
                if (np >= 1) {
                    if (mode == 1)
                        L.done = true;
                    L.held = L.rnd();
                }
                L.ph = mode == 1 && L.done ? 0.999999 : np - std::floor(np);
            }
            src[S_LFO1 + l] = g * Lfo::shape(base[pidx(p + "shape")], L.ph, L.held);
        }
        src[S_VEL] = v.vel;
        src[S_NOTE] = (v.note - 60) / 60.0;
        src[S_RAND] = v.rnd;
        for (int m = 0; m < 4; ++m)
            src[S_MACRO1 + m] = base[pidx("macro" + std::to_string(m + 1))];
        src[S_MODWHEEL] = base[pidx("modwheel")];
        std::copy(base, base + kNP, v.mod);
        for (auto& r : routes)
            v.mod[r.dst] += r.amt * (r.bip ? 2 * src[r.src] - 1 : src[r.src]);
        macs["mod"] += routes.size() + 3 * 2 + 4 * 2;

        // oscillators
        const double fsH = kFs * cfg.rate;
        for (int o = 0; o < 2; ++o) {
            Osc& O = v.osc[o];
            const std::string pre = o ? "b_" : "a_";
            const double semis = v.pitch + v.mod[pidx("pitch")] + 12 * v.mod[pidx(pre + "oct")] + v.mod[pidx(pre + "semi")] +
                                 v.mod[pidx(pre + "fine")] / 100.0;
            const double f = 440.0 * std::exp2((semis - 69) / 12.0);
            O.inc = f / fsH;
            const double np = std::clamp(v.mod[pidx(pre + "pos")], 0.0, 1.0);
            O.dir = np > O.pos ? 1 : np < O.pos ? -1 : O.dir;
            O.pos = np;
            O.level = v.mod[pidx(pre + "on")] > 0.5 ? std::clamp(v.mod[pidx(pre + "level")], 0.0, 1.0) : 0.0;
            O.pan = std::clamp(v.mod[pidx(pre + "pan")], -1.0, 1.0);
            O.w1 = int(v.mod[pidx(pre + "warp")]);
            O.w2 = int(v.mod[pidx(pre + "warp2")]);
            O.w1amt = std::clamp(v.mod[pidx(pre + "warp_amt")], 0.0, 1.0);
            O.w2amt = std::clamp(v.mod[pidx(pre + "warp2_amt")], 0.0, 1.0);
            O.on = v.mod[pidx(pre + "on")] > 0.5;
            O.uni = std::clamp(int(v.mod[pidx(pre + "unison")]), 1, 16);
            // the outer voices at +-knob^2 x range semitones
            const double dk = std::clamp(v.mod[pidx(pre + "detune")], 0.0, 1.0);
            O.detune = dk * dk * v.mod[pidx(pre + "range")];
            O.blend = std::clamp(v.mod[pidx(pre + "blend")], 0.0, 1.0);
            O.width = std::clamp(v.mod[pidx(pre + "width")], 0.0, 1.0);
            O.smooth = v.mod[pidx(pre + "smooth")] > 0.5;
            // the level: highest instantaneous frequency (warps, phase / frequency modulation)
            double fmax = f;
            if (cfg.warpAwareLevels) {
                double slope = 1, pm = 0;
                for (int s = 0; s < 2; ++s) {
                    const int w = s ? O.w2 : O.w1;
                    const double a = s ? O.w2amt : O.w1amt;
                    double sl = 1;
                    warp_phase(w, a, 0.5, &sl);
                    slope *= sl;
                    if (w == W_ODDEVEN && a > 0.5)
                        slope *= 2;
                    if (w == W_TZFM)
                        slope *= 1 + 4 * a;
                    // phase modulation: index 2a cycles x the modulator's top (~4 harmonics of its f)
                    if (w == W_FM)
                        pm += 2 * a * 2 * M_PI * 4 * f;
                    if (w == W_FM_SUB)
                        pm += 2 * a * 2 * M_PI * f * std::exp2(v.mod[pidx("sub_oct")]);
                }
                fmax = f * slope + pm;
            }
            fmax *= O.uni > 1 ? std::exp2(O.detune / 12.0) : 1.0;
            const double limit = cfg.rate == 1 ? 24000.0 : std::min(fsH - 24000.0, 0.45 * fsH);
            int k = wt::level_for(tab[o], fmax, limit);
            while (tab[o].size(k) > cfg.maxFrame && k + 1 < tab[o].levels)
                ++k;
            O.lev = k;
        }
        v.subInc = 440.0 * std::exp2((v.pitch + v.mod[pidx("pitch")] - 69) / 12.0 + v.mod[pidx("sub_oct")]) / fsH;
        // filter
        const double oct = v.mod[pidx("cutoff_oct")] + v.mod[pidx("filt_key")] * (v.pitch - 60) / 12.0;
        v.fc = std::clamp(v.mod[pidx("cutoff")] * std::exp2(oct), 20.0, 20000.0);
        v.res = std::clamp(v.mod[pidx("res")], 0.1, 10.0);
        v.drive = 1 + 7 * std::clamp(v.mod[pidx("drive")], 0.0, 1.0);
        v.mx = std::clamp(v.mod[pidx("morph_x")], 0.0, 1.0);
        v.my = std::clamp(v.mod[pidx("morph_y")], 0.0, 1.0);
        for (int s = 0; s < 4; ++s) {
            const std::string p = "slot" + std::to_string(s + 1);
            const int m = int(v.mod[pidx(p)]), sl = int(v.mod[pidx(p + "_slope")]);
            for (QmfSlot* S : {&v.filt.slot[s], &v.filtR.slot[s]}) {
                if (m != S->model || sl != S->slope)
                    S->set_slope(m, sl);
                S->type = int(v.mod[pidx(p + "_type")]);
            }
            v.oct[s] = v.mod[pidx(p + "_oct")];
        }
        // amplitude: env 1 x volume (ramped over the block)
        v.amp0 = v.amp1;
        v.amp1 = src[S_ENV1] * v.vel;
        if (!v.gate && v.env[0].st == Env::IDLE)
            v.active = false;
    }

    // one oscillator sample at the high rate. other: the other oscillator's output (this sample if
    // it ran first, else its previous one). Returns the mono signal (the modulator for the other
    // oscillator) and adds the level- and pan-scaled stereo contribution into l / r.
    double osc_sample(Voice& v, int o, double other, double& l, double& r) {
        Osc& O = v.osc[o];
        const wt::Table& T = tab[o];
        if (!O.on) {
            O.prev = 0;
            return 0;
        }
        const int N = O.uni;
        double sum = 0, sl = 0, sr = 0;
        const int taps = cfg.interp == Interp::SINC8 ? 8 : cfg.interp == Interp::HERM ? 4 : 2;
        // table position: steps through frames unless 'smooth' crossfades the neighbours
        double pos = O.pos * (T.frames - 1);
        if (!O.smooth)
            pos = std::round(pos);
        int k = O.lev;
        if (cfg.cache) {
            int kk = k;
            pos = O.cache.resolve(pos, k, T.frames, double(n), T.size(k), mem, O.dir, kk);
            if (kk < 0) {  // this note's first frames still loading
                O.prev = 0;
                return 0;
            }
            k = kk;
        }
        const bool two = pos != std::floor(pos);
        auto read = [&](double ph) {
            reads += taps * (two ? 2 : 1);
            macs["osc"] += taps * (two ? 2 : 1) + (cfg.interp == Interp::SINC8 ? taps : 0) + (two ? 1 : 0);
            return cfg.fixed ? read_q(T, k, pos, ph)
                             : read_table(T, k, T.frames > 1 ? pos / (T.frames - 1) : 0, ph, cfg.interp);
        };
        for (int u = 0; u < N; ++u) {
            const double sp = N > 1 ? 2.0 * u / (N - 1) - 1 : 0;  // -1 .. 1 across the stack
            double inc = O.inc * (N > 1 ? std::exp2(sp * O.detune / 12.0) : 1.0);
            for (int s = 0; s < 2; ++s)
                if ((s ? O.w2 : O.w1) == W_TZFM)
                    inc *= 1 + 4 * (s ? O.w2amt : O.w1amt) * other;
            double p = O.ph[u];
            O.ph[u] += inc;
            O.ph[u] -= std::floor(O.ph[u]);
            double gain = 1;
            int oddeven = -1;
            for (int s = 0; s < 2; ++s) {
                const int w = s ? O.w2 : O.w1;
                const double a = s ? O.w2amt : O.w1amt;
                switch (w) {
                case W_OFF: case W_TZFM: break;
                case W_FM: p += 2 * a * other; break;
                case W_FM_SUB: p += 2 * a * v.sub; break;
                case W_FM_NOISE: p += 2 * a * v.noise; break;
                case W_AM: gain *= 1 - a + a * 0.5 * (other + 1); break;
                case W_RM: gain *= 1 - a + a * other; break;
                case W_FLIP: gain *= warp_gain(w, a, p); break;
                case W_ODDEVEN: oddeven = s; break;
                case W_SYNCWIN: gain *= warp_gain(w, a, p); p = warp_phase(w, a, p); ++fns; break;
                case W_BEND_P: case W_BEND_M: case W_BEND_PM: case W_ASYM_P: case W_ASYM_M: case W_ASYM_PM:
                case W_MIRROR:
                    p = warp_phase(w, a, p);
                    fns += 2;  // u^p = exp2(p log2 u): two table functions
                    macs["osc"] += 1;
                    break;
                default: p = warp_phase(w, a, p); macs["osc"] += 1; break;
                }
                p -= std::floor(p);
            }
            double smp = read(p);
            if (oddeven >= 0) {  // e = -1: odd part, +1: even part
                const double e = 2 * (oddeven ? O.w2amt : O.w1amt) - 1;
                double q = p + 0.5;
                q -= std::floor(q);
                smp = (smp + e * read(q)) / (1 + std::fabs(e));
            }
            // blend: centre voice(s) vs the others; width: the stack spread over the stereo field
            const bool centre = N <= 2 || (N % 2 ? u == N / 2 : (u == N / 2 - 1 || u == N / 2));
            const double g = N <= 2 ? 1.0 : centre ? std::min(1.0, (1 - O.blend) / 0.25) : std::min(1.0, O.blend / 0.75);
            const double x = smp * gain * g;
            sum += x;
            const double pan = std::clamp(O.pan + O.width * sp, -1.0, 1.0);
            sl += x * std::sqrt(0.5 * (1 - pan));
            sr += x * std::sqrt(0.5 * (1 + pan));
            macs["osc"] += 6;
        }
        const double norm = 1 / std::sqrt(double(N));
        double out = sum * norm;
        if (cfg.fixed)
            out = qz(sat(out, 1.99), 17);
        O.prev = out;
        l += sl * norm * O.level * std::sqrt(2.0);
        r += sr * norm * O.level * std::sqrt(2.0);
        return out;
    }

    double sub_sample(Voice& v, int shape) {
        const double p = v.subPh, dt = v.subInc;
        v.subPh += dt;
        v.subPh -= std::floor(v.subPh);
        auto blep = [&](double t) {  // polyBLEP residual
            if (t < dt) { t /= dt; return t + t - t * t - 1; }
            if (t > 1 - dt) { t = (t - 1) / dt; return t * t + t + t + 1; }
            return 0.0;
        };
        switch (shape) {
        case 1: return 1 - 4 * std::fabs(p - 0.5);  // triangle (corners only: little aliasing)
        case 2: return 1 - 2 * p + blep(p);
        case 3: {
            double q = p + 0.5;
            q -= std::floor(q);
            return (p < 0.5 ? 1.0 : -1.0) - blep(p) + blep(q);
        }
        default: return std::sin(2 * M_PI * p);
        }
    }

    // the fixed-point read: s16 table, Q17 sinc coefficients (interpolated between 256 phases),
    // 32-bit phase, Q16 frame crossfade
    double read_q(const wt::Table& T, int k, double posFrames, double ph) const {
        const int N = T.size(k);
        const uint32_t P32 = uint32_t(int64_t(std::floor(ph * 4294967296.0)) & 0xFFFFFFFF);
        const int sh = 32 - __builtin_ctz(N);
        const uint32_t idx = P32 >> sh, frac = uint32_t((uint64_t(P32) << (32 - sh)) >> 16) & 0xFFFF;
        const int f0 = std::min(int(posFrames), T.frames - 1), f1 = std::min(f0 + 1, T.frames - 1);
        const int u = int((posFrames - f0) * 65536.0);
        auto one = [&](int f) -> int64_t {
            if (cfg.interp == Interp::SINC8) {
                static std::vector<int32_t> C;
                const auto& S = sinc_table();
                if (C.empty()) {
                    C.resize(S.c.size());
                    for (size_t i = 0; i < C.size(); ++i)
                        C[i] = int32_t(std::lrint(S.c[i] * 131072));
                }
                const int pi = int(frac >> 8), pf = int(frac & 255);
                int64_t acc = 0;
                for (int j = 0; j < S.T; ++j) {
                    const int c0 = C[pi * S.T + j], c1 = C[(pi + 1) * S.T + j];
                    acc += int64_t(c0 + (((c1 - c0) * pf) >> 8)) * T.atq(k, f, int(idx) - (S.T / 2 - 1) + j);
                }
                return acc >> 1;  // Q16 of a s16 sample -> s33 .. keep 16 fraction bits
            }
            if (cfg.interp == Interp::HERM) {
                const int64_t a = T.atq(k, f, int(idx) - 1), b = T.atq(k, f, int(idx)), c = T.atq(k, f, int(idx) + 1),
                              d = T.atq(k, f, int(idx) + 2);
                const double fr = frac / 65536.0;
                const double c1 = 0.5 * (c - a), c2 = a - 2.5 * b + 2 * c - 0.5 * d, c3 = 0.5 * (d - a) + 1.5 * (b - c);
                return int64_t(std::llrint((((c3 * fr + c2) * fr + c1) * fr + b) * 65536.0));
            }
            const int64_t a = T.atq(k, f, int(idx)), b = T.atq(k, f, int(idx) + 1);
            return (a << 16) + (b - a) * int64_t(frac);
        };
        const int64_t a = one(f0);
        const int64_t v = u > 0 ? a + (((one(f1) - a) * u) >> 16) : a;
        return double(v) / 65536.0 / 32767.0 / T.qscale;
    }
};

Engine::Engine(const Config& cfg, const Song& song) : p_(std::make_unique<Impl>()) {
    Impl& I = *p_;
    I.cfg = cfg;
    I.song = song;
    I.rng ^= cfg.seed * 0x85EBCA6Bu;
    for (int i = 0; i < kNP; ++i)
        I.base[i] = kParams[i].def;
    for (auto& [k, v] : song.set) {
        const int i = pidx(k);
        if (i < 0)
            std::fprintf(stderr, "unknown parameter '%s'\n", k.c_str());
        else
            I.base[i] = v;
    }
    for (int o = 0; o < 2; ++o) {
        const std::string key = o ? "tableB" : "tableA";
        const std::string spec = song.tables.count(key) ? song.tables.at(key) : "basic";
        if (!wt::get(spec, I.tab[o], cfg.nmin))
            std::fprintf(stderr, "can't load table '%s'\n", spec.c_str());
    }
    for (auto& m : song.mods) {
        Impl::Route r;
        r.src = src_index(m.src, r.bip);
        r.dst = pidx(m.dst);
        r.amt = m.amount;
        if (r.src < 0 || r.dst < 0)
            std::fprintf(stderr, "bad mod route %s -> %s\n", m.src.c_str(), m.dst.c_str());
        else
            I.routes.push_back(r);
    }
    I.voices.resize(std::max(1, cfg.voices));
    for (auto& v : I.voices) {
        for (auto& o : v.osc)
            o.cache.nbuf = cfg.cacheBufs;
        for (int l = 0; l < 4; ++l)
            v.lfo[l].rng = 0x1234567u + 7919u * l + cfg.seed;
    }
    I.mem.linesPerSample = cfg.sdramLines;
    I.qf.on = cfg.fixed;
    for (auto& v : I.voices)
        v.filt.set_rate(kFs * cfg.rate);
    for (int r = cfg.rate; r > 1; r /= 2)
        I.hb.emplace_back(71);
    for (auto& h : I.hb) {
        I.decL.push_back(std::make_unique<Decim2>(h));
        I.decR.push_back(std::make_unique<Decim2>(h));
    }
}

Engine::~Engine() = default;

std::vector<std::string> Engine::param_names() {
    std::vector<std::string> v;
    for (auto& p : kParams)
        v.push_back(p.name);
    return v;
}

void Engine::render(std::vector<float>& L, std::vector<float>& R) {
    Impl& I = *p_;
    const Song& s = I.song;
    const double spb = kFs * 60.0 / s.bpm;  // samples per beat
    double endBeat = s.endBeat;
    if (endBeat < 0) {
        for (auto& nt : s.notes)
            endBeat = std::max(endBeat, nt.beat + nt.beats);
        endBeat += 1 + I.base[pidx("env1_r")] * s.bpm / 60.0;
    }
    const uint64_t total = uint64_t(endBeat * spb);
    L.assign(total, 0);
    R.assign(total, 0);
    struct Ev {
        uint64_t at;
        int kind;  // 0 off, 1 on, 2 param
        int note;
        double vel;
        int param;
        double value;
    };
    std::vector<Ev> evs;
    for (auto& nt : s.notes) {
        evs.push_back({uint64_t(nt.beat * spb), 1, nt.midi, nt.vel, -1, 0});
        evs.push_back({uint64_t((nt.beat + nt.beats) * spb), 0, nt.midi, 0, -1, 0});
    }
    for (auto& c : s.changes)
        evs.push_back({uint64_t(c.beat * spb), 2, 0, 0, pidx(c.name), c.value});
    std::stable_sort(evs.begin(), evs.end(), [](auto& a, auto& b) { return a.at < b.at || (a.at == b.at && a.kind < b.kind); });
    size_t ei = 0;
    const int Rt = I.cfg.rate;
    const int div = std::max(1, I.cfg.ctrlDiv);
    std::vector<double> hiL(Rt), hiR(Rt);
    I.st = &st_;
    double vol = I.base[pidx("volume")];
    for (I.n = 0; I.n < total; ++I.n) {
        while (ei < evs.size() && evs[ei].at <= I.n) {
            const Ev& e = evs[ei++];
            if (e.kind == 1)
                I.note_on(e.note, e.vel);
            else if (e.kind == 0)
                I.note_off(e.note);
            else if (e.param >= 0)
                I.base[e.param] = e.value;
        }
        vol = I.base[pidx("volume")];
        const bool ctl = I.n % div == 0;
        std::fill(hiL.begin(), hiL.end(), 0.0);
        std::fill(hiR.begin(), hiR.end(), 0.0);
        for (auto& v : I.voices) {
            if (!v.active)
                continue;
            if (ctl)
                I.control(v, div / kFs);
            if (!v.active)
                continue;
            const double filtOn = v.mod[pidx("filt_on")] > 0.5;
            // the modulator is computed first: B first when A is cross-modulated by B (and B
            // isn't by A); with both, A gets B's previous sample
            auto xm = [](const Osc& O) { return warp_is_xmod(O.w1) || warp_is_xmod(O.w2); };
            const bool bFirst = xm(v.osc[0]) && !xm(v.osc[1]);
            const bool stereo = I.cfg.stereo && ((v.osc[0].on && v.osc[0].uni > 1 && v.osc[0].width > 0) ||
                                (v.osc[1].on && v.osc[1].uni > 1 && v.osc[1].width > 0) ||
                                (v.osc[0].on && v.osc[0].pan != 0) || (v.osc[1].on && v.osc[1].pan != 0));
            const bool subOn = v.mod[pidx("sub_on")] > 0.5, noiseOn = v.mod[pidx("noise_on")] > 0.5;
            const int subShape = int(v.mod[pidx("sub_shape")]);
            const double subLev = v.mod[pidx("sub_level")], noiseLev = v.mod[pidx("noise_level")];
            const bool aF = v.mod[pidx("a_filt")] > 0.5, bF = v.mod[pidx("b_filt")] > 0.5;
            const bool sF = v.mod[pidx("sub_filt")] > 0.5, nF = v.mod[pidx("noise_filt")] > 0.5;
            const double mix = std::clamp(v.mod[pidx("filt_mix")], 0.0, 1.0);
            for (int r = 0; r < Rt; ++r) {
                v.sub = subOn ? I.sub_sample(v, subShape) : 0;
                v.noise = noiseOn ? 2 * I.frand() - 1 : 0;
                double aL = 0, aR = 0, bL = 0, bR = 0;
                if (bFirst) {
                    const double b = I.osc_sample(v, 1, v.osc[0].prev, bL, bR);
                    I.osc_sample(v, 0, b, aL, aR);
                } else {
                    const double a = I.osc_sample(v, 0, v.osc[1].prev, aL, aR);
                    I.osc_sample(v, 1, a, bL, bR);
                }
                double fL = 0, fR = 0, dL = 0, dR = 0;
                (aF ? fL : dL) += aL;
                (aF ? fR : dR) += aR;
                (bF ? fL : dL) += bL;
                (bF ? fR : dR) += bR;
                const double sub = v.sub * subLev, noise = v.noise * noiseLev;
                (sF ? fL : dL) += sub;
                (sF ? fR : dR) += sub;
                (nF ? fL : dL) += noise;
                (nF ? fR : dR) += noise;
                if (!stereo) {  // a mono voice: one filter, the same signal on both sides
                    fL = fR = 0.5 * (fL + fR);
                    dL = dR = 0.5 * (dL + dR);
                }
                double yL = fL, yR = fR;
                if (filtOn) {
                    yL = v.filt.process(fL * v.drive, v.fc, v.res, v.oct, v.mx, v.my, I.qf);
                    I.macs["filter"] += v.filt.take_macs();
                    I.fns += v.filt.take_fns();
                    if (stereo) {
                        yR = v.filtR.process(fR * v.drive, v.fc, v.res, v.oct, v.mx, v.my, I.qf);
                        I.macs["filter"] += v.filtR.take_macs();
                        I.fns += v.filtR.take_fns();
                    } else {
                        yR = yL;
                    }
                    yL = fL + mix * (yL - fL);
                    yR = fR + mix * (yR - fR);
                }
                if (I.cfg.fixed) {
                    yL = qz(sat(yL, 63.99), 24);  // s32 Q6.24, like the filter states
                    yR = qz(sat(yR, 63.99), 24);
                }
                // amp: ramp across the control block
                const double frac = ((I.n % div) * Rt + r + 1) / double(div * Rt);
                const double g = (v.amp0 + (v.amp1 - v.amp0) * frac) * vol * std::sqrt(0.5);
                hiL[r] += (yL + dL) * g;
                hiR[r] += (yR + dR) * g;
                I.macs["mix"] += 8;
            }
        }
        // decimate R -> 1
        double l = 0, rr = 0;
        if (Rt == 1) {
            l = hiL[0];
            rr = hiR[0];
        } else {
            std::vector<double> xl(hiL), xr(hiR);
            for (size_t st = 0; st < I.decL.size(); ++st) {
                std::vector<double> yl(xl.size() / 2), yr(xr.size() / 2);
                for (size_t i = 0; i < yl.size(); ++i) {
                    yl[i] = I.decL[st]->push2(xl[2 * i], xl[2 * i + 1]);
                    yr[i] = I.decR[st]->push2(xr[2 * i], xr[2 * i + 1]);
                }
                xl.swap(yl);
                xr.swap(yr);
                I.macs["decim"] += 2 * yl.size() * ((I.hb[st].nonzero_taps() + 1) / 2);
            }
            l = xl[0];
            rr = xr[0];
        }
        if (I.cfg.fixed) {
            l = qz(l, 23);
            rr = qz(rr, 23);
        }
        st_.peak = std::max({st_.peak, std::fabs(l), std::fabs(rr)});
        if (std::fabs(l) > 1 || std::fabs(rr) > 1)
            ++st_.clips;
        L[I.n] = float(l);
        R[I.n] = float(rr);
        I.mem.tick(I.n);
    }
    st_.samples = total;
    uint64_t sum = 0;
    for (auto& [k, v] : I.macs)
        sum += v;
    st_.macsPerSample = double(sum) / std::max<uint64_t>(1, total);
    st_.macs = I.macs;
    st_.tableReadsPerSample = double(I.reads) / std::max<uint64_t>(1, total);
    st_.fnsPerSample = double(I.fns) / std::max<uint64_t>(1, total);
    for (auto& v : I.voices)
        for (auto& o : v.osc) {
            st_.frameLoads += o.cache.loads;
            st_.lagSamples += o.cache.lagSamples;
            st_.oscSamples += o.cache.calls;
            st_.lagSum += o.cache.lagSum;
            st_.maxLagFrames = std::max(st_.maxLagFrames, o.cache.maxLag);
        }
    st_.sdramMean = double(I.mem.lines) / std::max<uint64_t>(1, total);
    st_.sdramPeak = I.mem.peak;
}

}  // namespace synth
