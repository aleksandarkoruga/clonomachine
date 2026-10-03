// Compare the fixed-point golden model against a float port of BufGrainPlay::next
// (SC plumbing stripped, same maths). Randomness is disabled in all cases, so the
// two must agree up to fixed-point rounding.
#include "sampler_model.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace ref {

static const double k2Pow32 = 4294967296.0;
static const double kU32Max = 4294967295.0;
static float gHann[1025];

static float hann_u32(uint32_t ph) {
    const uint32_t i = ph >> 22;
    const float f = static_cast<float>(ph & ((1u << 22) - 1u)) * (1.f / static_cast<float>(1u << 22));
    return gHann[i] + f * (gHann[i + 1] - gHann[i]);
}
static int64_t phase_inc(double x) {
    if (x > 1073741824.0) x = 1073741824.0;
    else if (x < -1073741824.0) x = -1073741824.0;
    return static_cast<int64_t>(x * k2Pow32);
}

struct BufGrainPlay {
    struct Voice { uint32_t winPhase, winInc; uint64_t readPhase; int64_t readInc; };
    uint64_t m_phasor = 0;
    uint32_t m_trigPhase = 0;
    float m_hopMul = 1.f;
    Voice m_voices[32] = {};
    uint32_t m_activeMask = 0;
    float m_amp = 1.f;

    float next(const std::vector<float>& data, double sr, float pos, float rate, float pitch, float grainDur,
               float overlap_) {
        const uint32_t frames = static_cast<uint32_t>(data.size());
        const double oneSamp = k2Pow32 / frames;
        const float ampCoef = static_cast<float>(1.0 - std::exp(-1.0 / (0.01 * sr)));
        const float overlap = std::max(overlap_, 0.f);
        const double grainSamps = std::max(static_cast<double>(grainDur) * sr, 1.0);
        const uint32_t winInc = static_cast<uint32_t>(std::clamp(k2Pow32 / grainSamps, 1.0, kU32Max));
        const uint32_t trigInc = static_cast<uint32_t>(std::clamp(
            static_cast<double>(winInc) * overlap * m_hopMul, 0.0, kU32Max));
        float outVal = 0.f;
        uint32_t m = m_activeMask;
        while (m) {
            const int v = __builtin_ctz(m);
            m &= m - 1;
            Voice& g = m_voices[v];
            const uint64_t p = static_cast<uint64_t>(static_cast<uint32_t>(g.readPhase >> 32)) * frames;
            const uint32_t idx = static_cast<uint32_t>(p >> 32);
            const float frac = static_cast<float>(static_cast<uint32_t>(p)) * (1.f / 4294967296.f);
            uint32_t idx1 = idx + 1;
            if (idx1 >= frames) idx1 = 0;
            outVal += (data[idx] + frac * (data[idx1] - data[idx])) * hann_u32(g.winPhase);
            const uint32_t prevWin = g.winPhase;
            g.winPhase += g.winInc;
            if (g.winPhase < prevWin) m_activeMask &= ~(1u << v);
            g.readPhase += static_cast<uint64_t>(g.readInc);
        }
        const float target = 1.f / std::max(overlap, 1.f);
        m_amp += ampCoef * (target - m_amp);
        const float out = outVal * m_amp;
        if (pos >= 0.f) {
            const double t = pos;
            m_phasor = static_cast<uint64_t>(static_cast<uint32_t>((t - std::floor(t)) * k2Pow32)) << 32;
        }
        const uint32_t prevTrig = m_trigPhase;
        m_trigPhase += trigInc;
        if (trigInc && m_trigPhase < prevTrig) {
            const double frac = std::min(static_cast<double>(m_trigPhase) / trigInc, 0.999999999);
            const uint32_t freeMask = ~m_activeMask;
            if (freeMask) {
                const int v = __builtin_ctz(freeMask);
                m_activeMask |= (1u << v);
                Voice& g = m_voices[v];
                const int64_t readInc = phase_inc(oneSamp * pitch);
                g.winInc = winInc;
                g.readInc = readInc;
                g.winPhase = static_cast<uint32_t>(frac * winInc);
                g.readPhase = m_phasor + static_cast<uint64_t>(static_cast<int64_t>(frac * readInc));
            }
        }
        m_phasor += static_cast<uint64_t>(phase_inc(oneSamp * rate));
        return out;
    }
};

} // namespace ref

struct Case { const char* name; float rate, pitch, grainDur, overlap; };

int main(int argc, char** argv) {
    const char* romDir = argc > 1 ? argv[1] : "../rtl";
    for (int i = 0; i <= 1024; ++i)
        ref::gHann[i] = static_cast<float>(0.5 - 0.5 * std::cos(2 * M_PI * i / 1024));

    // Test material: 2 s of decaying tone bursts every 1/8 s + low noise floor.
    const uint32_t frames = 96000;
    std::vector<float> data(frames);
    uint32_t lcg = 1;
    for (uint32_t i = 0; i < frames; ++i) {
        const double t = (i % 6000) / 48000.0;
        lcg = lcg * 1664525u + 1013904223u;
        const double noise = ((lcg >> 8) / 16777216.0 - 0.5) * 0.02;
        data[i] = static_cast<float>(0.7 * std::exp(-t * 30) * std::sin(2 * M_PI * (110 + 40 * (i / 6000)) * t) + noise);
    }
    std::vector<int16_t> pcm(frames);
    for (uint32_t i = 0; i < frames; ++i) {
        pcm[i] = static_cast<int16_t>(std::lrint(data[i] * 32767.0));
        data[i] = pcm[i] / 32768.0f; // reference sees exactly the quantised samples
    }

    const Case cases[] = {
        {"ola_unity", 1.f, 1.f, 0.04f, 4.f},
        {"ola_half_speed", 0.5f, 1.f, 0.04f, 4.f},
        {"ola_pitch_up", 1.f, 1.5f, 0.04f, 4.f},
        {"ola_reverse", -1.f, 1.f, 0.06f, 2.f},
        {"granular_pitch_dn", 0.75f, 0.7f, 0.0071f, 1.f},
        {"stretch_x4", 0.25f, 1.f, 0.05f, 3.f},
    };
    int fails = 0;
    for (const Case& c : cases) {
        smp::Engine e(1);
        if (!e.loadTables(romDir)) {
            std::fprintf(stderr, "cannot load ROM hex from %s\n", romDir);
            return 2;
        }
        // SDRAM image with pre/post padding exactly as firmware lays it out
        e.mem.assign(SMP_PREPAD + frames + SMP_POSTPAD, 0);
        for (uint32_t i = 0; i < SMP_PREPAD; ++i) e.mem[i] = pcm[frames - SMP_PREPAD + i];
        for (uint32_t i = 0; i < frames; ++i) e.mem[SMP_PREPAD + i] = pcm[i];
        for (uint32_t i = 0; i < SMP_POSTPAD; ++i) e.mem[SMP_PREPAD + frames + i] = pcm[i];

        const double grainSamps = c.grainDur * 48000.0;
        const uint32_t winInc = static_cast<uint32_t>(4294967296.0 / grainSamps);
        auto W = [&](int reg, uint32_t v) { e.write(SMP_ADDR(0, reg), v); };
        W(SMP_R_BASE, 0);
        W(SMP_R_FRAMES, frames);
        W(SMP_R_PHASOR_INC, static_cast<uint32_t>(static_cast<int32_t>(std::lrint(c.rate * 16777216.0))));
        W(SMP_R_PITCH, static_cast<uint32_t>(static_cast<int32_t>(std::lrint(c.pitch * 16777216.0))));
        W(SMP_R_WIN_INC, winInc);
        W(SMP_R_OVERLAP, static_cast<uint32_t>(std::lrint(c.overlap * 16777216.0)));
        W(SMP_R_AMP_TARGET, static_cast<uint32_t>(std::lrint(65536.0 / std::max(c.overlap, 1.f))));
        W(SMP_R_ENV_DUR, 0xFFFFFF);
        W(SMP_R_CTRL, SMP_CTRL_ENABLE | SMP_CTRL_ENV_HOLD);
        e.slots[0].gate = true; // reference has no gate: start fully open
        e.slots[0].level = 65536;

        ref::BufGrainPlay u;
        double errE = 0, sigE = 0, maxErr = 0;
        const int N = 96000;
        for (int n = 0; n < N; ++n) {
            const double a = e.step() / (256.0 * 32768.0);
            const double b = u.next(data, 48000.0, -1.f, c.rate, c.pitch, c.grainDur, c.overlap);
            if (n < 4800) continue; // let the gain smoother settle (different rounding)
            errE += (a - b) * (a - b);
            sigE += b * b;
            maxErr = std::max(maxErr, std::fabs(a - b));
        }
        const double snr = 10 * std::log10(sigE / std::max(errE, 1e-30));
        const bool ok = snr > 70.0;
        fails += !ok;
        std::printf("%-20s SNR vs float UGen %6.1f dB  max|err| %.2e  %s\n", c.name, snr, maxErr, ok ? "ok" : "FAIL");
    }
    return fails ? 1 : 0;
}
