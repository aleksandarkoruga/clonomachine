// Bit-exact golden model, see sampler_model.h. Compile with -std=c++20 (arithmetic >>).
#include "sampler_model.h"

#include <algorithm>
#include <bit>
#include <cstdio>
#include <fstream>

namespace smp {

static bool readHex(const std::string& path, uint32_t* dst, int n) {
    std::ifstream f(path);
    if (!f)
        return false;
    for (int i = 0; i < n; ++i) {
        std::string s;
        if (!(f >> s))
            return false;
        dst[i] = static_cast<uint32_t>(std::stoul(s, nullptr, 16));
    }
    return true;
}

bool Tables::load(const std::string& dir) {
    return readHex(dir + "/hann.hex", hann, 1025) && readHex(dir + "/exp2.hex", exp2, 257);
}

static inline int32_t sat(int64_t v, int bits) {
    const int64_t hi = (int64_t(1) << (bits - 1)) - 1, lo = -(int64_t(1) << (bits - 1));
    return static_cast<int32_t>(v > hi ? hi : (v < lo ? lo : v));
}

static const uint32_t kRampStep = 1366; // 65536/48 rounded up: 1 ms ramp at 48 kHz

void Slot::reset(int idx) {
    *this = Slot();
    r[SMP_R_CTRL] = 0;
    r[SMP_R_FRAMES] = 128;
    r[SMP_R_PHASOR_INC] = 1u << 24;
    r[SMP_R_PITCH] = 1u << 24;
    r[SMP_R_WIN_INC] = 2236962; // 2^32/1920 (40 ms)
    r[SMP_R_OVERLAP] = 4u << 24;
    r[SMP_R_AMP_TARGET] = 16384;
    r[SMP_R_PROB] = 65536;
    r[SMP_R_MAX_GRAINS] = 32;
    r[SMP_R_ENV_DUR] = 96000;
    r[SMP_R_INTERVAL] = 4800;
    r[SMP_R_AMP] = 4096;
    r[SMP_R_ALGO_AMP] = 32768;
    rng = 0x9E3779B9u + static_cast<uint32_t>(idx);
}

uint32_t Slot::xorshift() {
    uint32_t x = rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng = x;
    return x;
}

int32_t Slot::step(const std::vector<int16_t>& mem, const Tables& t) {
    const uint32_t ctrl = r[SMP_R_CTRL];
    const uint32_t stb = strobe;
    strobe = 0;
    const int64_t F = static_cast<int64_t>(r[SMP_R_FRAMES] & 0xFFFFFF) << 32;
    if (!(ctrl & SMP_CTRL_ENABLE) || r[SMP_R_FRAMES] < 128) {
        gOut = 0;
        gateState = 0;
        pvRestart = false;
        return 0;
    }
    const bool gtrig = stb & SMP_STROBE_TRIG; // note-on: gate + stutter restart
    const int64_t posQ32 = static_cast<int64_t>(r[SMP_R_POS]) << 24;
    if (stb & SMP_STROBE_BG_RESET)
        bgPhasor = 0;
    if (bgPhasor >= F) // buffer switched to a shorter one
        bgPhasor = 0;
    const int64_t posEff = posQ32 < F ? posQ32 : 0;
    int64_t posBg = posEff; // (POS + background) mod F when POS_BG
    if (ctrl & SMP_CTRL_POS_BG) {
        posBg += bgPhasor;
        if (posBg >= F)
            posBg -= F;
    }

    // 0. stutter: counted pulse train retriggering the latched position.
    //    TRIG loads REPEATS; one pulse every INTERVAL samples until the count is spent.
    const uint32_t interval = r[SMP_R_INTERVAL] & 0x3FFF;
    bool retrig = false;
    if (gtrig) {
        posLatchQ = posBg;
        stutLeft = r[SMP_R_REPEATS] & 0x7F;
        stutPh = 0;
    } else if (stutLeft) {
        if (++stutPh >= interval) {
            stutPh = 0;
            --stutLeft;
            retrig = true;
        }
    }
    const bool trig = gtrig || retrig; // playhead jump + immediate onset
    const int64_t latQ32 = posLatchQ;

    // 1. strobes
    if (stb & SMP_STROBE_KILL)
        active = 0;
    if (phasor >= F) // buffer switched to a shorter one
        phasor = 0;
    if (stb & SMP_STROBE_SEEK)
        phasor = posEff;
    if (trig)
        phasor = latQ32 < F ? latQ32 : 0;

    // 2. render active grains
    int64_t S = 0;
    for (uint32_t m = active; m; m &= m - 1) {
        const int v = std::countr_zero(m);
        Grain& gr = g[v];
        const uint32_t base = gr.base, frames = gr.frames;
        const int64_t Fg = static_cast<int64_t>(frames) << 32;
        const uint32_t idx = static_cast<uint32_t>(gr.readPhase >> 32);
        const int32_t fr = static_cast<int32_t>((gr.readPhase >> 16) & 0xFFFF);
        const uint32_t idx1 = (idx + 1 == frames) ? 0 : idx + 1;
        const int32_t s0 = mem[(base + SMP_PREPAD + idx) % mem.size()];
        const int32_t s1 = mem[(base + SMP_PREPAD + idx1) % mem.size()];
        const int32_t s = s0 + static_cast<int32_t>((static_cast<int64_t>(s1 - s0) * fr) >> 16);
        const uint32_t hi = gr.winPhase >> 22;
        const int32_t wf = static_cast<int32_t>((gr.winPhase >> 6) & 0xFFFF);
        const int32_t h0 = static_cast<int32_t>(t.hann[hi]), h1 = static_cast<int32_t>(t.hann[hi + 1]);
        const int32_t w = h0 + (((h1 - h0) * wf) >> 16);
        S += static_cast<int64_t>(s) * w;

        const uint32_t prev = gr.winPhase;
        gr.winPhase += gr.winInc;
        if (gr.winPhase < prev)
            active &= ~(1u << v);
        gr.readPhase += gr.readInc;
        if (gr.readPhase >= Fg)
            gr.readPhase -= Fg;
        else if (gr.readPhase < 0)
            gr.readPhase += Fg;
    }

    // 3. gain smoothing (tau 10 ms) and normalisation
    {
        const int64_t target = static_cast<int64_t>(r[SMP_R_AMP_TARGET] & 0x1FFFF) << 8;
        amp = static_cast<uint32_t>(amp + (((target - amp) * 136) >> 16));
    }
    const int32_t y = sat((S * static_cast<int64_t>(amp >> 8)) >> 24, 24);

    // 4. position pin
    if (ctrl & SMP_CTRL_POS_PIN)
        phasor = posBg;
    pvPh = phasor;
    pvRestart = trig || (stb & (SMP_STROBE_SEEK | SMP_STROBE_KILL));

    // 5. onset clock
    const uint64_t ti = ((((static_cast<uint64_t>(r[SMP_R_WIN_INC]) * r[SMP_R_OVERLAP]) >> 24) * hop) >> 16);
    const uint32_t trigInc = ti > 0xFFFFFFFFull ? 0xFFFFFFFFu : static_cast<uint32_t>(ti);
    bool onset = false;
    uint32_t frac = 0;
    if (trig) {
        trigPhase = 0;
        onset = true;
    } else {
        const uint64_t n = static_cast<uint64_t>(trigPhase) + trigInc;
        trigPhase = static_cast<uint32_t>(n);
        if (trigInc && (n >> 32)) {
            onset = true;
            frac = static_cast<uint32_t>((static_cast<uint64_t>(trigPhase) << 16) / trigInc);
            if (frac > 65535)
                frac = 65535;
        }
    }
    if (onset) {
        const uint32_t r1 = xorshift(), r2 = xorshift(), r3 = xorshift(), r4 = xorshift();
        const int32_t sepT = static_cast<int32_t>(((r[SMP_R_SEPARATION] & 0xFFFF) * 58982u) >> 16);
        hop = static_cast<uint32_t>(65536 + ((sepT * static_cast<int16_t>(r1 >> 16)) >> 15));
        const uint32_t freeMask = ~active;
        const bool fire = (r[SMP_R_PROB] & 0x1FFFF) > (r2 >> 16) && freeMask
            && static_cast<uint32_t>(std::popcount(active)) < (r[SMP_R_MAX_GRAINS] & 63);
        if (fire) {
            const int v = std::countr_zero(freeMask);
            const int32_t e12 = (static_cast<int16_t>(r[SMP_R_RANDPITCH]) * static_cast<int16_t>(r3 >> 16)) >> 15;
            const int ei = e12 >> 12;
            const int ef = e12 & 0xFFF;
            const int li = ef >> 4, lo = ef & 15;
            const int32_t e0 = static_cast<int32_t>(t.exp2[li]), e1 = static_cast<int32_t>(t.exp2[li + 1]);
            const int64_t m = e0 + (((e1 - e0) * lo) >> 4);
            int64_t pe = (static_cast<int64_t>(static_cast<int32_t>(r[SMP_R_PITCH])) * m) >> 16;
            pe = ei >= 0 ? (pe << ei) : (pe >> -ei);
            const int64_t readInc = pe << 8;
            const int64_t off = static_cast<int64_t>(static_cast<uint64_t>(r4) * (r[SMP_R_RANDPOS] & 0xFFFFFF));
            int64_t rp = phasor + off + ((static_cast<int64_t>(frac) * readInc) >> 16);
            while (rp >= F)
                rp -= F;
            while (rp < 0)
                rp += F;
            Grain& gr = g[v];
            gr.winInc = r[SMP_R_WIN_INC];
            gr.winPhase = static_cast<uint32_t>((static_cast<uint64_t>(frac) * r[SMP_R_WIN_INC]) >> 16);
            gr.readPhase = rp;
            gr.readInc = readInc;
            gr.base = r[SMP_R_BASE] & 0xFFFFFF;
            gr.frames = r[SMP_R_FRAMES] & 0xFFFFFF;
            active |= 1u << v;
        }
    }

    // 6. playhead and background phasor
    const int64_t inc = static_cast<int64_t>(static_cast<int32_t>(r[SMP_R_PHASOR_INC])) << 8;
    phasor += inc;
    if (phasor >= F)
        phasor -= F;
    else if (phasor < 0)
        phasor += F;
    bgPhasor += inc;
    if (bgPhasor >= F)
        bgPhasor -= F;
    else if (bgPhasor < 0)
        bgPhasor += F;

    const bool stutActive = stutLeft != 0;
    const int32_t z = y;

    // 8. gate envelope
    if (stb & SMP_STROBE_GATE_OFF)
        gate = false;
    if (gtrig) {
        gate = true;
        envLeft = r[SMP_R_ENV_DUR] & 0xFFFFFF;
    } else if (gate && !(ctrl & SMP_CTRL_ENV_HOLD)) {
        if (envLeft)
            --envLeft;
        if (!envLeft)
            gate = false;
    }
    if (gate)
        level = level + kRampStep > 65536 ? 65536 : level + kRampStep;
    else
        level = level > kRampStep ? level - kRampStep : 0;

    // 9. output gain chain
    int64_t o = (static_cast<int64_t>(z) * (r[SMP_R_AMP] & 0xFFFF)) >> 12;
    o = (o * level) >> 16;
    o = (o * (r[SMP_R_ALGO_AMP] & 0xFFFF)) >> 15;
    gOut = sat(o, 24);

    gateState = stutActive ? 4 : gate ? (level < 65536 ? 1 : 2) : (level ? 3 : 0);
    return gOut;
}

Engine::Engine(int n): nSlots(n), slots(n), pvs(n) {
    for (int i = 0; i < n; ++i)
        slots[i].reset(i);
}

void Engine::write(uint32_t addr, uint32_t data) {
    const uint32_t s = (addr >> 5) & 7, reg = addr & 31;
    if (s == SMP_GLOBAL_SLOT) {
        switch (reg) {
        case SMP_G_MASTER: master = data & 0xFFFF; break;
        case SMP_G_BEAT_INC: beatInc = data; break;
        case SMP_G_BEAT_CTRL:
            if (data & 1)
                beat = 0;
            break;
        default: break;
        }
        return;
    }
    if (s >= static_cast<uint32_t>(nSlots) || reg >= SMP_NUM_SLOT_REGS)
        return;
    Slot& sl = slots[s];
    if (reg == SMP_R_STROBE) {
        sl.strobe |= data & 31;
        return;
    }
    sl.r[reg] = data;
    if (reg == SMP_R_SEED)
        sl.rng = data ? data : 0x9E3779B9u;
}

uint32_t Engine::status(uint32_t idx) const {
    if (idx < 56) {
        const uint32_t s = idx >> 3, i = idx & 7;
        if (s >= static_cast<uint32_t>(nSlots))
            return 0;
        const Slot& sl = slots[s];
        switch (i) {
        case SMP_S_PHASOR: return static_cast<uint32_t>(sl.phasor >> 24);
        case SMP_S_ACTIVE: return static_cast<uint32_t>(std::popcount(sl.active));
        case SMP_S_GATE: return sl.gateState;
        case SMP_S_OUT: return static_cast<uint32_t>(sl.lastOut) & 0xFFFFFF;
        case SMP_S_PV: return (pvs[s].active ? 1u : 0u) | (std::min<uint32_t>(pvs[s].x, 65535) << 16);
        default: return 0;
        }
    }
    switch (idx) {
    case SMP_SG_BEAT_HI: return static_cast<uint32_t>(beat >> 32);
    case SMP_SG_BEAT_LO: return static_cast<uint32_t>(beat);
    case SMP_SG_SAMPLES: return samples;
    case SMP_SG_MAGIC: return SMP_MAGIC;
    default: return 0;
    }
}

static inline int64_t wrapPos(int64_t p, int64_t F) {
    p %= F;
    return p < 0 ? p + F : p;
}

void Engine::pvIssue(int s, uint32_t c, int64_t pos, int64_t inc, bool reset, int win) {
    const Slot& sl = slots[s];
    PvSlot& p = pvs[s];
    PvJob j;
    j.frames = sl.r[SMP_R_FRAMES] & 0xFFFFFF;
    j.base = sl.r[SMP_R_BASE] & 0xFFFFFF;
    j.pos = wrapPos(pos, static_cast<int64_t>(j.frames) << 32);
    j.inc = inc;
    const int32_t pitch = static_cast<int32_t>(sl.r[SMP_R_PITCH]);
    j.rate = std::clamp<uint32_t>(static_cast<uint32_t>(pitch < 0 ? -int64_t(pitch) : pitch), 1u << 20, 1u << 26);
    j.reset = reset;
    j.win = win;
    pv.transientSuppression = !(sl.r[SMP_R_ALGO] & SMP_ALGO_NO_TRANSIENT);
    pv.run(j, p.st, mem);
    pv.accumulate(p.ring.data(), c, win);
    ++pvJobs;
}

// Phase-vocoder path and output mix of slot s for this sample (after Slot::step).
int32_t Engine::mixSlot(int s) {
    Slot& sl = slots[s];
    PvSlot& p = pvs[s];
    const uint32_t e = samples;
    constexpr uint32_t mask = PV_RING - 1;
    const uint32_t ctrl = sl.r[SMP_R_CTRL];
    const bool enabled = (ctrl & SMP_CTRL_ENABLE) && sl.r[SMP_R_FRAMES] >= 128;
    const uint32_t algo = enabled ? (sl.r[SMP_R_ALGO] & SMP_ALGO_MASK) : 0;

    bool restart = false;
    if (!enabled && p.active) { // hard stop, like the grain path
        p.active = false;
        p.st.valid = false;
        p.x = 0;
        std::fill(p.ring.begin(), p.ring.end(), 0);
    }
    if (algo != p.algo) {
        p.algo = algo;
        p.xfStart = e + SMP_LATENCY;
        if (algo == SMP_ALGO_PV && !p.active) {
            p.active = true;
            restart = true;
        }
    }
    if (p.active && sl.pvRestart)
        restart = true;

    const int64_t inc = ((ctrl & SMP_CTRL_POS_PIN) && !(ctrl & SMP_CTRL_POS_BG))
        ? 0 : static_cast<int64_t>(static_cast<int32_t>(sl.r[SMP_R_PHASOR_INC])) << 8;
    if (restart) {
        // old content: fades out from c0, everything later is cleared
        const uint32_t c0 = e + SMP_LATENCY;
        const PvTables& t = pv.tables();
        for (uint32_t u = 0; u < PV_FADE; ++u) {
            int32_t& v = p.ring[(c0 + u) & mask];
            v = static_cast<int32_t>((static_cast<int64_t>(v) * t.fadeOut[u] + 32768) >> 16);
        }
        for (uint32_t tt = c0 + PV_FADE; tt != e + PV_RING; ++tt)
            p.ring[tt & mask] = 0;
        for (int k = 0; k < 3; ++k)
            pvIssue(s, c0 + k * PV_H, sl.pvPh + int64_t(k) * PV_H * inc, inc, k == 0, PV_WIN_R0 + k);
        p.nextC = c0 + 3 * PV_H;
    } else if (p.active && static_cast<int32_t>(e - (p.nextC - 2 * PV_H - SMP_PV_LOOKAHEAD)) >= 0) {
        const int32_t ahead = static_cast<int32_t>(p.nextC - SMP_LATENCY - e);
        pvIssue(s, p.nextC, sl.pvPh + int64_t(ahead) * inc, inc, false, PV_WIN_NORMAL);
        p.nextC += PV_H;
    }

    int32_t pvOut = 0;
    if (p.active) {
        int32_t& v = p.ring[e & mask];
        pvOut = sat(v, 24);
        v = 0;
    }
    const uint32_t di = e % SMP_LATENCY;
    const int32_t gD = p.dlOut[di];
    const uint32_t levelD = p.dlLevel[di];
    p.dlOut[di] = sl.gOut;
    p.dlLevel[di] = enabled ? sl.level : 0;

    if (static_cast<int32_t>(e - p.xfStart) >= 0) {
        constexpr uint32_t step = 65536 / SMP_ALGO_XF;
        if (algo == SMP_ALGO_PV)
            p.x = std::min<uint32_t>(p.x + step, 65536);
        else
            p.x = p.x > step ? p.x - step : 0;
    }
    if (p.active && p.algo != SMP_ALGO_PV && p.x == 0) {
        p.active = false;
        p.st.valid = false;
        std::fill(p.ring.begin(), p.ring.end(), 0);
    }

    int64_t o = (static_cast<int64_t>(pvOut) * (sl.r[SMP_R_AMP] & 0xFFFF)) >> 12;
    o = (o * levelD) >> 16;
    o = (o * (sl.r[SMP_R_ALGO_AMP] & 0xFFFF)) >> 15;
    const int64_t op = sat(o, 24);
    sl.lastOut = sat((static_cast<int64_t>(gD) * (65536 - p.x) + op * p.x) >> 16, 24);
    return sl.lastOut;
}

int32_t Engine::step() {
    int64_t sum = 0;
    for (int s = 0; s < nSlots; ++s) {
        slots[s].step(mem, tab);
        sum += mixSlot(s);
    }
    beat += beatInc;
    ++samples;
    return sat((sum * master) >> 12, 24);
}

} // namespace smp
