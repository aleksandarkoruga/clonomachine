// Frame cache + SDRAM traffic model for the synth's oscillators.
//
// Every oscillator keeps whole frames in block RAM (a phase-modulated read can land anywhere in
// the frame, so streaming lines like the grains do would miss every few samples). It plays
// the two frames around its table position, crossfaded; a third buffer lets the next frame load
// while those two play. Frames come from SDRAM as 64-word lines through one queue shared by all
// oscillators: a level-k frame is max(1, N_k / 64) lines, served at 'linesPerSample' (what the
// grains leave free) after 'latency' samples.
//
// When the position moves faster than frames arrive, the oscillator plays the loaded frames
// nearest to it (the position is clamped into them) and always asks for the frames at the
// current position, so a fast scan skips frames instead of falling behind. The counters say
// how far the played position was from the wanted one.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace synth {

struct Sdram {
    double linesPerSample = 6.0;  // capacity left for the synth
    double latency = 2.0;         // samples from request to the first word
    double busyUntil = 0;         // the queue's end, in samples
    uint64_t lines = 0, win = 0;
    double peak = 0;
    // a load of n lines requested at time t (samples): when it's complete
    double load(double t, int n) {
        const double start = std::max(t + latency, busyUntil);
        busyUntil = start + n / linesPerSample;
        lines += n;
        win += n;
        return busyUntil;
    }
    void tick(uint64_t n) {  // once per output sample
        if ((n & 63) == 63) {
            peak = std::max(peak, win / 64.0);
            win = 0;
        }
    }
};

struct FrameCache {
    struct Buf {
        int frame = -1, level = -1;
        double readyAt = 1e300;
        double used = -1;  // last time it was played
    };
    static constexpr int kMax = 4;
    int nbuf = 3;
    Buf b[kMax];
    bool fresh = true;  // note start: nothing valid to play
    // stats
    uint64_t loads = 0, lagSamples = 0, calls = 0;
    double lagSum = 0, maxLag = 0;  // |played - wanted| in frames

    int find(int f, int k) const {
        for (int i = 0; i < nbuf; ++i)
            if (b[i].frame == f && b[i].level == k)
                return i;
        return -1;
    }
    bool ready(int i, double t) const { return i >= 0 && b[i].readyAt <= t; }
    // start loading frame f, level k into the least recently played buffer that isn't in use
    void request(int f, int k, double t, int words, Sdram& mem, int keepA, int keepB) {
        if (f < 0 || find(f, k) >= 0)
            return;
        int v = -1;
        for (int i = 0; i < nbuf; ++i) {
            if (i == keepA || i == keepB || (b[i].frame >= 0 && b[i].readyAt > t))
                continue;  // playing, or still loading
            if (v < 0 || b[i].used < b[v].used)
                v = i;
        }
        if (v < 0)
            return;
        b[v].frame = f;
        b[v].level = k;
        b[v].used = t;
        b[v].readyAt = mem.load(t, std::max(1, words / 64));
        ++loads;
    }
    // wanted: position r (frames) at level k at time t. Returns the position to play and, in
    // levelOut, the level of the frames it uses (-1: nothing loaded yet, play silence)
    double resolve(double r, int k, int frames, double t, int words, Sdram& mem, int dir, int& levelOut) {
        ++calls;
        const int f0 = std::min(int(r), frames - 1), f1 = std::min(f0 + 1, frames - 1);
        const bool need1 = f1 != f0 && r > f0;
        const int i0 = find(f0, k), i1 = find(f1, k);
        if (ready(i0, t) && (!need1 || ready(i1, t))) {
            fresh = false;
            b[i0].used = t;
            if (need1)
                b[i1].used = t;
            const int nx = dir >= 0 ? f1 + (need1 ? 1 : 0) : f0 - 1;  // prefetch along the motion
            if (nx < frames)
                request(nx, k, t, words, mem, i0, need1 ? i1 : -1);
            levelOut = k;
            return r;
        }
        // missing: play the nearest loaded frames (any level), ask for the wanted ones
        int a = -1;
        double best = 1e300;
        for (int i = 0; i < nbuf; ++i)
            if (ready(i, t) && !fresh) {
                const double d = std::fabs(b[i].frame - r) + (b[i].level == k ? 0 : 0.5);
                if (d < best)
                    best = d, a = i;
            }
        int c = -1;  // its neighbour towards r, same level
        if (a >= 0) {
            const int nb = b[a].frame + (r > b[a].frame ? 1 : -1);
            c = find(nb, b[a].level);
            if (!ready(c, t))
                c = -1;
        }
        if (!ready(i0, t))
            request(f0, k, t, words, mem, a, c);
        if (need1 && !ready(i1, t))
            request(f1, k, t, words, mem, a, c);
        ++lagSamples;
        if (a < 0) {
            levelOut = -1;
            return r;
        }
        b[a].used = t;
        double p = b[a].frame;
        if (c >= 0) {
            b[c].used = t;
            p = std::clamp(r, double(std::min(b[a].frame, b[c].frame)), double(std::max(b[a].frame, b[c].frame)));
        }
        const double lag = std::fabs(r - p);
        lagSum += lag;
        maxLag = std::max(maxLag, lag);
        levelOut = b[a].level;
        return p;
    }
};

}  // namespace synth
