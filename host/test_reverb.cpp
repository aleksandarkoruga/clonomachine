// The 2x2 plate: float reference (model/pmwave_ref) against the fixed-point model the FPGA will
// run (model/reverb_model + fw/reverb_ctl), same seed so the rails get the same random lengths.
// Prints how close they are and how the energy decays, checks that silence goes back to zero,
// and writes WAVs to build/reverb/ for listening (impulse responses; a loop if given).
//
//   gcc -O2 -c -Ifw fw/reverb_ctl.c -o build/host/reverb_ctl.o
//   g++ -O2 -std=c++17 -o build/host/test_reverb host/test_reverb.cpp model/pmwave_ref.cpp
//       model/reverb_model.cpp build/host/reverb_ctl.o
//   build/host/test_reverb [loop.wav] [decay hfDecay time rand1 wander]
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "../model/pmwave_ref.h"
#include "../model/reverb_model.h"

static const int FS = 48000;

static void write_wav(const std::string& path, const std::vector<float>& l, const std::vector<float>& r) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f)
        return;
    const uint32_t n = (uint32_t)l.size(), data = n * 4;
    auto u32 = [&](uint32_t v) { fwrite(&v, 4, 1, f); };
    auto u16 = [&](uint16_t v) { fwrite(&v, 2, 1, f); };
    fwrite("RIFF", 1, 4, f), u32(36 + data), fwrite("WAVEfmt ", 1, 8, f), u32(16), u16(1), u16(2), u32(FS),
        u32(FS * 4), u16(4), u16(16), fwrite("data", 1, 4, f), u32(data);
    for (uint32_t i = 0; i < n; ++i)
        for (float x : {l[i], r[i]}) {
            const float c = x > 1 ? 1 : x < -1 ? -1 : x;
            u16((uint16_t)(int16_t)lrintf(c * 32767));
        }
    fclose(f);
}

// PCM 16/24/32-bit or float32, any channels -> mono float (no resampling: expects 48 kHz)
static std::vector<float> read_wav(const char* path) {
    std::vector<float> out;
    FILE* f = fopen(path, "rb");
    if (!f)
        return out;
    std::vector<uint8_t> b;
    uint8_t buf[65536];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
        b.insert(b.end(), buf, buf + n);
    fclose(f);
    int fmt = 0, ch = 0, bits = 0;
    for (size_t p = 12; p + 8 <= b.size();) {
        uint32_t sz;
        memcpy(&sz, &b[p + 4], 4);
        if (!memcmp(&b[p], "fmt ", 4)) {
            fmt = b[p + 8] | b[p + 9] << 8;
            ch = b[p + 10] | b[p + 11] << 8;
            bits = b[p + 22] | b[p + 23] << 8;
            if (fmt == 0xFFFE)
                fmt = b[p + 32] | b[p + 33] << 8;
        } else if (!memcmp(&b[p], "data", 4) && ch && bits) {
            const int bps = bits / 8;
            const size_t frames = std::min<size_t>(sz, b.size() - p - 8) / (size_t)(bps * ch);
            for (size_t i = 0; i < frames; ++i) {
                double s = 0;
                for (int c = 0; c < ch; ++c) {
                    const uint8_t* q = &b[p + 8 + (i * ch + c) * bps];
                    if (fmt == 3 && bits == 32) {
                        float v;
                        memcpy(&v, q, 4);
                        s += v;
                    } else if (bits == 16)
                        s += (int16_t)(q[0] | q[1] << 8) / 32768.0;
                    else if (bits == 24)
                        s += ((int32_t)(q[0] << 8 | q[1] << 16 | q[2] << 24) >> 8) / 8388608.0;
                    else if (bits == 32)
                        s += (int32_t)(q[0] | q[1] << 8 | q[2] << 16 | (uint32_t)q[3] << 24) / 2147483648.0;
                }
                out.push_back((float)(s / ch));
            }
            break;
        }
        p += 8 + sz + (sz & 1);
    }
    return out;
}

struct Run {
    std::vector<float> ref, fix; // mono wet: (pickup L + pickup R) / 2
};

static Run run(const std::vector<float>& in, const pmw::Params& P, uint32_t seed) {
    Run r;
    pmw::Ref ref(FS, seed);
    ref.set(P);
    rv_params rp;
    rp.decay = (int32_t)lrint(P.decay * 65536), rp.time = (int32_t)lrint(P.time * 65536);
    rp.glide = (int32_t)lrint(P.glide * 65536), rp.hf_decay = (int32_t)lrint(P.hfDecay * 65536);
    rp.rand1 = (int32_t)lrint(P.rand1 * 65536), rp.rand2 = (int32_t)lrint(P.rand2 * 65536);
    rp.wander = (int32_t)lrint(P.wander * 65536), rp.glide_mode = P.glideMode;
    static rv_ctl ctl;
    rv_init(&ctl, FS, seed, &rp);
    static ReverbModel m;
    m = ReverbModel();
    for (int k = 0; k < ReverbModel::NR; ++k)
        m.len[k] = ctl.len[k]; /* start where the plate is (the unit glides in from reset) */
    r.ref.resize(in.size());
    r.fix.resize(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        float l, rr;
        ref.process(&in[i], &l, &rr, 1);
        r.ref[i] = 0.5f * (l + rr);
        if (i % 256 == 0)
            rv_block(&ctl, &m.next);
        const int32_t x = (int32_t)lrint(std::max(-1.99f, std::min(1.99f, in[i])) * 4194304);
        r.fix[i] = m.step(x) / 4194304.0f;
    }
    return r;
}

static double db(double e) { return e > 1e-30 ? 10 * std::log10(e) : -300; }

int main(int argc, char** argv) {
    pmw::Params P;
    P.decay = 4, P.hfDecay = 1, P.time = 0.3f, P.rand1 = 0.25f, P.wander = 0.0f, P.glide = 0.2f, P.rand2 = 0.2f;
    if (argc > 6) {
        P.decay = (float)atof(argv[2]), P.hfDecay = (float)atof(argv[3]), P.time = (float)atof(argv[4]);
        P.rand1 = (float)atof(argv[5]), P.wander = (float)atof(argv[6]);
    }
    system("mkdir -p build/reverb");
    printf("plate 2x2: decay %.2f s, hfDecay %.2f s, time %.2f, rand1 %.2f, wander %.3f\n", P.decay, P.hfDecay, P.time,
           P.rand1, P.wander);

    /* impulse response */
    std::vector<float> imp((size_t)(FS * (P.decay * 1.5 + 1)), 0.f);
    imp[0] = 1.0f;
    Run ir = run(imp, P, 7);
    double e_ref = 0, e_diff = 0;
    for (size_t i = 0; i < (size_t)FS * 2 && i < imp.size(); ++i) {
        e_ref += (double)ir.ref[i] * ir.ref[i];
        const double d = ir.fix[i] - ir.ref[i];
        e_diff += d * d;
    }
    printf("impulse response, first 2 s: fixed vs float difference %.1f dB below the signal\n", db(e_ref) - db(e_diff));
    printf("energy per 100 ms window (dB):   t      float    fixed\n");
    for (double t : {0.0, 0.1, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0}) {
        const size_t a = (size_t)(t * FS), b = a + FS / 10;
        if (b > imp.size())
            break;
        double er = 0, ef = 0;
        for (size_t i = a; i < b; ++i)
            er += (double)ir.ref[i] * ir.ref[i], ef += (double)ir.fix[i] * ir.fix[i];
        printf("                               %4.1f s  %7.1f  %7.1f\n", t, db(er), db(ef));
    }
    write_wav("build/reverb/ir_float.wav", ir.ref, ir.ref);
    write_wav("build/reverb/ir_fixed.wav", ir.fix, ir.fix);

    /* silence after a burst must decay to nothing (no limit cycles in the fixed loops) */
    std::vector<float> burst((size_t)FS * 30, 0.f);
    uint32_t s = 1;
    for (int i = 0; i < FS / 2; ++i) {
        s ^= s << 13, s ^= s >> 17, s ^= s << 5;
        burst[i] = ((s >> 8) / 16777216.0f - 0.5f) * 0.8f;
    }
    pmw::Params Q = P;
    Q.decay = 1.0f, Q.hfDecay = 0.5f;
    Run tail = run(burst, Q, 3);
    float peak = 0;
    for (size_t i = (size_t)FS * 25; i < burst.size(); ++i)
        peak = std::max(peak, std::fabs(tail.fix[i]));
    printf("after a burst, decay 1 s: fixed output peak in 25..30 s = %.1f LSB of the 18-bit rails (%s)\n",
           peak * 65536, peak * 65536 <= 4 ? "silent" : "LIMIT CYCLE");

    if (argc > 1) { /* a loop through the plate: dry left, wet right; and wet only */
        std::vector<float> loop = read_wav(argv[1]);
        if (loop.empty()) {
            printf("could not read %s\n", argv[1]);
            return 1;
        }
        const size_t tailn = (size_t)(FS * P.decay);
        std::vector<float> in(loop);
        in.resize(loop.size() * 2 + tailn, 0.f);
        std::copy(loop.begin(), loop.end(), in.begin() + (long)loop.size()); /* twice, then the tail */
        for (float& x : in)
            x *= 0.5f;
        Run lr = run(in, P, 11);
        std::vector<float> dry(in);
        for (float& x : dry)
            x *= 2;
        write_wav("build/reverb/loop_float.wav", dry, lr.ref);
        write_wav("build/reverb/loop_fixed.wav", dry, lr.fix);
        printf("loop: build/reverb/loop_{float,fixed}.wav (dry left, plate right)\n");
    }
    printf("impulse responses: build/reverb/ir_{float,fixed}.wav\n");
    return 0;
}
