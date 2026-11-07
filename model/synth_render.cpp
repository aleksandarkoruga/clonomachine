// Renders a synth song (synth_patch.h format) through the reference and the hardware
// configuration of the synth model, writes both as WAVs, compares them and prints what the
// hardware version costs.
//
//   synth_render song.txt [-o out_prefix] [--only ref|hw] [hardware options]
//     --rate R           oscillator / filter rate, x 48 kHz (1 2 4 8; default 2)
//     --interp I         lin | herm | sinc8 (default sinc8)
//     --nmin N           smallest stored mip level (default 256)
//     --maxframe N       largest level played: the cache size (default 1024)
//     --ctrl N           modulation every N samples (default 16)
//     --float            no word-length emulation
//     --nocache          read tables directly (no frame cache / SDRAM model)
//     --naive-levels     levels from the note's pitch only (not the warps)
//     --sdram L          lines per sample left for the synth (default 6)
//     --tableA / --tableB  a built-in table name or a wavetable WAV, instead of the song's
//     --stereo           stereo voices (the board's synth output is mono)
//     --ref-float        compare against the same configuration in floating point (isolates the
//                        word lengths) instead of the 8x reference
//
// Build: model/build_synth.sh
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "synth_analysis.h"
#include "synth_engine.h"
#include "synth_patch.h"

using namespace synth;

static bool write_stereo(const std::string& path, const std::vector<float>& L, const std::vector<float>& R) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f)
        return false;
    const uint32_t bytes = uint32_t(L.size() * 8);
    auto w32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
    auto w16 = [&](uint16_t v) { std::fwrite(&v, 2, 1, f); };
    std::fwrite("RIFF", 1, 4, f);
    w32(36 + bytes);
    std::fwrite("WAVEfmt ", 1, 8, f);
    w32(16);
    w16(3);
    w16(2);
    w32(48000);
    w32(48000 * 8);
    w16(8);
    w16(32);
    std::fwrite("data", 1, 4, f);
    w32(bytes);
    for (size_t i = 0; i < L.size(); ++i) {
        std::fwrite(&L[i], 4, 1, f);
        std::fwrite(&R[i], 4, 1, f);
    }
    std::fclose(f);
    return true;
}

static void report(const char* name, const Stats& s, const Config& c) {
    std::printf("%s: %.2f s, peak %.2f dBFS%s\n", name, s.samples / 48000.0, 20 * std::log10(s.peak + 1e-12),
                s.clips ? (" (" + std::to_string(s.clips) + " samples clipped)").c_str() : "");
    std::printf("  multiplies per output sample: %.0f  (", s.macsPerSample);
    for (auto& [k, v] : s.macs)
        std::printf(" %s %.0f", k.c_str(), double(v) / std::max<uint64_t>(1, s.samples));
    std::printf(" )\n  table reads per sample %.0f, function evaluations (tanh, divide, ...) per sample %.0f\n",
                s.tableReadsPerSample, s.fnsPerSample);
    if (c.cache)
        std::printf("  SDRAM lines per sample: mean %.2f, peak %.2f (64-sample window); frame loads %llu (%.0f/s)\n"
                    "  position not at the wanted frames: %.2f %% of oscillator samples, mean error %.3f frames, max %.1f\n",
                    s.sdramMean, s.sdramPeak, (unsigned long long)s.frameLoads, s.frameLoads / (s.samples / 48000.0),
                    100.0 * s.lagSamples / std::max<uint64_t>(1, s.oscSamples), s.lagSum / std::max<uint64_t>(1, s.oscSamples),
                    s.maxLagFrames);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s song.txt [options] (see the source)\n", argv[0]);
        return 1;
    }
    std::string songPath = argv[1], out, only, tabA, tabB;
    bool refFloat = false;
    Config hw;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "-o") out = next();
        else if (a == "--only") only = next();
        else if (a == "--rate") hw.rate = std::atoi(next().c_str());
        else if (a == "--interp") {
            const std::string v = next();
            hw.interp = v == "lin" ? Interp::LIN : v == "herm" ? Interp::HERM : Interp::SINC8;
        } else if (a == "--nmin") hw.nmin = std::atoi(next().c_str());
        else if (a == "--maxframe") hw.maxFrame = std::atoi(next().c_str());
        else if (a == "--ctrl") hw.ctrlDiv = std::atoi(next().c_str());
        else if (a == "--float") hw.fixed = false;
        else if (a == "--nocache") hw.cache = false;
        else if (a == "--naive-levels") hw.warpAwareLevels = false;
        else if (a == "--sdram") hw.sdramLines = std::atof(next().c_str());
        else if (a == "--tableA") tabA = next();
        else if (a == "--ref-float") refFloat = true;
        else if (a == "--stereo") hw.stereo = true;
        else if (a == "--tableB") tabB = next();
        else {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            return 1;
        }
    }
    if (out.empty()) {
        out = songPath.substr(songPath.find_last_of('/') + 1);
        out = "build/synth/" + out.substr(0, out.find_last_of('.'));
    }
    Song song;
    std::string err;
    if (!load_song(songPath, song, err)) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }
    if (!tabA.empty())
        song.tables["tableA"] = tabA;
    if (!tabB.empty())
        song.tables["tableB"] = tabB;
    std::vector<float> rl, rr, hl, hr;
    Config ref = Config::reference();
    if (refFloat) {  // the hardware configuration without word lengths: what the fixed point costs
        ref = hw;
        ref.fixed = false;
    }
    if (only != "hw") {
        Engine e(ref, song);
        e.render(rl, rr);
        write_stereo(out + "_ref.wav", rl, rr);
        report((out + "_ref.wav").c_str(), e.stats(), ref);
    }
    if (only != "ref") {
        Engine e(hw, song);
        e.render(hl, hr);
        write_stereo(out + "_hw.wav", hl, hr);
        report((out + "_hw.wav").c_str(), e.stats(), hw);
    }
    if (!rl.empty() && !hl.empty()) {
        std::vector<float> rm(rl.size()), hm(hl.size());
        for (size_t i = 0; i < rl.size(); ++i)
            rm[i] = 0.5f * (rl[i] + rr[i]);
        for (size_t i = 0; i < hl.size(); ++i)
            hm[i] = 0.5f * (hl[i] + hr[i]);
        double lag = 0;
        const double d = snr_frac(hm, rm, &lag);
        std::printf("hardware vs reference: SNR %.1f dB up to 18 kHz (hardware %.2f samples later) -- with feedback or noise "
                    "this only says how far they drift, listen to both\n", d, lag);
    }
    return 0;
}
