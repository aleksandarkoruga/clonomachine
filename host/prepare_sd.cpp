// Prepares a sample card on the PC so the board loads fast and has nothing to analyse:
// every WAV in /drums /melody /bass /kick (loose files and one level of subfolders, as the
// firmware indexes them) becomes mono 16-bit PCM at its own sample rate, normalised; then
// each folder gets the firmware's caches, computed with the firmware's own code:
//   .smpidx  format / length / peak per file              (fw/samples.c, version 2)
//   .smptr   peak + transients per sample and per part     (fw/transients.c, fw/wavutil.c split)
// Drum loops whose transients don't fit SC's length-based tempo (the loop is 2^k
// beats) get a tempo for a whole number of beats that fits them (3-bar loops, loops with a
// tail beat), stored in the index; everything else keeps the length-based tempo.
// The records carry date/time 0: the board matches them by name and size and rewrites them
// with the card's real timestamps.
//
//   g++ -O2 -std=c++17 -Ifw -Iinclude host/prepare_sd.cpp fw/wavutil.c fw/transients.c -o prepare_sd
//   ./prepare_sd /run/media/$USER/CARD            # report only
//   ./prepare_sd /run/media/$USER/CARD --write    # convert in place + write the caches
//   ./prepare_sd CARD --csv table.csv             # also: one line per sample (length, bpm, transients)
//
// Files that are already mono 16-bit with full-scale peaks are left as they are. Formats the
// firmware can't read (e.g. ADPCM) are left alone and get no records.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

extern "C" {
#include "transients.h"
#include "wavutil.h"
}
#include "../include/sampler_regs.h"
#include "../fw/samples.h"

namespace fs = std::filesystem;

static const uint32_t kMaxFrames = BANK_MAX_BYTES / 2 - SMP_PREPAD - SMP_POSTPAD - 64; // = SAMPLE_MAX_FRAMES
static const char* const kLibs[] = {"drums", "melody", "bass", "kick"};

static uint32_t rd32(const uint8_t* p) { return p[0] | p[1] << 8 | p[2] << 16 | uint32_t(p[3]) << 24; }
static uint16_t rd16(const uint8_t* p) { return uint16_t(p[0] | p[1] << 8); }

struct Wav {
    int fmt = 0, ch = 0, bits = 0;
    uint32_t sr = 0, data_off = 0, data_len = 0, frames = 0;
};

// same rules as parse_wav() in fw/samples.c
static bool parse_wav(const fs::path& p, Wav& w, uint64_t fsize) {
    FILE* f = std::fopen(p.c_str(), "rb");
    if (!f)
        return false;
    uint8_t h[12];
    bool ok = std::fread(h, 1, 12, f) == 12 && !std::memcmp(h, "RIFF", 4) && !std::memcmp(h + 8, "WAVE", 4);
    while (ok) {
        uint8_t c[8];
        if (std::fread(c, 1, 8, f) != 8) {
            ok = false;
            break;
        }
        const uint32_t len = rd32(c + 4);
        if (!std::memcmp(c, "fmt ", 4)) {
            uint8_t b[40] = {};
            const uint32_t want = len < 40 ? len : 40;
            if (std::fread(b, 1, want, f) != want) {
                ok = false;
                break;
            }
            w.fmt = rd16(b);
            w.ch = rd16(b + 2);
            w.sr = rd32(b + 4);
            w.bits = rd16(b + 14);
            if (w.fmt == 0xFFFE && len >= 26)
                w.fmt = rd16(b + 24);
            std::fseek(f, long(len - want + (len & 1)), SEEK_CUR);
        } else if (!std::memcmp(c, "data", 4)) {
            w.data_off = uint32_t(std::ftell(f));
            w.data_len = len;
            break;
        } else
            std::fseek(f, long(len + (len & 1)), SEEK_CUR);
    }
    std::fclose(f);
    if (!ok || !wav_supported(w.fmt, w.ch, w.bits) || !w.sr)
        return false;
    if (uint64_t(w.data_off) + w.data_len > fsize)
        w.data_len = uint32_t(fsize - w.data_off);
    w.frames = w.data_len / uint32_t(w.ch * (w.bits / 8));
    return w.frames >= 128;
}

// one channel sample as a double in -1..1 (floats as they are)
static double sample_at(const uint8_t* p, int fmt, int bits) {
    if (fmt == WAV_FLOAT) {
        if (bits == 32) {
            float v;
            std::memcpy(&v, p, 4);
            return std::isfinite(v) ? v : 0.0;
        }
        double v;
        std::memcpy(&v, p, 8);
        return std::isfinite(v) ? v : 0.0;
    }
    switch (bits) {
    case 8: return (p[0] - 128) / 128.0;
    case 16: return int16_t(rd16(p)) / 32768.0;
    case 24: return int32_t(uint32_t(p[0]) << 8 | uint32_t(p[1]) << 16 | uint32_t(p[2]) << 24) / 2147483648.0;
    default: return int32_t(rd32(p)) / 2147483648.0;
    }
}

// mono 16-bit, normalised to a 32767 peak (mean of the channels, as the firmware mixes down)
static bool convert(const fs::path& p, const Wav& w, std::vector<int16_t>& out) {
    FILE* f = std::fopen(p.c_str(), "rb");
    if (!f)
        return false;
    const int bps = w.bits / 8, fb = bps * w.ch;
    std::vector<uint8_t> buf(size_t(fb) * 65536);
    std::vector<float> mono(w.frames);
    std::fseek(f, long(w.data_off), SEEK_SET);
    double peak = 0;
    for (uint32_t i = 0; i < w.frames;) {
        const uint32_t n = std::min<uint32_t>(65536, w.frames - i);
        if (std::fread(buf.data(), size_t(fb), n, f) != n) {
            std::fclose(f);
            return false;
        }
        for (uint32_t k = 0; k < n; ++k) {
            double acc = 0;
            for (int c = 0; c < w.ch; ++c)
                acc += sample_at(&buf[size_t(k) * fb + size_t(c) * bps], w.fmt, w.bits);
            const double m = acc / w.ch;
            mono[i + k] = float(m);
            peak = std::max(peak, std::fabs(m));
        }
        i += n;
    }
    std::fclose(f);
    const double g = peak > 0 ? 32767.0 / peak : 1.0;
    out.resize(w.frames);
    for (uint32_t i = 0; i < w.frames; ++i)
        out[i] = int16_t(std::lrint(std::clamp(mono[i] * g, -32768.0, 32767.0)));
    return true;
}

static bool write_wav(const fs::path& p, uint32_t sr, const std::vector<int16_t>& x) {
    const fs::path tmp = p.string() + ".prep_tmp";
    FILE* f = std::fopen(tmp.c_str(), "wb");
    if (!f)
        return false;
    const uint32_t data = uint32_t(x.size() * 2);
    uint8_t h[44];
    auto put32 = [&](int o, uint32_t v) { h[o] = uint8_t(v); h[o + 1] = uint8_t(v >> 8); h[o + 2] = uint8_t(v >> 16); h[o + 3] = uint8_t(v >> 24); };
    auto put16 = [&](int o, uint16_t v) { h[o] = uint8_t(v); h[o + 1] = uint8_t(v >> 8); };
    std::memcpy(h, "RIFF", 4); put32(4, 36 + data); std::memcpy(h + 8, "WAVEfmt ", 8); put32(16, 16);
    put16(20, 1); put16(22, 1); put32(24, sr); put32(28, sr * 2); put16(32, 2); put16(34, 16);
    std::memcpy(h + 36, "data", 4); put32(40, data);
    const bool ok = std::fwrite(h, 1, 44, f) == 44 && std::fwrite(x.data(), 2, x.size(), f) == x.size();
    if (std::fclose(f) != 0 || !ok) {
        fs::remove(tmp);
        return false;
    }
    fs::rename(tmp, p);
    return true;
}

// ---- the firmware's cache formats --------------------------------------------------------
#pragma pack(push, 1)
struct IdxHdr { uint32_t magic, version, count, rec_size; };
struct IdxRec {
    char name[SAMPLE_NAME_LEN];
    uint32_t fsize;
    uint16_t fdate, ftime;
    uint32_t data_off, data_len, sr, frames;
    uint8_t ch, bits;
    int16_t peak;
    uint8_t fmt, pad;
    uint16_t bpm_x100;
};
struct TrfHdr { uint32_t magic, version, count, rec_size; };
struct TrfRec { uint32_t fsize; uint16_t fdate, ftime; uint32_t hash; uint8_t n, pad; int16_t peak; };
#pragma pack(pop)
static_assert(sizeof(IdxRec) == 112, "idx record layout");
static_assert(sizeof(TrfRec) == 16, "trf record layout");

static uint32_t fnv(uint32_t h, const uint8_t* p, size_t n) {
    while (n--)
        h = (h ^ *p++) * 16777619u;
    return h;
}

static uint32_t sample_key(const std::string& name, uint32_t data_off, uint32_t frames, uint32_t nseg) {
    uint32_t h = fnv(2166136261u, reinterpret_cast<const uint8_t*>(name.data()), name.size());
    if (nseg > 1) {
        const uint32_t w[3] = {data_off, frames, nseg};
        h = fnv(h, reinterpret_cast<const uint8_t*>(w), sizeof(w));
    }
    return h;
}

static bool is_wav(const fs::path& p) {
    std::string e = p.extension().string();
    std::transform(e.begin(), e.end(), e.begin(), ::tolower);
    return e == ".wav";
}

// loose files first, then subfolders; by name (fw/samples.c name_order)
static bool name_order(const std::string& a, const std::string& b) {
    const bool sa = a.find('/') != std::string::npos, sb = b.find('/') != std::string::npos;
    return sa != sb ? sb : a < b;
}

// copy of samples_track_bpm_q16() (fw/samples.c, as in SC): bpm = 60/length folded into [100, 200]
static uint32_t track_bpm_q16(uint32_t frames, uint32_t sr, int32_t* beats_log2) {
    int32_t k = 0;
    uint64_t bpm = 0;
    for (int it = 0; it < 1000; ++it) {
        const uint64_t num = (uint64_t)60 * sr << 16;
        if (k >= 0)
            bpm = (k < 24) ? (num << k) / frames : 0xFFFFFFFFull;
        else
            bpm = (num / frames) >> (-k);
        if (bpm < (100ull << 16))
            ++k;
        else if (bpm > (200ull << 16))
            --k;
        else
            break;
    }
    if (beats_log2)
        *beats_log2 = k;
    return (uint32_t)bpm;
}

// ---- tempo for loops that are not 2^k beats long -------------------------------------
static double fold_bpm(double b) {
    while (b < 100)
        b *= 2;
    while (b > 200)
        b /= 2;
    return b;
}

// fraction of transients within 12 % of a 16th of the best-phased grid at bpm (chance: 0.24)
static double grid_fit(const uint32_t* pos, int n, uint32_t sr, double bpm) {
    const double d = sr * 60.0 / bpm / 4, tau = 2 * M_PI;
    double c = 0, s = 0;
    for (int i = 0; i < n; ++i) {
        const double x = std::fmod(pos[i] / d, 1.0);
        c += std::cos(tau * x);
        s += std::sin(tau * x);
    }
    const double phi = std::fmod(std::atan2(s, c) / tau + 1.0, 1.0);
    int hits = 0;
    for (int i = 0; i < n; ++i) {
        const double x = std::fmod(pos[i] / d, 1.0), e = std::fabs(x - phi);
        hits += std::min(e, 1.0 - e) < 0.12;
    }
    return double(hits) / n;
}

struct LoopTempo { double fit_len = 0, fit_new = 0, bpm = 0; int beats = 0; uint16_t bpm_x100 = 0; bool maybe = false; };

// A whole number of beats with a real tempo (65..210 bpm before folding into 100..200, as
// the tempo folds), scored on the transients; powers of two and the tempo in the name win
// close calls. Used only when SC's length-based tempo fits badly (< 60 %) and the new one
// fits clearly better (+25 points) and well (>= 80 %); between 70 and 80 % (swing makes any
// 16th grid fit worse) it is only reported as a suggestion.
static LoopTempo loop_tempo(const std::string& name, uint32_t frames, uint32_t sr, const uint32_t* pos, int n) {
    LoopTempo t;
    int32_t k = 0;
    t.fit_len = n > 0 ? grid_fit(pos, n, sr, track_bpm_q16(frames, sr, &k) / 65536.0) : 0;
    if (n < 8 || t.fit_len >= 0.6)
        return t;
    const double secs = double(frames) / sr, nb = wav_name_bpm_milli(name.c_str()) / 1000.0;
    double best = -1;
    for (int B = 1; B <= 128; ++B) {
        const double raw = 60.0 * B / secs;
        if (raw < 65 || raw > 210)
            continue;
        const double bpm = fold_bpm(raw), fit = grid_fit(pos, n, sr, bpm);
        const double v = fit + ((B & (B - 1)) == 0 ? 0.03 : 0) + (nb > 0 && std::fabs(bpm / fold_bpm(nb) - 1) < 0.01 ? 0.05 : 0);
        if (v > best) {
            best = v;
            t.fit_new = fit;
            t.bpm = bpm;
            t.beats = B;
        }
    }
    t.beats = int(std::lrint(secs * t.bpm / 60)); /* at the folded tempo */
    if (t.fit_new - t.fit_len >= 0.25 && t.fit_new >= 0.8)
        t.bpm_x100 = uint16_t(std::lrint(t.bpm * 100));
    else if (t.fit_new - t.fit_len >= 0.25 && t.fit_new >= 0.7)
        t.maybe = true;
    return t;
}

static int16_t get16(void* ctx, uint32_t i) { return (*static_cast<std::vector<int16_t>*>(ctx))[i]; }

struct Totals { uint64_t before = 0, after = 0; int files = 0, converted = 0, samples = 0, skipped = 0, tempos = 0; };
static FILE* g_csv;

static void do_lib(const fs::path& dir, bool write, Totals& tot) {
    std::vector<std::string> names;
    for (const auto& e : fs::directory_iterator(dir)) {
        const std::string n = e.path().filename().string();
        if (n.empty() || n[0] == '.')
            continue;
        if (e.is_regular_file() && is_wav(e.path()))
            names.push_back(n);
        else if (e.is_directory() && n.size() < 32)
            for (const auto& s : fs::directory_iterator(e.path())) {
                const std::string m = s.path().filename().string();
                if (!m.empty() && m[0] != '.' && s.is_regular_file() && is_wav(s.path()))
                    names.push_back(n + "/" + m);
            }
    }
    names.erase(std::remove_if(names.begin(), names.end(), [](const std::string& n) { return n.size() >= SAMPLE_NAME_LEN; }),
                names.end());
    std::sort(names.begin(), names.end(), name_order);

    std::vector<IdxRec> idx;
    std::vector<uint8_t> trf; // records + positions
    uint32_t ntrf = 0;
    for (const std::string& name : names) {
        const fs::path p = dir / name;
        const uint64_t fsize0 = fs::file_size(p);
        Wav w;
        if (!parse_wav(p, w, fsize0)) {
            std::printf("  skip %s (not a supported WAV)\n", name.c_str());
            ++tot.skipped;
            continue;
        }
        ++tot.files;
        tot.before += fsize0;
        std::vector<int16_t> x;
        if (!convert(p, w, x)) {
            std::printf("  skip %s (read error)\n", name.c_str());
            ++tot.skipped;
            continue;
        }
        int16_t peak = 0;
        for (int16_t v : x)
            peak = std::max<int16_t>(peak, v == -32768 ? 32767 : int16_t(std::abs(v)));
        const bool native = w.fmt == WAV_PCM && w.ch == 1 && w.bits == 16 && w.data_off == 44 &&
                            w.data_len == w.frames * 2;
        bool rewrite = !native;
        if (native) { /* already mono 16-bit: rewrite only if it isn't normalised */
            Wav w2 = w;
            std::vector<int16_t> orig(w.frames);
            FILE* f = std::fopen(p.c_str(), "rb");
            std::fseek(f, 44, SEEK_SET);
            const size_t got = std::fread(orig.data(), 2, orig.size(), f);
            std::fclose(f);
            int16_t pk0 = 0;
            for (size_t i = 0; i < got; ++i)
                pk0 = std::max<int16_t>(pk0, orig[i] == -32768 ? 32767 : int16_t(std::abs(orig[i])));
            rewrite = pk0 < 32767;
            if (!rewrite)
                x.swap(orig);
            (void)w2;
        }
        if (rewrite) {
            ++tot.converted;
            if (write && !write_wav(p, w.sr, x)) {
                std::printf("  FAILED to write %s\n", name.c_str());
                continue;
            }
        }
        /* a file left as it is keeps its size (some carry chunks after the audio data) */
        const uint32_t F = w.frames, fsize = rewrite ? 44 + F * 2 : uint32_t(fsize0);
        tot.after += fsize;
        const uint32_t len = wav_split_len(F, w.sr, name.c_str(), kMaxFrames);
        const uint32_t parts = len ? wav_split_count(F, len) : 1;

        IdxRec r{};
        std::snprintf(r.name, sizeof(r.name), "%s", name.c_str());
        r.fsize = fsize;
        r.data_off = 44;
        r.data_len = F * 2;
        r.sr = w.sr;
        r.frames = F;
        r.ch = 1;
        r.bits = 16;
        r.peak = parts > 1 ? 0 : peak;
        r.fmt = WAV_PCM;
        idx.push_back(r);

        std::string tempo_note;
        for (uint32_t k = 0; k < parts; ++k) {
            const uint32_t start = len ? wav_split_start(F, len, k) : 0, n = len ? len : F;
            std::vector<int16_t> part(x.begin() + start, x.begin() + start + n);
            int16_t pp = 0;
            for (int16_t v : part)
                pp = std::max<int16_t>(pp, v == -32768 ? 32767 : int16_t(std::abs(v)));
            if (pp > 0 && pp < 32767) { /* the board's load-time gain (fw/samples.c job_start + wav_convert) */
                const int32_t g = int32_t((32767u << 16) / uint32_t(pp));
                std::vector<int16_t> scaled(n);
                wav_convert(scaled.data(), reinterpret_cast<const uint8_t*>(part.data()), n, WAV_PCM, 1, 16, g);
                part.swap(scaled);
            }
            uint32_t pos[TR_MAX];
            const int nt = tr_analyse(get16, &part, n, pos, nullptr);
            if (parts == 1 && dir.filename() == "drums") {
                const LoopTempo lt = loop_tempo(name, n, w.sr, pos, nt);
                idx.back().bpm_x100 = lt.bpm_x100;
                if (lt.bpm_x100 || lt.maybe) {
                    int32_t bl = 0;
                    tempo_note = std::string(lt.maybe ? "  maybe (not applied): " : "  tempo ") +
                                 std::to_string(track_bpm_q16(n, w.sr, &bl) >> 16) + " -> " +
                                 std::to_string(lt.bpm).substr(0, 6) + " bpm (" + std::to_string(lt.beats) + " beats, fit " +
                                 std::to_string(int(lt.fit_len * 100)) + "% -> " + std::to_string(int(lt.fit_new * 100)) + "%)";
                    tot.tempos += lt.bpm_x100 != 0;
                }
            }
            if (g_csv) {
                int32_t bl = 0;
                const uint32_t bpm = track_bpm_q16(n, w.sr, &bl);
                std::fprintf(g_csv, "%s,%s,%u,%u,%.3f,%u,%d,%.2f,%d", dir.filename().c_str(), name.c_str(), k, parts,
                             double(n) / w.sr, w.sr, int(bl), bpm / 65536.0, nt);
                for (int q = 0; q < nt; ++q)
                    std::fprintf(g_csv, ",%u", pos[q]);
                std::fprintf(g_csv, "\n");
            }
            TrfRec t{};
            t.fsize = fsize;
            t.hash = sample_key(name, 44 + start * 2, n, parts > 1 ? parts : 1);
            t.n = uint8_t(nt < 0 ? 0 : nt);
            t.peak = pp ? pp : 1;
            const uint8_t* tp = reinterpret_cast<const uint8_t*>(&t);
            trf.insert(trf.end(), tp, tp + sizeof(t));
            const uint8_t* pp8 = reinterpret_cast<const uint8_t*>(pos);
            trf.insert(trf.end(), pp8, pp8 + 4 * t.n);
            ++ntrf;
            ++tot.samples;
        }
        std::printf("  %-50s %s %5.1f s %s%s%s\n", name.c_str(), rewrite ? (native ? "normalise" : "convert  ") : "ok       ",
                    double(F) / w.sr, parts > 1 ? (std::to_string(parts) + " parts").c_str() : "",
                    rewrite ? (" (" + std::to_string(fsize0 / 1024) + " -> " + std::to_string(fsize / 1024) + " KB)").c_str() : "",
                    tempo_note.c_str());
    }
    if (!write)
        return;
    {
        FILE* f = std::fopen((dir / ".smpidx").c_str(), "wb");
        const IdxHdr h{0x58444953u, 2, uint32_t(idx.size()), sizeof(IdxRec)};
        std::fwrite(&h, sizeof(h), 1, f);
        std::fwrite(idx.data(), sizeof(IdxRec), idx.size(), f);
        std::fclose(f);
    }
    {
        FILE* f = std::fopen((dir / ".smptr").c_str(), "wb");
        const TrfHdr h{0x4E525453u, 1, ntrf, sizeof(TrfRec)};
        std::fwrite(&h, sizeof(h), 1, f);
        std::fwrite(trf.data(), 1, trf.size(), f);
        std::fclose(f);
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: %s <card mount point> [--write]\n", argv[0]);
        return 1;
    }
    const fs::path root = argv[1];
    bool write = false;
    for (int i = 2; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--write"))
            write = true;
        else if (!std::strcmp(argv[i], "--csv") && i + 1 < argc)
            g_csv = std::fopen(argv[++i], "w");
    }
    Totals tot;
    for (const char* lib : kLibs) {
        const fs::path d = root / lib;
        if (!fs::is_directory(d))
            continue;
        std::printf("/%s\n", lib);
        do_lib(d, write, tot);
    }
    if (g_csv)
        std::fclose(g_csv);
    std::printf("\n%d WAVs (%d to convert or normalise, %d skipped) -> %d samples analysed, %d drum loops get a tempo "
                "for a whole number of beats; %.0f MB -> %.0f MB%s\n",
                tot.files, tot.converted, tot.skipped, tot.samples, tot.tempos, tot.before / 1048576.0, tot.after / 1048576.0,
                write ? "" : "   (report only: run again with --write)");
    return 0;
}
