// Host check of fw/transients.c: prints the onsets found in WAV files (mono mixdown,
// 16-bit like the SDRAM copy) as seconds and as fractions of the loop.
//
//   g++ -O2 -std=c++20 -Ifw -Imodel host/test_transients.cpp fw/transients.c -o test_transients
//   ./test_transients loop.wav [more.wav ...]
#include <cmath>
#include <cstdio>
#include <vector>

#include "wav.h"
extern "C" {
#include "transients.h"
}

static int16_t get(void* ctx, uint32_t i) { return (*static_cast<std::vector<int16_t>*>(ctx))[i]; }

int main(int argc, char** argv) {
    for (int a = 1; a < argc; ++a) {
        wav::Audio w;
        if (!wav::read(argv[a], w)) {
            std::printf("%s: cannot read\n", argv[a]);
            continue;
        }
        std::vector<int16_t> x(w.x.size());
        for (size_t i = 0; i < x.size(); ++i)
            x[i] = int16_t(std::lrint(std::fmax(-1.0, std::fmin(1.0, w.x[i])) * 32767));
        uint32_t pos[TR_MAX];
        const int n = tr_analyse(get, &x, uint32_t(x.size()), pos, nullptr);
        std::printf("%s: %.2f s, %d transients\n ", argv[a], double(x.size()) / w.rate, n);
        for (int k = 0; k < n; ++k)
            std::printf(" %.3f", double(pos[k]) / w.rate);
        std::printf("\n  frac:");
        for (int k = 0; k < n; ++k)
            std::printf(" %.3f", double(pos[k]) / x.size());
        std::printf("\n");
    }
    return 0;
}
