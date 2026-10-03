// Drive rtl/dsm_dac.v with a sine and dump the 4-bit codes for scripts/dsm_check.py.
#include "Vdsm_dac.h"
#include "verilated.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const int mode = argc > 1 ? atoi(argv[1]) : 0;
    const double amp = argc > 2 ? atof(argv[2]) : 0.5;
    const char* path = argc > 3 ? argv[3] : "dsm_out.bin";
    Vdsm_dac t;
    t.mode = mode;
    t.rst = 1;
    for (int i = 0; i < 8; ++i) { t.clk = 0; t.eval(); t.clk = 1; t.eval(); }
    t.rst = 0;
    FILE* f = fopen(path, "wb");
    const int n = 2048, period = 1024;
    const double fsig = 31.0 * 48000.0 / 1024.0;
    for (int s = 0; s < n; ++s) {
        t.sample = static_cast<int32_t>(std::lrint(amp * 8388607.0 * std::sin(2 * M_PI * fsig * s / 48000.0))) & 0xFFFFFF;
        for (int c = 0; c < period; ++c) {
            t.tick = (c == 0);
            t.clk = 0; t.eval(); t.clk = 1; t.eval();
            if ((c & 3) == 0 && s >= 1024) { unsigned char b = t.code; fwrite(&b, 1, 1, f); }
        }
    }
    fclose(f);
    return 0;
}
