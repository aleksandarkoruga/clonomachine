// rtl/reverb_unit.v against model/reverb_model.cpp, sample by sample, with fw/reverb_ctl.c
// driving the registers every block as the soft core will. Input: impulses, noise bursts and
// silence; the parameters change on the way (sizes glide, wander, decays).
//   make reverb && ./obj_rv/Vreverb_unit [seconds]
#include <cstdio>
#include <cstdlib>

#include "Vreverb_unit.h"
#include "verilated.h"

#include "../model/reverb_model.h"

static Vreverb_unit* dut;
static uint64_t cycles;
static void tick() {
    dut->clk = 0;
    dut->eval();
    dut->clk = 1;
    dut->eval();
    ++cycles;
}

static void write_reg(int addr, uint32_t v) {
    dut->we = 1;
    dut->waddr = addr;
    dut->wdata = v;
    tick();
    dut->we = 0;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    const double seconds = argc > 1 ? atof(argv[1]) : 3.0;
    dut = new Vreverb_unit;
    dut->rst = 1;
    for (int i = 0; i < 4; ++i)
        tick();
    dut->rst = 0;
    static ReverbModel m;
    static rv_ctl ctl;
    rv_params p;
    rv_defaults(&p);
    p.rand1 = (int32_t)(0.25 * 65536);
    p.wander = (int32_t)(0.02 * 65536);
    rv_init(&ctl, 48000, 99, &p);
    const int n = (int)(seconds * 48000);
    uint32_t s = 12345;
    int bad = 0, max_busy = 0;
    for (int i = 0; i < n; ++i) {
        /* the parameters move now and then: glides, decay changes */
        if (i == n / 4) { p.time = (int32_t)(0.05 * 65536); p.glide = (int32_t)(0.5 * 65536); rv_set(&ctl, &p); }
        if (i == n / 2) { p.decay = (int32_t)(1.5 * 65536); p.hf_decay = (int32_t)(0.3 * 65536); p.glide_mode = 1; rv_set(&ctl, &p); }
        if (i == 3 * n / 4) { p.time = (int32_t)(0.6 * 65536); p.wander = (int32_t)(0.1 * 65536); rv_set(&ctl, &p); }
        if (i % 256 == 0) { /* the next block's registers, as the soft core writes them */
            rv_block(&ctl, &m.next);
            for (int r = 0; r < 16; ++r) {
                write_reg(r, m.next.target[r]);
                write_reg(16 + r, (uint32_t)m.next.gain[r]);
                write_reg(32 + r, (uint32_t)m.next.pole[r]);
            }
        }
        /* input: impulses, noise bursts, silence */
        int32_t x = 0;
        const int ph = i % 24000;
        if (ph == 0)
            x = 6000000;
        else if (ph > 4000 && ph < 9000) {
            s ^= s << 13, s ^= s >> 17, s ^= s << 5;
            x = (int32_t)(s >> 8) - (1 << 23);
            x >>= 1;
        }
        const int32_t want = m.step(x);
        dut->in = (uint32_t)x & 0xFFFFFF;
        dut->start = 1;
        tick();
        dut->start = 0;
        int busy = 1;
        while (dut->busy) {
            tick();
            ++busy;
            if (busy > 1024) {
                printf("stuck at sample %d\n", i);
                return 1;
            }
        }
        if (busy > max_busy)
            max_busy = busy;
        const int32_t got = ((int32_t)(dut->out << 8)) >> 8;
        if (got != want && bad++ < 10)
            printf("sample %d: rtl %d model %d\n", i, got, want);
    }
    printf("%d samples, %d mismatches, %d cycles per sample at most (budget 1024)\n", n, bad, max_busy);
    delete dut;
    return bad != 0;
}
