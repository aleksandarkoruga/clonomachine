// Offline run of the break cutter (esp/cutter/cutter.c): feeds it the FPGA's messages for a
// few phrases and prints what it would send, or a move histogram.
//
//   gcc -O2 -Wall -Iesp/cutter host/cutter_sim.c esp/cutter/cutter.c -lm -o build/host/cutter_sim
//   build/host/cutter_sim [preset] [phrases] [bpm] [-q]
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cutter.h"

static const char* pname(int id) {
    static const char* names[] = {
#define N(id, name, frac) #name,
        SMP_PARAMS(N)
#undef N
    };
    return id >= 0 && id < P_NUM_PARAMS ? names[id] : "?";
}

int main(int argc, char** argv) {
    const int preset = argc > 1 ? atoi(argv[1]) : 1;
    const int phrases = argc > 2 ? atoi(argv[2]) : 2;
    const double bpm = argc > 3 ? atof(argv[3]) : 173.5;
    const int quiet = argc > 4 && !strcmp(argv[4], "-q");
    static struct cutter c;
    cut_init(&c, 12345);
    cut_preset(&c, preset);
    const struct smp_event start = {EV_START, 0, 0, 0, 0}, b = {EV_BPM, 0, 0, 0, (int32_t)(bpm * 1000)},
                           own = {EV_OWNER, 0, 0, 1, 16};
    cut_rx(&c, &start);
    cut_rx(&c, &b);
    cut_rx(&c, &own);
    const double s32 = 60.0 / bpm / 8;
    const int steps = phrases * 128;
    struct cut_frame f;
    int frames = 0, events = 0;
    printf("preset %s, %d phrases at %.1f bpm\n", cut_preset_name(preset), phrases, bpm);
    for (int s = 0; s < steps; ++s) {
        const double t = s * s32;
        while (cut_pop(&c, t, &f)) { /* due before this tick */
            ++frames;
            events += f.n;
            if (!quiet) {
                printf("%8.3f s  step %4d |", f.due, s);
                for (int i = 0; i < f.n; ++i)
                    if (f.ev[i].b == P_POS)
                        printf(" pos %.3f", f.ev[i].value < 0 ? -1.0 : f.ev[i].value / 1073741824.0);
                    else if (f.ev[i].b == P_TRIG || f.ev[i].b == P_GATEOFF)
                        printf(" %s", pname(f.ev[i].b));
                    else if (f.ev[i].b == P_BUFNUM || f.ev[i].b == P_REPEATS)
                        printf(" %s %d", pname(f.ev[i].b), f.ev[i].value);
                    else if (f.ev[i].b != P_ENVHOLD && f.ev[i].b != P_BGADVANCE)
                        printf(" %s %.3f", pname(f.ev[i].b), f.ev[i].value / 65536.0);
                printf("\n");
            }
        }
        cut_clock(&c, t);
        const struct smp_event tick = {EV_TICK, 1, 1, 0, s};
        cut_rx(&c, &tick);
    }
    printf("%d frames, %d events; moves:", frames, events);
    for (int m = 0; m < CUT_NMOVES; ++m)
        if (c.stats_hits[m])
            printf(" %s %d", cut_move_name(m), c.stats_hits[m]);
    printf("\n");
    return 0;
}
