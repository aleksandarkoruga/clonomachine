// The break cutter (esp/cutter/cutter.c) as a shared library for host/cutter_live.py (ctypes).
//   gcc -O2 -shared -fPIC -Iesp/cutter host/cutter_lib.c esp/cutter/cutter.c -lm -o build/host/libcutter.so
#include <stdlib.h>
#include <string.h>

#include "cutter.h"

void* cutl_new(unsigned seed) {
    struct cutter* c = malloc(sizeof(*c));
    if (c)
        cut_init(c, seed);
    return c;
}
void cutl_rx(void* c, double t, const unsigned char* ev) {
    struct smp_event e;
    memcpy(&e, ev, 8);
    cut_clock(c, t);
    cut_rx(c, &e);
}
/* the next due frame's events into out (8 bytes each), returns how many, 0 if none is due */
int cutl_pop(void* c, double t, unsigned char* out) {
    struct cut_frame f;
    if (!cut_pop(c, t, &f))
        return 0;
    memcpy(out, f.ev, (size_t)f.n * 8);
    return f.n;
}
void cutl_preset(void* c, int n) { cut_preset(c, n); }
int cutl_npresets(void) { return cut_npresets(); }
const char* cutl_preset_name(int n) { return cut_preset_name(n); }
void cutl_knob(void* c, int k, float v) { cut_set_knob(c, k, v); }
float cutl_get_knob(void* c, int k) { return ((struct cutter*)c)->p.knob[k]; }
void cutl_conductor(void* c, int on) { cut_set_conductor(c, on); }
int cutl_state(void* c, int* out) { /* owner playing n16 conductor cond_state preset step */
    const struct cutter* p = c;
    out[0] = p->owner;
    out[1] = p->playing;
    out[2] = p->n16;
    out[3] = p->conductor;
    out[4] = p->cond_state;
    out[5] = p->preset;
    out[6] = (int)p->step;
    return 7;
}
int cutl_moves(void* c, int* out) {
    memcpy(out, ((struct cutter*)c)->stats_hits, sizeof(int) * CUT_NMOVES);
    return CUT_NMOVES;
}
const char* cutl_move_name(int m) { return cut_move_name(m); }
