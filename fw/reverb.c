/* The plate reverb from the soft core, see reverb.h. The knobs' 0..127 become the UGen's
 * parameters here; reverb_service() applies them (once per main-loop pass, however many CCs
 * came) and runs the control once per 256-sample block, writing the 48 rail registers. */
#include "reverb.h"

#include <stdio.h>

#include "../include/sampler_regs.h"
#include "dmath.h"
#include "engine.h"
#include "midimap.h"
#include "reverb_ctl.h"
#include "slot.h"

static struct rv_ctl g_rv;
static struct rv_params g_p;
static struct rv_regs g_regs;
static int g_knob[RK_NUM];
static int g_dirty, g_inited;
static uint32_t g_blk;
static int32_t g_last_gain[RV_RAILS], g_last_pole[RV_RAILS];

/* defaults: a medium plate, 4 s, a little damped and modulated, the return at 1 */
static const int k_default[RK_NUM] = {
    [RK_SIZE] = 89, [RK_DECAY] = 82, [RK_DAMP] = 25, [RK_DIFFUSION] = 32, [RK_WANDER] = 28,
    [RK_RETURN] = 85, [RK_GLIDE] = 73, [RK_SPREAD] = 13, [RK_GLIDEMODE] = 0,
};

static int32_t q16(double x) { return (int32_t)(x * 65536.0 + 0.5); }
static double pow_d(double b, double e) { return d_exp(e * d_log(b)); }

/* 0..127 -> the UGen's inputs */
static void params_from_knobs(struct rv_params* p) {
    const double size = g_knob[RK_SIZE] / 127.0, dec = g_knob[RK_DECAY] / 127.0, damp = g_knob[RK_DAMP] / 127.0;
    const double decay = 0.1 * pow_d(300.0, dec);                  /* 0.1 .. 30 s          */
    p->time = q16(1.0 - size);                                     /* 127: the biggest     */
    p->decay = q16(decay);
    p->hf_decay = q16(decay * pow_d(2.0, -10.0 * damp));           /* 0: no damping        */
    p->rand1 = q16(g_knob[RK_DIFFUSION] / 127.0);                  /* 0 .. 1               */
    const double wv = g_knob[RK_WANDER] / 127.0;
    p->wander = q16(0.1 * wv * wv);                                /* 0 .. 0.1             */
    p->glide = q16(0.001 * pow_d(10000.0, g_knob[RK_GLIDE] / 127.0)); /* 1 ms .. 10 s      */
    p->rand2 = q16(2.0 * g_knob[RK_SPREAD] / 127.0);               /* 0 .. 2               */
    p->glide_mode = g_knob[RK_GLIDEMODE] >= 64;
}

static void write_return(void) { /* u4.12, 0 .. 1.5 */
    eng_write(SMP_ADDR(SMP_GLOBAL_SLOT, SMP_G_REV_RETURN), (uint32_t)(g_knob[RK_RETURN] * 6144 / 127));
}

static void write_regs(int all) {
    for (int r = 0; r < RV_RAILS; ++r) {
        eng_write(SMP_ADDR(SMP_REV_PAGE0, r), g_regs.target[r]);
        if (all || g_regs.gain[r] != g_last_gain[r])
            eng_write(SMP_ADDR(SMP_REV_PAGE0, 16 + r), (uint32_t)g_regs.gain[r]);
        if (all || g_regs.pole[r] != g_last_pole[r])
            eng_write(SMP_ADDR(SMP_REV_PAGE1, r), (uint32_t)g_regs.pole[r]);
        g_last_gain[r] = g_regs.gain[r];
        g_last_pole[r] = g_regs.pole[r];
    }
}

void reverb_init(void) {
    for (int k = 0; k < RK_NUM; ++k)
        g_knob[k] = k_default[k];
    params_from_knobs(&g_p);
    rv_init(&g_rv, eng_fs_q16() >> 16, 0x5EED, &g_p);
    rv_block(&g_rv, &g_regs);
    write_regs(1);
    write_return();
    g_blk = eng_status(SMP_SG_SAMPLES) >> RV_BLOCK_LOG2;
    g_inited = 1;
}

void reverb_knob(int k, int v) {
    if (k <= RK_NONE || k >= RK_NUM)
        return;
    v = v < 0 ? 0 : v > 127 ? 127 : v;
    if (k >= RK_SEND0 && k <= RK_SEND2) { /* a voice's send, as its fxSend knob */
        const int s = k - RK_SEND0;
        if (s < NUM_SLOTS) {
            g_slots[s].fxsend_q16 = v * 65536 / 127;
            slot_apply_output(s);
        }
        return;
    }
    g_knob[k] = v;
    if (k == RK_RETURN)
        write_return();
    else
        g_dirty = 1;
}

int reverb_knob_value(int k) { return k > RK_NONE && k < RK_NUM ? g_knob[k] : 0; }

void reverb_service(void) {
    if (!g_inited)
        return;
    if (g_dirty) { /* however many CCs came, one recomputation */
        g_dirty = 0;
        params_from_knobs(&g_p);
        rv_set(&g_rv, &g_p);
    }
    const uint32_t blk = eng_status(SMP_SG_SAMPLES) >> RV_BLOCK_LOG2;
    uint32_t n = blk - g_blk;
    if (!n)
        return;
    if (n > 4) /* the loader held us up: catch up a little, the plate glides on */
        n = 4;
    while (n--)
        rv_block(&g_rv, &g_regs);
    write_regs(0);
    g_blk = blk;
}

static void print_q16(const char* name, int32_t v, const char* unit) {
    printf(" %s %lu.%03lu%s", name, (unsigned long)(v >> 16), (unsigned long)(((v & 0xFFFF) * 1000) >> 16), unit);
}

void reverb_status(void) {
    printf("rev:");
    for (int k = RK_SIZE; k <= RK_GLIDEMODE; ++k)
        printf(" %s %d", rk_names[k], g_knob[k]);
    printf("  sends");
    for (int s = 0; s < NUM_SLOTS; ++s)
        printf(" %ld", (long)(g_slots[s].fxsend_q16 * 127 / 65536));
    printf("\n    ");
    print_q16("time", g_p.time, "");
    print_q16("decay", g_p.decay, " s");
    print_q16("hfDecay", g_p.hf_decay, " s");
    print_q16("rand1", g_p.rand1, "");
    print_q16("wander", g_p.wander, "");
    print_q16("glide", g_p.glide, " s");
    print_q16("rand2", g_p.rand2, "");
    printf(" glideMode %d  rail lengths", g_p.glide_mode);
    for (int r = 0; r < RV_RAILS; r += 5)
        printf(" %lu", (unsigned long)(g_regs.target[r] >> 20));
    printf(" ...\n");
}
