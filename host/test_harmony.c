/* The harmony generator (esp/harmony/harmony.c) on the PC: the function chain keeps the rules,
 * phrases end in cadences, the voicing avoids parallels where it can, and the note events from
 * the ticks are balanced. Prints a few phrases.
 *
 *   gcc -O2 -Wall -Iinclude host/test_harmony.c esp/harmony/harmony.c fw/scales.c -o test_harmony && ./test_harmony
 */
#include <stdio.h>
#include <string.h>

#include "../esp/harmony/harmony.h"
#include "../include/synth_params.h"

static int fails;
static void check(int ok, const char* what) {
    printf("  %-60s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok)
        ++fails;
}

static void param(struct harmony* h, int p, double v) {
    struct smp_event e = {EV_SYN_PARAM, 0, (uint8_t)p, 0, (int32_t)(v * 65536)};
    harm_rx(h, &e);
}

int main(void) {
    static struct harmony h;
    harm_init(&h, 12345);
    param(&h, SP_HARMPHRASE, 8);
    /* the chain: 4000 steps */
    static const char* const allowed[8] = {"", "1234567", "2457", "234567", "457", "1567", "24567", "136"};
    int bad = 0, cad_ok = 0, cad_n = 0, par = 0, steps = 4000, forbidden = 0;
    int prev = 1;
    h.fresh = 1;
    harm_step(&h);
    int pc[3];
    memcpy(pc, h.chord, sizeof(pc));
    printf("phrases (functions, chords as degrees):\n");
    for (int i = 0; i < steps; ++i) {
        const int pos = h.pos;
        harm_step(&h);
        const int f = h.func;
        const int cadence_step = pos == h.phrase - 1 || pos == h.phrase - 2;
        if (!strchr(allowed[prev], '0' + f) && !(cadence_step && (f == 5 || f == 1 || f == 6)))
            ++bad;
        if ((prev == 3 && f == 1) || (prev == 4 && f == 2) || (prev == 6 && (f == 1 || f == 3)) ||
            (prev == 2 && f == 1) || (prev == 4 && f == 1))
            ++forbidden;
        if (pos == h.phrase - 1) {
            ++cad_n;
            cad_ok += f == 1 || f == 6 || f == 5;
        }
        for (int a = 0; a < 2; ++a)
            for (int b = a + 1; b < 3; ++b) {
                const int x = (pc[a] - pc[b] + 70) % 7, y = (h.chord[a] - h.chord[b] + 70) % 7;
                if ((x == 4 && y == 4) || (x == 0 && y == 0))
                    ++par;
            }
        memcpy(pc, h.chord, sizeof(pc));
        if (i < 24) {
            printf("%d[%d %d %d] ", f, h.chord[0], h.chord[1], h.chord[2]);
            if (pos == h.phrase - 1)
                printf("|\n");
        }
        prev = f;
    }
    printf("\n%d steps: %d off the chain, %d forbidden, cadences %d of %d phrase ends, %d parallel pairs, "
           "melody %d notes\n", steps, bad, forbidden, cad_ok, cad_n, par, h.mlen);
    check(bad == 0, "every step follows the chain (or is a cadence)");
    check(forbidden == 0, "never 3-1, 4-2, 6-1, 6-3, 2-1, 4-1");
    check(cad_ok == cad_n, "every phrase ends in a cadence (1, 6 or 5)");
    check(par < steps / 20, "parallel fifths / octaves rare (only where unavoidable)");

    /* ticks: a bar per chord, notes balanced, at most three sounding */
    harm_init(&h, 777);
    param(&h, SP_HARMONY, 1);
    struct smp_event st = {EV_START, 0, 0, 0, 0};
    harm_rx(&h, &st);
    int sounding[128] = {0}, ons = 0, offs = 0, maxs = 0, nows = 0, frames = 0;
    for (uint32_t s = 0; s < 32 * 64; ++s) { /* 64 bars of 32nds */
        struct smp_event t = {EV_TICK, 0, 1, 0, (int32_t)s};
        harm_rx(&h, &t);
        struct harm_frame f;
        while (harm_pop(&h, &f)) {
            ++frames;
            for (int k = 0; k < f.n; ++k) {
                const struct smp_event* e = &f.ev[k];
                if (e->type != EV_SYN_NOTE)
                    continue;
                if (e->b) {
                    ++ons;
                    nows += !sounding[e->a]++;
                } else {
                    ++offs;
                    if (sounding[e->a])
                        nows -= !--sounding[e->a];
                }
            }
            if (nows > maxs)
                maxs = nows;
        }
    }
    struct smp_event sp = {EV_SYN_PARAM, 0, SP_HARMONY, 0, 0};
    harm_rx(&h, &sp);
    struct harm_frame f;
    while (harm_pop(&h, &f))
        for (int k = 0; k < f.n; ++k)
            if (!f.ev[k].b && sounding[f.ev[k].a])
                nows -= !--sounding[f.ev[k].a], ++offs;
    printf("ticks: %d frames, %d note ons, %d offs, at most %d sounding, %d left after off\n", frames, ons, offs,
           maxs, nows);
    check(ons > 20 && maxs <= 3, "chords play, never more than three notes");
    check(nows == 0, "turning it off silences every note");

    /* offset mode on the beat: the next chord's offset goes out a step ahead with its 32nd; the
       FPGA (simulated) applies it on that tick */
    harm_init(&h, 4242);
    param(&h, SP_HARMSEED, 3);
    param(&h, SP_HARMONY, 2);
    harm_rx(&h, &st);
    {
        int applied = 0, pend = 0, pend_ofs = 0, now_n = 0, late = 0, notes = 0, out_of_range = 0, distinct = 0;
        uint32_t pend_at = 0, last_at = 0;
        int bad_grid = 0, seen[16] = {0};
        for (uint32_t s = 0; s < 32 * 32; ++s) {
            if (pend && (int32_t)(s - pend_at) >= 0) { /* the FPGA's tick: before the ESP sees it */
                if (s != pend_at)
                    ++late;
                applied = pend_ofs, pend = 0;
            }
            struct smp_event t = {EV_TICK, 0, 1, 0, (int32_t)s};
            harm_rx(&h, &t);
            struct harm_frame f;
            while (harm_pop(&h, &f))
                for (int k = 0; k < f.n; ++k) {
                    const struct smp_event* e = &f.ev[k];
                    if (e->type == EV_SYN_NOTE) {
                        ++notes;
                        continue;
                    }
                    if (e->type != EV_SYN_HOFS)
                        continue;
                    const int o = (int)e->a - 64;
                    if (o < -3 || o > 3)
                        ++out_of_range;
                    if (!e->b)
                        applied = o, ++now_n;
                    else {
                        if ((uint32_t)e->value % 32 || (last_at && (uint32_t)e->value != last_at + 32) ||
                            (int32_t)((uint32_t)e->value - s) <= 0)
                            ++bad_grid;
                        last_at = pend_at = (uint32_t)e->value, pend_ofs = o, pend = 1;
                    }
                }
            if (!seen[applied + 8]++)
                ++distinct;
        }
        printf("offset mode, beat: %lu chords, %d sent now, %d applied late, %d off the grid, %d out of range, "
               "%d distinct offsets, %d notes\n", (unsigned long)h.chords, now_n, late, bad_grid, out_of_range,
               distinct, notes);
        check(now_n == 1 && late == 0 && bad_grid == 0, "the first chord now, then each one ahead on its bar");
        check(out_of_range == 0 && distinct >= 3, "offsets within half an octave, the chords move");
        check(notes == 0, "offset mode plays no notes itself");
    }

    /* offset mode on trigs (harmTrig 5): a pending offset is out before every trig */
    harm_init(&h, 99);
    param(&h, SP_HARMSEED, 7);
    param(&h, SP_HARMTRIG, 5);
    param(&h, SP_HARMONY, 2);
    {
        int pend = 0, pend_ofs = 0, missing = 0, applied = 0, distinct = 0, seen[16] = {0}, trigs = 0;
        struct harm_frame f;
        while (harm_pop(&h, &f)) /* what turning it on sent: the first chord, waiting for a trig */
            for (int k = 0; k < f.n; ++k)
                if (f.ev[k].type == EV_SYN_HOFS && f.ev[k].b)
                    pend_ofs = (int)f.ev[k].a - 64, pend = 1;
        for (uint32_t s = 0; s < 32 * 16; ++s) {
            if (s % 8 == 0) { /* a kick every beat: the FPGA applies, then tells the ESP */
                ++trigs;
                if (pend)
                    applied = pend_ofs, pend = 0;
                else
                    ++missing;
                struct smp_event tr = {EV_SYN_TRIG, 0, 0, 0, 0};
                harm_rx(&h, &tr);
            }
            struct smp_event t = {EV_TICK, 0, 1, 0, (int32_t)s};
            harm_rx(&h, &t);
            while (harm_pop(&h, &f))
                for (int k = 0; k < f.n; ++k)
                    if (f.ev[k].type == EV_SYN_HOFS) {
                        if (f.ev[k].b)
                            pend_ofs = (int)f.ev[k].a - 64, pend = 1;
                        else
                            applied = (int)f.ev[k].a - 64;
                    }
            if (s % 8 == 0 && !seen[applied + 8]++)
                ++distinct;
        }
        /* the parameters again (the FPGA repeats them every second): nothing restarts */
        const uint32_t before = h.chords;
        const int midx = h.midx;
        param(&h, SP_HARMONY, 2);
        param(&h, SP_HARMTRIG, 5);
        param(&h, SP_HARMSEED, 7);
        printf("offset mode, trigs: %d trigs, %d without an offset ready, %d distinct offsets\n", trigs, missing,
               distinct);
        check(missing == 0 && distinct >= 3, "every trig finds the next chord's offset waiting");
        check(h.chords == before && h.midx == midx, "repeated parameters change nothing (the melody runs on)");
        while (harm_pop(&h, &f))
            ;
    }

    /* play mode on trigs with a gate: chords on the trigs, off after half the trig interval */
    harm_init(&h, 5);
    param(&h, SP_HARMTRIG, 5);
    param(&h, SP_HARMGATE, 0.5);
    param(&h, SP_HARMONY, 1);
    {
        int on_at_trig = 0, on_elsewhere = 0, off_mid = 0, trigs = 0;
        struct harm_frame f;
        for (uint32_t s = 0; s < 32 * 16; ++s) {
            struct smp_event t = {EV_TICK, 0, 0, 0, (int32_t)s}; /* stopped: trigs work anyway */
            harm_rx(&h, &t);
            if (s % 16 == 0) {
                ++trigs;
                struct smp_event tr = {EV_SYN_TRIG, 0, 0, 0, 0};
                harm_rx(&h, &tr);
            }
            while (harm_pop(&h, &f))
                for (int k = 0; k < f.n; ++k) {
                    if (f.ev[k].type != EV_SYN_NOTE)
                        continue;
                    if (f.ev[k].b)
                        (s % 16 == 0 ? ++on_at_trig : ++on_elsewhere);
                    else if (s % 16 == 8)
                        ++off_mid;
                }
        }
        printf("play mode, trigs: %d trigs, %d note ons on them, %d elsewhere, %d offs half way\n", trigs,
               on_at_trig, on_elsewhere, off_mid);
        check(on_at_trig >= 3 * (trigs - 1) && on_elsewhere == 0, "the chords strike on the trigs");
        check(off_mid >= 3 * (trigs - 2), "harmGate 0.5: off half way to the next trig");
    }
    printf(fails ? "%d FAILED\n" : "ALL PASS\n", fails);
    return fails ? 1 : 0;
}
