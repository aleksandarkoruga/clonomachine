// Host check of fw/midimap.c + fw/jsonp.c: defaults -> JSON -> parse round trip, an edited
// file, and the error paths.
//
//   gcc -O2 -Wall -Ifw host/test_midimap.c fw/midimap.c fw/jsonp.c -o test_midimap && ./test_midimap
#include <stdio.h>
#include <string.h>

#include "midimap.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); ++fails; } } while (0)

int main(void) {
    struct midimap a, b;
    char buf[4096], err[96];
    midimap_defaults(&a);
    const int n = midimap_write(&a, buf, sizeof(buf));
    printf("%s", buf);
    CHECK(n > 0 && n < (int)sizeof(buf) - 1);
    CHECK(midimap_parse(&b, buf, n, err, sizeof(err)) == 0);
    CHECK(memcmp(&a, &b, sizeof(a)) == 0);

    const char* edited =
        "{ \"pages\": [ {\"channel\": 10, \"voice\": 1, \"cc\": 70, \"rate\": [-0.5, 3.25],"
        "  \"knobs\": [\"pitch\", \"none\", \"Gate\"], \"mw\": \"none\", \"pb\": \"out\", \"_comment\": [1, {\"x\": null}]} ],"
        "  \"clockFollow\": false }";
    CHECK(midimap_parse(&b, edited, (int)strlen(edited), err, sizeof(err)) == 0);
    CHECK(b.npages == 1 && b.page[0].chan == 9 && b.page[0].voice == 1 && b.page[0].cc == 70);
    CHECK(b.page[0].rate_lo == -32768 && b.page[0].rate_hi == 3 * 65536 + 16384);
    CHECK(b.page[0].nknobs == 3 && b.page[0].knobs[0] == MT_PITCH && b.page[0].knobs[1] == MT_NONE &&
          b.page[0].knobs[2] == MT_GATE);
    CHECK(b.page[0].mw == MT_NONE && b.page[0].at == MT_GRAINDUR && b.page[0].pb == MT_OUT);
    CHECK(b.clock_follow == 0);
    CHECK(b.page[0].vel == MT_AMP); /* a file without "velocity": the volume */
    const char* vel = "{ \"pages\": [ {\"channel\": 5, \"velocity\": \"repeats\", \"knobs\": [\"fxSend\"]} ] }";
    CHECK(midimap_parse(&b, vel, (int)strlen(vel), err, sizeof(err)) == 0);
    CHECK(b.page[0].vel == MT_REPEATS && b.page[0].knobs[0] == MT_FXSEND && b.page[0].pb == MT_NOTEMODE);

    const char* bad1 = "{ \"pages\": [ {\"channel\": 5, \"knobs\": [\"wobble\"]} ] }";
    CHECK(midimap_parse(&b, bad1, (int)strlen(bad1), err, sizeof(err)) != 0);
    printf("bad control  -> %s\n", err);
    CHECK(b.npages == 3); /* defaults */
    const char* bad2 = "{ \"pages\": [ {\"channel\": 5 \"voice\": 1} ] }";
    CHECK(midimap_parse(&b, bad2, (int)strlen(bad2), err, sizeof(err)) != 0);
    printf("missing comma -> %s\n", err);
    const char* bad3 = "{ \"pages\": [ {\"channel\": 17} ] }";
    CHECK(midimap_parse(&b, bad3, (int)strlen(bad3), err, sizeof(err)) != 0);
    printf("channel 17   -> %s\n", err);
    printf(fails ? "FAILURES\n" : "ALL PASS\n");
    return fails != 0;
}
