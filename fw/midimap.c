#include "midimap.h"

#include <stdio.h>
#include <string.h>

#include "jsonp.h"

const char* const mm_target_names[MT_NUM] = {
    "none",   "bufNum", "bank",      "rate",        "pitch",    "gate",     "interval",  "amp",
    "overlap", "randPos", "randomPitch", "prob",    "separation", "maxGrains", "tempoLock", "bgAdvance",
    "envHold", "pvTransient", "algo",     "grainDur", "noteMode", "out",      "cutter", "repeats", "fxSend", "volume"};

static int name_eq(const char* a, const char* b) { /* case-insensitive */
    for (; *a && *b; ++a, ++b)
        if ((*a | 32) != (*b | 32))
            return 0;
    return *a == *b;
}

const char* const rk_names[RK_NUM] = {"none",   "size",  "decay",  "damp",      "diffusion", "wander", "return",
                                       "glide",  "spread", "glideMode", "send0",  "send1",     "send2"};

int rk_of(const char* name) {
    for (int t = 0; t < RK_NUM; ++t)
        if (name_eq(name, rk_names[t]))
            return t;
    return -1;
}

static void reverb_defaults(struct mm_reverb* r) {
    static const uint8_t k[] = {RK_SIZE,  RK_DECAY, RK_DAMP,  RK_DIFFUSION, RK_WANDER, RK_RETURN,
                                RK_SEND0, RK_SEND1, RK_SEND2, RK_GLIDE,     RK_SPREAD, RK_GLIDEMODE};
    memset(r, 0, sizeof(*r));
    r->chan = 7; /* channel 8 */
    r->cc = 16;
    r->nknobs = sizeof(k);
    memcpy(r->knobs, k, sizeof(k));
    r->mw = RK_GLIDE;
}

int mm_target_of(const char* name) {
    for (int t = 0; t < MT_NUM; ++t)
        if (name_eq(name, mm_target_names[t]))
            return t;
    return -1;
}

/* every control on a CC (the Machinedrum's MIDI machine sends 6 at a time: its CC numbers can
   be moved along the list). The volume is on velocity, so the 7th CC (the volume's before) has
   the stutter repeats; amp isn't on a CC (unity, the cutter's accents). */
static const uint8_t g_default_knobs[] = {
    MT_BUFNUM, MT_BANK,     MT_RATE,       MT_PITCH,     MT_GATE,      MT_INTERVAL,  MT_REPEATS,  MT_FXSEND, MT_OVERLAP,
    MT_RANDPOS, MT_RANDOMPITCH, MT_PROB, MT_SEPARATION, MT_MAXGRAINS, MT_TEMPOLOCK, MT_BGADVANCE, MT_ENVHOLD, MT_PVTRANSIENT,
    MT_NOTEMODE, MT_ALGO,   MT_GRAINDUR,   MT_OUT,       MT_VOLUME};

static void page_init(struct mm_page* p, int chan1, int voice, int cc, int rate_lo, int rate_hi, int pb) {
    memset(p, 0, sizeof(*p));
    p->chan = (uint8_t)(chan1 - 1);
    p->voice = (uint8_t)voice;
    p->cc = (uint8_t)cc;
    p->nknobs = sizeof(g_default_knobs);
    memcpy(p->knobs, g_default_knobs, sizeof(g_default_knobs));
    p->rate_lo = rate_lo * 65536;
    p->rate_hi = rate_hi * 65536;
    p->mw = MT_ALGO;
    p->at = MT_GRAINDUR;
    p->pb = (uint8_t)pb;
    p->vel = MT_VOLUME;
    p->note_off = 1;
}

void midimap_defaults(struct midimap* m) {
    memset(m, 0, sizeof(*m));
    m->clock_follow = 1;
    m->cutter_chan = 7; /* the cutter's structure controls (on the ESP) */
    reverb_defaults(&m->reverb);
    page_init(&m->page[0], 5, 0, 16, 0, 4, MT_CUTTER);    /* Machinedrum MIDI machine on channel 5: drums */
    page_init(&m->page[1], 3, 1, 16, -2, 2, MT_NOTEMODE); /* channel 3: melody */
    page_init(&m->page[2], 6, 2, 25, -2, 2, MT_NOTEMODE); /* channel 6: third voice */
    m->npages = 3;
}

static int target(struct json* j, uint8_t* out, char* err, int errn) {
    char s[24];
    if (!json_string(j, s, sizeof(s)))
        return 0;
    const int t = mm_target_of(s);
    if (t < 0) {
        snprintf(err, (size_t)errn, "unknown control \"%s\"", s);
        j->err = 1;
        return 0;
    }
    *out = (uint8_t)t;
    return 1;
}

static int rk_target(struct json* j, uint8_t* out, char* err, int errn) {
    char s[24];
    if (!json_string(j, s, sizeof(s)))
        return 0;
    const int t = rk_of(s);
    if (t < 0) {
        snprintf(err, (size_t)errn, "unknown reverb control \"%s\"", s);
        j->err = 1;
        return 0;
    }
    *out = (uint8_t)t;
    return 1;
}

static int parse_reverb(struct json* j, struct mm_reverb* r, char* err, int errn) {
    char key[24];
    int first = 1, chan = 8, cc = 16;
    while (json_member(j, &first, key, sizeof(key))) {
        if (name_eq(key, "channel"))
            json_int(j, &chan);
        else if (name_eq(key, "cc"))
            json_int(j, &cc);
        else if (name_eq(key, "knobs")) {
            int f = 1;
            r->nknobs = 0;
            while (json_element(j, &f)) {
                uint8_t t = 0;
                if (!rk_target(j, &t, err, errn))
                    return 0;
                if (r->nknobs < MM_MAX_KNOBS)
                    r->knobs[r->nknobs++] = t;
            }
        } else if (name_eq(key, "mw")) {
            if (!rk_target(j, &r->mw, err, errn))
                return 0;
        } else if (name_eq(key, "at")) {
            if (!rk_target(j, &r->at, err, errn))
                return 0;
        } else if (name_eq(key, "pb")) {
            if (!rk_target(j, &r->pb, err, errn))
                return 0;
        } else
            json_skip(j);
    }
    if (j->err)
        return 0;
    if (chan < 0 || chan > 16 || cc < 0 || cc > 127) {
        snprintf(err, (size_t)errn, "reverb: channel 1..16 (0: none), cc 0..127");
        return 0;
    }
    r->chan = chan ? (uint8_t)(chan - 1) : 0xFF;
    r->cc = (uint8_t)cc;
    return 1;
}

static int parse_page(struct json* j, struct mm_page* p, char* err, int errn) {
    char key[24];
    int first = 1, chan = 1, voice = 0, cc = 0;
    page_init(p, 1, 0, 0, -2, 2, MT_NOTEMODE);
    while (json_member(j, &first, key, sizeof(key))) {
        if (name_eq(key, "channel"))
            json_int(j, &chan);
        else if (name_eq(key, "voice"))
            json_int(j, &voice);
        else if (name_eq(key, "cc"))
            json_int(j, &cc);
        else if (name_eq(key, "rate")) {
            int f = 1, n = 0;
            int32_t v[2] = {p->rate_lo, p->rate_hi};
            while (json_element(j, &f))
                if (n < 2)
                    json_q16(j, &v[n++]);
                else
                    json_skip(j);
            p->rate_lo = v[0];
            p->rate_hi = v[1];
        } else if (name_eq(key, "knobs")) {
            int f = 1;
            p->nknobs = 0;
            while (json_element(j, &f)) {
                uint8_t t = 0;
                if (!target(j, &t, err, errn))
                    return 0;
                if (p->nknobs < MM_MAX_KNOBS)
                    p->knobs[p->nknobs++] = t;
            }
        } else if (name_eq(key, "mw")) {
            if (!target(j, &p->mw, err, errn))
                return 0;
        } else if (name_eq(key, "at")) {
            if (!target(j, &p->at, err, errn))
                return 0;
        } else if (name_eq(key, "pb")) {
            if (!target(j, &p->pb, err, errn))
                return 0;
        } else if (name_eq(key, "velocity")) {
            if (!target(j, &p->vel, err, errn))
                return 0;
        } else if (name_eq(key, "noteOff")) {
            int b = 1;
            json_bool(j, &b);
            p->note_off = (uint8_t)b;
        } else
            json_skip(j); /* "_comment" and the like */
    }
    if (j->err)
        return 0;
    if (chan < 1 || chan > 16 || voice < 0 || voice > 7 || cc < 0 || cc > 127) {
        snprintf(err, (size_t)errn, "page: channel 1..16, voice 0..7, cc 0..127");
        return 0;
    }
    p->chan = (uint8_t)(chan - 1);
    p->voice = (uint8_t)voice;
    p->cc = (uint8_t)cc;
    return 1;
}

int midimap_parse(struct midimap* m, const char* buf, int len, char* err, int errn) {
    struct midimap t;
    struct json j;
    char key[24];
    int first = 1;
    memset(&t, 0, sizeof(t));
    t.clock_follow = 1;
    t.cutter_chan = 7;
    reverb_defaults(&t.reverb);
    err[0] = 0;
    json_init(&j, buf, len);
    while (json_member(&j, &first, key, sizeof(key))) {
        if (name_eq(key, "clockFollow"))
            json_bool(&j, &t.clock_follow);
        else if (name_eq(key, "reverb")) {
            if (!parse_reverb(&j, &t.reverb, err, errn))
                break;
        } else if (name_eq(key, "cutterChannel")) {
            json_int(&j, &t.cutter_chan);
            if (t.cutter_chan < 0 || t.cutter_chan > 16) {
                snprintf(err, (size_t)errn, "cutterChannel: 1..16 (0: none)");
                j.err = 1;
            }
        }
        else if (name_eq(key, "pages")) {
            int f = 1;
            while (json_element(&j, &f)) {
                if (t.npages == MM_MAX_PAGES) {
                    json_skip(&j);
                    continue;
                }
                if (!parse_page(&j, &t.page[t.npages], err, errn))
                    break;
                ++t.npages;
            }
        } else
            json_skip(&j);
    }
    if (j.err || err[0]) {
        if (!err[0])
            snprintf(err, (size_t)errn, "syntax error at byte %d", json_pos(&j, buf));
        midimap_defaults(m);
        return -1;
    }
    *m = t;
    return 0;
}

/* Q16 -> "1.5" / "-2" (three decimals at most) */
static int q16_str(char* out, int n, int32_t v) {
    const int neg = v < 0;
    uint32_t a = (uint32_t)(neg ? -v : v);
    uint32_t ip = a >> 16, fp = (uint32_t)((((uint64_t)(a & 0xFFFF)) * 1000 + 32768) >> 16);
    if (fp >= 1000) {
        ++ip;
        fp -= 1000;
    }
    if (!fp)
        return snprintf(out, (size_t)n, "%s%u", neg ? "-" : "", (unsigned)ip);
    while (fp % 10 == 0)
        fp /= 10;
    return snprintf(out, (size_t)n, "%s%u.%u", neg ? "-" : "", (unsigned)ip, (unsigned)fp);
}

int midimap_write(const struct midimap* m, char* buf, int n) {
    int k = 0;
#define OUT(...) do { if (k < n) k += snprintf(buf + k, (size_t)(n - k), __VA_ARGS__); } while (0)
    OUT("{\n");
    OUT("  \"_comment\": \"MIDI pages for the ULX3S sampler. channel: 1-16 as the instrument shows it; "
        "voice: 0 drums, 1 melody, 2 third voice; cc: first CC of the page, the knobs follow on "
        "consecutive CCs; rate: range of the rate knob (63 = 1); velocity / mw / at / pb: a note's velocity, "
        "mod wheel, channel aftertouch, pitch bend (its coarse 7 bits); cutterChannel: MIDI channel "
        "passed on to the break cutter on the ESP32 (0: none); reverb: the plate reverb's page, controls "
        "size decay damp diffusion wander return glide spread glideMode send0 send1 send2 (the voices' sends). "
        "Controls: ");
    for (int t = 0; t < MT_NUM; ++t)
        OUT("%s%s", mm_target_names[t], t + 1 < MT_NUM ? " " : "");
    OUT(". gate: 1/32 bar .. 16 bars (exponential, 64 ~ 3/4 bar); volume: on top of amp, kept while the cutter "
        "plays; noteOff: false = a note-off doesn't end the gate; repeats: stutter retriggers per note; fxSend: reverb "
        "send; cutter: below the "
        "centre hands voice 0 to the cutter (ESP32 or OSC).\",\n");
    OUT("  \"clockFollow\": %s,\n", m->clock_follow ? "true" : "false");
    OUT("  \"cutterChannel\": %d,\n", m->cutter_chan);
    {
        const struct mm_reverb* r = &m->reverb;
        OUT("  \"reverb\": {\"channel\": %d, \"cc\": %d, \"knobs\": [", r->chan == 0xFF ? 0 : r->chan + 1, r->cc);
        for (int k2 = 0; k2 < r->nknobs; ++k2)
            OUT("\"%s\"%s", rk_names[r->knobs[k2]], k2 + 1 < r->nknobs ? ", " : "");
        OUT("],\n             \"mw\": \"%s\", \"at\": \"%s\", \"pb\": \"%s\"},\n", rk_names[r->mw], rk_names[r->at],
            rk_names[r->pb]);
    }
    OUT("  \"pages\": [\n");
    for (int i = 0; i < m->npages; ++i) {
        const struct mm_page* p = &m->page[i];
        char lo[16], hi[16];
        q16_str(lo, sizeof(lo), p->rate_lo);
        q16_str(hi, sizeof(hi), p->rate_hi);
        OUT("    {\"channel\": %d, \"voice\": %d, \"cc\": %d, \"rate\": [%s, %s],\n", p->chan + 1, p->voice, p->cc, lo, hi);
        OUT("     \"knobs\": [");
        for (int k2 = 0; k2 < p->nknobs; ++k2)
            OUT("\"%s\"%s", mm_target_names[p->knobs[k2]], k2 + 1 < p->nknobs ? ", " : "");
        OUT("],\n");
        OUT("     \"velocity\": \"%s\", \"noteOff\": %s, \"mw\": \"%s\", \"at\": \"%s\", \"pb\": \"%s\"}%s\n",
            mm_target_names[p->vel], p->note_off ? "true" : "false", mm_target_names[p->mw], mm_target_names[p->at],
            mm_target_names[p->pb], i + 1 < m->npages ? "," : "");
    }
    OUT("  ]\n}\n");
#undef OUT
    return k < n ? k : n - 1;
}
