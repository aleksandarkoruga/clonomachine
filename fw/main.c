/* ULX3S sampler firmware: SD -> SDRAM loader, event loop (USB console + ESP32-S3 UART),
 * text shell on the console for bring-up. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <irq.h>
#include <system.h>
#include <libbase/console.h>
#include <libbase/uart.h>
#include <generated/csr.h>
#include <generated/soc.h>

#include "engine.h"
#include "mapping.h"
#include "samples.h"
#include "slip.h"
#include "slot.h"
#include "usbh.h"

static struct slip g_slip_console, g_slip_esp;
extern volatile unsigned int uart_rx_overflows; /* fw/uart.c */
static uint32_t g_cin_drops;

static void dispatch(const struct slip* d) {
    for (unsigned i = 0; i + 8 < d->len; i += 8) {
        struct smp_event e;
        memcpy(&e, d->buf + i, 8);
        mapping_event(&e);
    }
}

/* ---- status ------------------------------------------------------------------------- */
static void print_q16(int32_t v) {
    const int neg = v < 0;
    uint32_t a = neg ? (uint32_t)-v : (uint32_t)v;
    printf("%s%lu.%03lu", neg ? "-" : "", (unsigned long)(a >> 16), (unsigned long)(((a & 0xFFFF) * 1000) >> 16));
}

static void cmd_status(void) {
    printf("magic %08lx  fs %lu Hz  bpm %lu.%03lu  cycles %lu (max %lu / %d)  samples %lu  req drops %lu  pv misses %lu\n",
           (unsigned long)eng_status(SMP_SG_MAGIC), (unsigned long)(eng_fs_q16() >> 16),
           (unsigned long)(g_bpm_milli / 1000), (unsigned long)(g_bpm_milli % 1000),
           (unsigned long)eng_status(SMP_SG_CYCLES), (unsigned long)eng_status(SMP_SG_CYCLES_MAX),
           ENGINE_CYCLES_PER_SAMPLE, (unsigned long)eng_status(SMP_SG_SAMPLES),
           (unsigned long)eng_status(SMP_SG_REQ_DROPS), (unsigned long)eng_status(SMP_SG_PV_MISSES));
    printf("beat %lu  links: console %lu frames / %lu bad / %lu rx overflows, esp %lu frames / %lu bad\n",
           (unsigned long)eng_status(SMP_SG_BEAT_HI), (unsigned long)g_slip_console.frames,
           (unsigned long)g_slip_console.crc_errors, (unsigned long)(uart_rx_overflows + g_cin_drops),
           (unsigned long)g_slip_esp.frames,
           (unsigned long)g_slip_esp.crc_errors);
    for (int s = 0; s < NUM_SLOTS; ++s) {
        const struct slot* sl = &g_slots[s];
        printf("slot %d: lib %d bank %d buf %d  phasor %lu  grains %lu  gate %lu  misses %lu  ratio ", s, sl->lib, sl->bank, sl->buf,
               (unsigned long)(eng_status(s * 8 + SMP_S_PHASOR) >> 8), (unsigned long)eng_status(s * 8 + SMP_S_ACTIVE),
               (unsigned long)eng_status(s * 8 + SMP_S_GATE), (unsigned long)eng_status(s * 8 + SMP_S_CACHE_MISS));
        print_q16((int32_t)(slot_tempo_ratio_q24(s) >> 8));
        printf("  rate ");
        print_q16(sl->rate_q16);
        printf("  pitch ");
        print_q16(sl->pitch_q16);
        printf("  16ths %lu  tempoLock %d  bgAdvance %d  algo %s", (unsigned long)slot_sixteenths(s), sl->tempo_lock,
               sl->bg_advance, sl->algo ? "pv" : "grain");
        {
            const uint32_t pv = eng_status(s * 8 + SMP_S_PV);
            printf(" (pv %s, xfade %lu%%)  out %s\n", (pv & 1) ? "on" : "off",
                   (unsigned long)(((pv >> 16) * 100 + 32768) >> 16), sl->out == 0 ? "L" : sl->out == 1 ? "R" : "LR");
        }
    }
}

/* ---- shell -------------------------------------------------------------------------- */
static int32_t parse_q16(const char* s) {
    /* decimal with optional fraction -> Q16.16 */
    int neg = 0;
    if (*s == '-') { neg = 1; ++s; }
    int32_t ip = 0;
    while (*s >= '0' && *s <= '9') ip = ip * 10 + (*s++ - '0');
    uint32_t frac = 0, div = 1;
    if (*s == '.') {
        ++s;
        while (*s >= '0' && *s <= '9' && div < 100000) { frac = frac * 10 + (uint32_t)(*s++ - '0'); div *= 10; }
    }
    int32_t v = ip * 65536 + (int32_t)(((uint64_t)frac << 16) / div);
    return neg ? -v : v;
}

#define SMP_PARAM_NAME(id, name, frac) #name,
#define SMP_PARAM_FRAC(id, name, frac) frac,
static const char* const g_param_names[P_NUM_PARAMS] = {SMP_PARAMS(SMP_PARAM_NAME)};
static const uint8_t g_param_frac[P_NUM_PARAMS] = {SMP_PARAMS(SMP_PARAM_FRAC)};

static int name_eq(const char* a, const char* b) { /* case-insensitive */
    for (; *a && *b; ++a, ++b)
        if ((*a | 32) != (*b | 32))
            return 0;
    return *a == *b;
}

static void help(void) {
    puts("status | list [lib [bank]] | pool | rescan | bank <slot> <n> | buf <slot> <n> | bpm <x>\n"
         "trig <slot> [pos] | note <slot> <note> [vel] | set <slot> <param> <value> | gatetest <slot>\n"
         "dsm <0|1> | norm <0|1> | verbose <0|1> | reset\n"
         "usb | usbdesc | midimon <0|1|2 raw|3 raw+clocks> | usbpull <0 pull-downs|1 released> | clock <0|1> (follow MIDI clock)\n"
         "midimap [reload] (MIDI pages from /midimap.json) | midiin <status> <d1> <d2> (hex, as from USB)");
    fputs("params:", stdout);
    for (int p = 0; p < P_NUM_PARAMS; ++p)
        printf(" %s", g_param_names[p]);
    putchar('\n');
}

static void apply_all(void) {
    for (int s = 0; s < NUM_SLOTS; ++s)
        slot_apply(s);
    set_global_bpm(g_bpm_milli);
}

static void shell_line(char* line) {
    char* argv[6];
    int argc = 0;
    for (char* t = strtok(line, " \t"); t && argc < 6; t = strtok(NULL, " \t"))
        argv[argc++] = t;
    if (!argc)
        return;
    const char* c = argv[0];
    struct smp_event e = {0};
    if (!strcmp(c, "help") || !strcmp(c, "?")) help();
    else if (!strcmp(c, "status")) cmd_status();
    else if (!strcmp(c, "list")) samples_list(argc > 1 ? atoi(argv[1]) : -1, argc > 2 ? atoi(argv[2]) : -1);
    else if (!strcmp(c, "verbose") && argc > 1) g_sd_verbose = atoi(argv[1]);
    else if (!strcmp(c, "bank") && argc > 2) slot_select_bank(atoi(argv[1]), atoi(argv[2]));
    else if (!strcmp(c, "pool")) samples_stats();
    else if (!strcmp(c, "sdbench")) samples_sdbench();
    else if (!strcmp(c, "gatetest") && argc > 1) {
        /* trigger, then time the gate (open -> ramp-down starts) in engine samples */
        const int s = atoi(argv[1]);
        const unsigned g = (unsigned)(s * 8 + SMP_S_GATE);
        slot_trig(s);
        uint32_t t0 = eng_status(SMP_SG_SAMPLES), t1 = t0, open_at = 0;
        while (eng_status(SMP_SG_SAMPLES) - t0 < 48000 * 10) {
            const uint32_t st = eng_status(g), now = eng_status(SMP_SG_SAMPLES);
            if (!open_at && (st == 1 || st == 2 || st == 4)) open_at = now;
            if (open_at && (st == 3 || st == 0)) { t1 = now; break; }
        }
        printf("gate %lu samples (%lu ms) buf %d bpm %lu\n", (unsigned long)(t1 - open_at),
               (unsigned long)((t1 - open_at) / 48), g_slots[s].buf, (unsigned long)(g_bpm_milli / 1000));
    }
    else if (!strcmp(c, "buf") && argc > 2) slot_select(atoi(argv[1]), atoi(argv[2]));
    else if (!strcmp(c, "rescan")) {
        samples_init();
        for (int s = 0; s < NUM_SLOTS; ++s) {
            g_slots[s].bank = g_slots[s].buf = -1;
            slot_select_bank(s, g_slots[s].want_bank);
        }
        apply_all();
    }
    else if (!strcmp(c, "reset")) ctrl_reset_write(1);
    else if (!strcmp(c, "usb")) { usbh_status(); mapping_midi_status(); }
    else if (!strcmp(c, "usbdesc")) usbh_descriptors();
    else if (!strcmp(c, "midimap")) { if (argc > 1) mapping_midimap_load(); mapping_midimap_print(); }
    else if (!strcmp(c, "midiin") && argc > 1) /* hex bytes, as if they came from USB */
        mapping_midi((uint8_t)strtoul(argv[1], NULL, 16), (uint8_t)(argc > 2 ? strtoul(argv[2], NULL, 16) : 0),
                     (uint8_t)(argc > 3 ? strtoul(argv[3], NULL, 16) : 0));
    else if (!strcmp(c, "midimon") && argc > 1) g_midi_monitor = atoi(argv[1]);
    else if (!strcmp(c, "usbpull") && argc > 1) usbh_set_pull(atoi(argv[1]));
    else if (!strcmp(c, "usbdbg")) usbh_debug();
    else if (!strcmp(c, "usbwatch")) usbh_watch(argc > 1 ? atoi(argv[1]) : 2000);
    else if (!strcmp(c, "clock") && argc > 1) g_clock_follow = atoi(argv[1]);
    else if (!strcmp(c, "bpm") && argc > 1) set_global_bpm((uint32_t)(((int64_t)parse_q16(argv[1]) * 1000) >> 16));
    else if (!strcmp(c, "dsm") && argc > 1) sampler_ctrl_write((sampler_ctrl_read() & ~1u) | (atoi(argv[1]) & 1));
    else if (!strcmp(c, "norm") && argc > 1) g_normalize = atoi(argv[1]);
    else if (!strcmp(c, "trig") && argc > 1) {
        const int s = atoi(argv[1]);
        if (s >= 0 && s < NUM_SLOTS) {
            if (argc > 2) g_slots[s].pos_q30 = (uint32_t)parse_q16(argv[2]) << 14;
            slot_trig(s);
        }
    } else if (!strcmp(c, "note") && argc > 2) {
        e.type = EV_NOTE_ON;
        e.chan = (uint8_t)(9 + atoi(argv[1]));
        e.a = (uint8_t)atoi(argv[2]);
        e.b = (uint8_t)(argc > 3 ? atoi(argv[3]) : 1);
        mapping_event(&e);
    } else if (!strcmp(c, "set") && argc > 3) {
        int p = 0;
        while (p < P_NUM_PARAMS && !name_eq(g_param_names[p], argv[2])) ++p;
        if (p == P_NUM_PARAMS) { printf("unknown param %s\n", argv[2]); return; }
        e.type = EV_RAW_PARAM;
        e.a = (uint8_t)atoi(argv[1]);
        e.b = (uint8_t)p;
        if (!g_param_frac[p])
            e.value = atoi(argv[3]);
        else if (argv[3][0] == '-' && p == P_POS)
            e.value = -1;                                   /* pos < 0: run free */
        else
            e.value = (int32_t)((int64_t)parse_q16(argv[3]) << (g_param_frac[p] - 16));
        mapping_event(&e);
    } else
        printf("? %s (try help)\n", c);
}

/* Console bytes: SLIP event frames and shell text share the link.
 *   ISR ring (8 KB, fw/uart.c) -> console_drain() -> g_cin (16 KB) -> route_byte()
 * route_byte() dispatches complete frames immediately, also from inside the SD loader
 * (samples_idle_hook), so a busy Monomachine never backs up while banks load. Shell
 * text is only executed from the main loop (it may rescan the card). */
static char g_line[96];
static unsigned g_line_len;

static void text_byte(uint8_t c) {
    if (c == '\r' || c == '\n') {
        putchar('\n');
        g_line[g_line_len] = 0;
        shell_line(g_line);
        g_line_len = 0;
        printf("smp> ");
    } else if (c == 0x7F || c == 0x08) {
        if (g_line_len) { --g_line_len; fputs("\x08 \x08", stdout); }
    } else if (c >= 0x20 && c < 0x7F && g_line_len < sizeof(g_line) - 1) {
        g_line[g_line_len++] = (char)c;
        putchar(c);
    }
}

static uint8_t g_cin[16384];
static unsigned g_cin_w, g_cin_r;
static uint8_t g_text[512];          /* text seen while in the loader, run later */
static unsigned g_text_w, g_text_r;

static void console_drain(void) {
    while (readchar_nonblock()) {
        const uint8_t c = (uint8_t)getchar();
        const unsigned nw = (g_cin_w + 1) & (sizeof(g_cin) - 1);
        if (nw != g_cin_r) {
            g_cin[g_cin_w] = c;
            g_cin_w = nw;
        } else
            ++g_cin_drops;
    }
}

static void route_byte(uint8_t c, int in_loader) {
    /* An event frame always starts with its type byte (0..10, never printable):
       printable text right after an END means "not a frame" (stray 0xC0 / typing). */
    if (g_slip_console.in_frame && g_slip_console.len == 0 && !g_slip_console.esc && c >= 0x20 && c < 0x7F)
        g_slip_console.in_frame = 0;
    if (c == SLIP_END || g_slip_console.in_frame) {
        if (slip_feed(&g_slip_console, c))
            dispatch(&g_slip_console);
        return;
    }
    if (in_loader) {
        const unsigned nw = (g_text_w + 1) & (sizeof(g_text) - 1);
        if (nw != g_text_r) {
            g_text[g_text_w] = c;
            g_text_w = nw;
        }
        return;
    }
    text_byte(c);
}

/* samples_drain_hook: inside SD sector reads, buffer only (UART input, USB-MIDI events) */
static void io_drain(void) {
    console_drain();
    usbh_service();
}

/* samples_idle_hook: called by the loader between slices of work */
static void console_pump(void) {
    console_drain();
    usbh_poll(); /* MIDI keeps flowing while banks load */
    while (g_cin_r != g_cin_w) {
        const uint8_t c = g_cin[g_cin_r];
        g_cin_r = (g_cin_r + 1) & (sizeof(g_cin) - 1);
        route_byte(c, 1);
    }
}

static void poll_esp(void) {
#ifdef CSR_ESP_UART_BASE
    for (int n = 0; n < 64 && !esp_uart_rxempty_read(); ++n) {
        const uint8_t c = (uint8_t)esp_uart_rxtx_read();
        esp_uart_ev_pending_write(UART_EV_RX);
        if (slip_feed(&g_slip_esp, c))
            dispatch(&g_slip_esp);
    }
#endif
}

int main(void) {
#ifdef CONFIG_CPU_HAS_INTERRUPT
    irq_setmask(0);
    irq_setie(1);
#endif
    uart_init();
    printf("\n\e[1mULX3S sampler\e[0m  slots %d  audio clk %lu Hz  fs %lu.%03lu Hz\n", NUM_SLOTS,
           (unsigned long)AUDIO_CLK_FREQ, (unsigned long)(eng_fs_q16() >> 16),
           (unsigned long)(((eng_fs_q16() & 0xFFFF) * 1000) >> 16));

    sampler_ctrl_write(1); /* engine reset while loading */
    sampler_ctrl_write(0);
    if (eng_status(SMP_SG_MAGIC) != SMP_MAGIC)
        printf("WARNING: engine status magic %08lx\n", (unsigned long)eng_status(SMP_SG_MAGIC));

    mapping_init();
    slots_init();
    /* 's' within 1 s skips the SD card (LiteX's SD driver blocks when no card is present) */
    puts("press 's' within 1 s to skip the SD card");
    int skip_sd = 0;
    for (int i = 0; i < 1000 && !skip_sd; ++i) {
        busy_wait(1);
        while (readchar_nonblock())
            if (getchar() == 's')
                skip_sd = 1;
    }
    if (!skip_sd)
        puts("SD: mounting ...");
    if (skip_sd || samples_init() == 0) {
        puts("no samples on SD: using built-in test loops");
        samples_load_builtin();
        for (int s = 0; s < NUM_SLOTS; ++s)
            g_slots[s].lib = 0;
    }
    eng_write(SMP_ADDR(SMP_GLOBAL_SLOT, SMP_G_MASTER), 4096);
    for (int s = 0; s < NUM_SLOTS; ++s)
        slot_select(s, (g_slots[s].lib == 0 && s) ? 1 : 0);
    apply_all();
    help();
    printf("smp> ");

    mapping_midimap_load();
    usbh_init();
    samples_idle_hook = console_pump;  /* loader slices: dispatch events */
    samples_drain_hook = io_drain;      /* inside SD reads: buffer only */
    for (;;) {
        console_drain();
        while (g_text_r != g_text_w) {
            const uint8_t c = g_text[g_text_r];
            g_text_r = (g_text_r + 1) & (sizeof(g_text) - 1);
            text_byte(c);
        }
        while (g_cin_r != g_cin_w) {
            const uint8_t c = g_cin[g_cin_r];
            g_cin_r = (g_cin_r + 1) & (sizeof(g_cin) - 1);
            route_byte(c, 0);
        }
        poll_esp();
        usbh_poll();
        samples_poll();
        slots_service();
    }
    return 0;
}
