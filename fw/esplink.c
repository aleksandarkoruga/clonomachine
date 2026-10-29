/* The on-board ESP32 runs the break cutter (procedural, see the ESP firmware) and talks to the
 * sampler over its UART at 3 Mbaud, both ways in the SLIP + CRC-8 event frames of
 * include/sampler_events.h.
 *
 * ESP -> FPGA: EV_HELLO at least once a second (the link is up while they keep coming) and the
 * cut events (EV_RAW_PARAM / EV_LOAD_BUF ...), handled like the console's. Anything aimed at
 * voice 0 is dropped while MIDI owns it (drum page pitch bend above the centre), and the ESP
 * can't move the transport (BPM / START / STOP / CLOCK from it are ignored).
 *
 * The hello carries a signature (a = 'E', b = 'S', value = protocol version 1), so noise from
 * the ESP's boot can't bring the link up.
 *
 * "esp sim 1": the PC stands in for the ESP on the USB console (host/cutter_live.py): the same
 * frames go both ways there, mixed into the console text (0xC0-delimited).
 *
 * FPGA -> ESP, only while the link is up (before that the ESP may sit in its ROM or a stock
 * MicroPython prompt, which must not get bytes): EV_TICK on every 32nd (from the MIDI clock
 * while it runs, from the engine's beat otherwise), EV_BPM, EV_START / EV_STOP, EV_OWNER and
 * the MIDI messages of the cutter channel (midimap.json "cutterChannel"), EV_LOAD_BUF with the
 * drum page's sample knob (the cutter's base sample); bpm, owner and base sample again once a
 * second so a restarted ESP catches up.
 */
#include "esplink.h"

#include <stdio.h>
#include <string.h>

#include <generated/csr.h>
#include <libbase/uart.h> /* UART_EV_RX, uart_write: the console */

#include "../include/sampler_events.h"
#include "../include/sampler_regs.h"
#include "engine.h"
#include "mapping.h"
#include "slip.h"
#include "slot.h"

#ifdef CSR_ESP_UART_BASE

#define LINK_TIMEOUT (2 * 48000)  /* samples without a hello: the link is down */
#define MIDI_CLOCK_GONE (12000)   /* 250 ms without a MIDI-clock 32nd: ticks from the engine */

static struct slip g_rx;
static uint8_t g_ring[1024];      /* bytes received inside SD reads */
static uint32_t g_ring_r, g_ring_w;
enum { ESP, SIM, NPATHS }; /* the ESP's UART, the PC on the console ("esp sim 1") */
static struct {
    int up, playing, stopped, have_step; /* up: either path; stopped: a Stop came, no Start since */
    int sim, path_up[NPATHS], n16, base_buf; /* base_buf: drum page sample knob + 1, 0 unset */
    uint32_t hello_at[NPATHS], refresh_at, midi_at, version;
    uint32_t step, last_x;
    uint32_t tx_frames, tx_drops, dropped, ring_drops, ticks;
} g;

static uint32_t now(void) { return eng_status(SMP_SG_SAMPLES); }

/* ---- transmit ---------------------------------------------------------------------------- */
static int put(int path, uint8_t c) {
    if (path == SIM) {
        uart_write((char)c); /* raw: no newline translation */
        return 1;
    }
    if (esp_uart_txfull_read())
        return 0;
    esp_uart_rxtx_write(c);
    return 1;
}

static void send_on(int path, const uint8_t* raw, int len) {
    int ok = put(path, SLIP_END);
    for (int i = 0; i < len && ok; ++i) {
        const uint8_t c = raw[i];
        if (c == SLIP_END)
            ok = put(path, SLIP_ESC) && put(path, SLIP_ESC_END);
        else if (c == SLIP_ESC)
            ok = put(path, SLIP_ESC) && put(path, SLIP_ESC_ESC);
        else
            ok = put(path, c);
    }
    ok = ok && put(path, SLIP_END);
    if (ok)
        ++g.tx_frames;
    else
        ++g.tx_drops; /* TX FIFO full: the frame is cut short, the ESP's CRC drops it */
}

static void send(const struct smp_event* ev, int n) {
    uint8_t raw[8 * 4 + 1];
    memcpy(raw, ev, (size_t)(8 * n));
    raw[8 * n] = smp_crc8(raw, (unsigned)(8 * n));
    for (int p = 0; p < NPATHS; ++p)
        if (g.path_up[p])
            send_on(p, raw, 8 * n + 1);
}

static void send1(uint8_t type, uint8_t chan, uint8_t a, uint8_t b, int32_t value) {
    const struct smp_event e = {type, chan, a, b, value};
    send(&e, 1);
}

/* EV_OWNER's value: sixteenths of the sample voice 0 plays (the cutter's grid for pos) */
static void send_state(void) {
    send1(EV_BPM, 0, 0, 0, (int32_t)g_bpm_milli);
    g.n16 = (int)slot_sixteenths(0);
    send1(EV_OWNER, 0, 0, (uint8_t)g_md_slot0_external, g.n16);
    if (g.base_buf > 0)
        send1(EV_LOAD_BUF, 0, 0, (uint8_t)(g.base_buf - 1), 0);
}

static void tick(uint32_t step, int source) {
    g.step = step;
    g.have_step = 1;
    ++g.ticks;
    send1(EV_TICK, (uint8_t)source, (uint8_t)g.playing, 0, (int32_t)step);
}

/* ---- receive ----------------------------------------------------------------------------- */
static int allowed(const struct smp_event* e, int path) {
    switch (e->type) {
    case EV_HELLO:
        if (e->a != 'E' || e->b != 'S' || e->value != 1)
            return 0;
        g.hello_at[path] = now();
        if (!g.path_up[path]) {
            g.path_up[path] = 1;
            g.up = 1;
            g.version = (uint32_t)e->value;
            send_state();
        }
        return 0;
    case EV_BPM: case EV_START: case EV_STOP: case EV_CLOCK: case EV_TICK: case EV_OWNER:
        return 0; /* the transport is the FPGA's */
    case EV_RAW_PARAM: case EV_LOAD_BUF:
        return !(e->a == 0 && !g_md_slot0_external);
    default: /* channel events: Monomachine table, channel 10 (9 here) is voice 0 */
        return !(e->chan == 9 && !g_md_slot0_external);
    }
}

static void event_in(const struct smp_event* e, int path) {
    if (allowed(e, path))
        mapping_event(e);
    else if (e->type != EV_HELLO)
        ++g.dropped;
}

static void byte_in(uint8_t c) {
    if (!slip_feed(&g_rx, c))
        return;
    for (unsigned i = 0; i + 8 < g_rx.len; i += 8) {
        struct smp_event e;
        memcpy(&e, g_rx.buf + i, 8);
        event_in(&e, ESP);
    }
}

int esplink_sim(void) { return g.sim; }
void esplink_sim_event(const struct smp_event* e) { event_in(e, SIM); }
void esplink_set_sim(int on) {
    g.sim = on;
    if (!on)
        g.path_up[SIM] = 0;
}

void esplink_drain(void) {
    while (!esp_uart_rxempty_read()) {
        const uint8_t c = (uint8_t)esp_uart_rxtx_read();
        esp_uart_ev_pending_write(UART_EV_RX);
        if (g_ring_w - g_ring_r < sizeof(g_ring))
            g_ring[g_ring_w++ & (sizeof(g_ring) - 1)] = c;
        else
            ++g.ring_drops;
    }
}

void esplink_poll(void) {
    esplink_drain();
    while (g_ring_r != g_ring_w)
        byte_in(g_ring[g_ring_r++ & (sizeof(g_ring) - 1)]);
    const uint32_t t = now();
    for (int p = 0; p < NPATHS; ++p)
        if (g.path_up[p] && t - g.hello_at[p] > LINK_TIMEOUT)
            g.path_up[p] = 0;
    g.up = g.path_up[ESP] || g.path_up[SIM];
    if (!g.up)
        return;
    if (t - g.refresh_at > 48000 || (int)slot_sixteenths(0) != g.n16) {
        g.refresh_at = t;
        send_state();
    }
    /* no MIDI clock: a 32nd = an eighth of the engine's beat */
    if (t - g.midi_at > MIDI_CLOCK_GONE) {
        const uint32_t step = (eng_status(SMP_SG_BEAT_HI) << 3) | (eng_status(SMP_SG_BEAT_LO) >> 29);
        if (!g.have_step || step != g.step) {
            g.playing = !g.stopped;
            tick(step, 0);
        }
    }
}

/* ---- from the rest of the firmware ------------------------------------------------------- */
/* every 3rd clock is a 32nd; when lost clocks jump over one, the next clock still sends
   the step it belongs to */
void esplink_midi_tick(uint32_t x) {
    const uint32_t t = now();
    const int fresh = t - g.midi_at > MIDI_CLOCK_GONE;
    g.midi_at = t;
    g.playing = !g.stopped; /* clock without a Start (it was running before we booted): playing */
    if (fresh || x == 0 || x / 3 != g.last_x / 3)
        tick(x / 3, 1);
    g.last_x = x;
}

void esplink_transport(int start) {
    g.stopped = !start;
    g.playing = start;
    send1(start ? EV_START : EV_STOP, 0, 0, 0, 0);
}

void esplink_base_buf(int buf) {
    g.base_buf = buf + 1;
    send1(EV_LOAD_BUF, 0, 0, (uint8_t)buf, 0);
}

void esplink_owner(int slot, int cutter) {
    send1(EV_OWNER, 0, (uint8_t)slot, (uint8_t)cutter, slot == 0 ? (int32_t)slot_sixteenths(0) : 0);
}

void esplink_forward_midi(uint8_t st, uint8_t d1, uint8_t d2) {
    const uint8_t chan = st & 0xF;
    switch (st & 0xF0) {
    case 0x90: send1(d2 ? EV_NOTE_ON : EV_NOTE_OFF, chan, d1, d2, 0); break;
    case 0x80: send1(EV_NOTE_OFF, chan, d1, d2, 0); break;
    case 0xB0: send1(EV_CC, chan, d1, d2, 0); break;
    case 0xC0: send1(EV_PC, chan, d1, 0, 0); break;
    case 0xE0: send1(EV_BEND, chan, 0, 0, d1 | (d2 << 7)); break;
    default: break;
    }
}

void esplink_status(void) {
    printf("esp: link %s", g.path_up[ESP] ? "up" : "down (no hello from the ESP)");
    if (g.sim)
        printf(", PC stand-in on the console %s", g.path_up[SIM] ? "up" : "waiting for its hello");
    if (g.up)
        printf(" (protocol %lu)", (unsigned long)g.version);
    printf("  rx %lu frames / %lu bad, %lu events dropped (voice 0 is MIDI's or transport)\n",
           (unsigned long)g_rx.frames, (unsigned long)g_rx.crc_errors, (unsigned long)g.dropped);
    printf("     tx %lu frames / %lu cut short, ticks %lu (step %lu, %s, %s)  ring drops %lu  voice 0: %s\n",
           (unsigned long)g.tx_frames, (unsigned long)g.tx_drops, (unsigned long)g.ticks, (unsigned long)g.step,
           now() - g.midi_at > MIDI_CLOCK_GONE ? "engine beat" : "MIDI clock", g.playing ? "playing" : "stopped",
           (unsigned long)g.ring_drops, g_md_slot0_external ? "cutter" : "MIDI");
}

#else
void esplink_poll(void) {}
void esplink_drain(void) {}
void esplink_midi_tick(uint32_t x) { (void)x; }
void esplink_transport(int start) { (void)start; }
void esplink_owner(int slot, int cutter) { (void)slot; (void)cutter; }
void esplink_base_buf(int buf) { (void)buf; }
void esplink_forward_midi(uint8_t st, uint8_t d1, uint8_t d2) { (void)st; (void)d1; (void)d2; }
void esplink_status(void) { printf("esp: no ESP UART in this gateware\n"); }
int esplink_sim(void) { return 0; }
void esplink_sim_event(const struct smp_event* e) { mapping_event(e); }
void esplink_set_sim(int on) { (void)on; }
#endif
