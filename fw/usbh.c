/* Minimal OHCI host for one USB-MIDI device, see usbh.h.
 *
 * The controller (SpinalHDL UsbOhci via LiteX usb_ohci, registers at USB_OHCI_CTRL_BASE)
 * walks endpoint / transfer descriptors in main RAM by DMA. The DMA goes through the same
 * L2 cache as the CPU and the CPU data cache is write-through, so the only care needed is
 * to invalidate the data cache before reading what the controller wrote.
 *
 * Attach -> 100 ms debounce -> port reset -> enumeration (address 1, configuration, the
 * MIDIStreaming interface's bulk IN endpoint) -> one 64-byte IN transfer always queued;
 * the controller retries NAKs by itself, usbh_poll() parses whatever came in and requeues.
 */
#include "usbh.h"

#include <stdio.h>
#include <string.h>

#include <generated/csr.h>
#include <generated/mem.h>
#include <system.h>

#include "../include/sampler_regs.h"
#include "engine.h"
#include "mapping.h"

int g_midi_monitor;

#ifdef USB_OHCI_CTRL_BASE

#define R(o) (*(volatile uint32_t*)(USB_OHCI_CTRL_BASE + (o)))
enum {
    HC_REVISION = 0x00, HC_CONTROL = 0x04, HC_CMDSTATUS = 0x08, HC_INTSTATUS = 0x0C, HC_INTDISABLE = 0x14,
    HC_HCCA = 0x18, HC_CTRL_HEAD = 0x20, HC_CTRL_CUR = 0x24, HC_BULK_HEAD = 0x28, HC_BULK_CUR = 0x2C,
    HC_FMINTERVAL = 0x34, HC_FMNUMBER = 0x3C, HC_PERIODICSTART = 0x40, HC_LSTHRESHOLD = 0x44,
    HC_RHDESCA = 0x48, HC_RHSTATUS = 0x50, HC_RHPORT1 = 0x54,
};
/* HcControl */
#define CTL_CBSR_4  3u
#define CTL_CLE     (1u << 4)
#define CTL_BLE     (1u << 5)
#define CTL_OPER    (2u << 6)
/* HcCommandStatus */
#define CMD_HCR     (1u << 0)
#define CMD_CLF     (1u << 1)
#define CMD_BLF     (1u << 2)
/* HcRhPortStatus */
#define PS_CCS      (1u << 0)
#define PS_PES      (1u << 1)
#define PS_PRS      (1u << 4)
#define PS_PPS      (1u << 8)
#define PS_LSDA     (1u << 9)
#define PS_CSC      (1u << 16)
#define PS_PESC     (1u << 17)
#define PS_PRSC     (1u << 20)
/* descriptors */
#define ED_SKIP     (1u << 14)
#define ED_LOWSPEED (1u << 13)
#define TD_ROUND    (1u << 18)
#define TD_SETUP    (0u << 19)
#define TD_OUT      (1u << 19)
#define TD_IN       (2u << 19)
#define TD_NOINT    (7u << 21)
#define TD_DATA0    (2u << 24)
#define TD_DATA1    (3u << 24)
#define TD_CC_NA    (15u << 28)
#define TD_CC(c)    ((c) >> 28)

struct ed { volatile uint32_t ctrl, tailp, headp, next; };
struct td { volatile uint32_t ctrl, cbp, next, be; };

static uint8_t g_hcca[256] __attribute__((aligned(256)));
static struct ed g_ed_ctrl __attribute__((aligned(16)));
static struct ed g_ed_bulk __attribute__((aligned(16)));
static struct td g_td[4] __attribute__((aligned(16)));      /* control: setup, data, status, dummy */
static struct td g_btd[2] __attribute__((aligned(16)));     /* bulk IN: active + dummy, swapped     */
static uint8_t g_setup[8] __attribute__((aligned(4)));
static uint8_t g_cbuf[512] __attribute__((aligned(4)));     /* control data (config descriptor)   */
static uint8_t g_bbuf[2][64] __attribute__((aligned(4)));   /* bulk IN buffers, one per TD          */

enum { ST_OFF, ST_IDLE, ST_DEBOUNCE, ST_RESET, ST_RUN, ST_FAIL };
static const char* const g_st_name[] = {"no controller", "waiting for a device", "debounce", "port reset",
                                        "running", "enumeration failed"};
static struct {
    int state, lowspeed, active;          /* active: index of the bulk TD the controller owns */
    uint32_t t0;                          /* state entry time, samples                         */
    uint16_t vid, pid, mps0, in_mps;
    uint8_t in_ep, out_ep, iface, cfg, cls;
    char product[40];
    uint32_t msgs, packets, errors, attaches, bounces;
    uint32_t cc_count[16];                /* bulk-IN errors per OHCI condition code */
    uint16_t cfg_len;
} g_u;

static inline uint32_t now_samples(void) { return eng_status(SMP_SG_SAMPLES); }
static inline uint32_t ms_since(uint32_t t) { return (now_samples() - t) / 48; }
static inline void mb(void) { __asm__ volatile("fence" ::: "memory"); }
static uint32_t ptr(const volatile void* p) { return (uint32_t)(uintptr_t)p; }

void usbh_set_pull(int mode) {
#ifdef CSR_USB_PULL_BASE
    /* bits: dp, dn levels, dp_oe, dn_oe. 0: both driven low (host pull-downs), 1: released,
       >= 2: the raw bits (debug, e.g. 5 = D+ resistor driven high like a device) */
    usb_pull_ctrl_write(mode >= 2 ? (uint32_t)mode & 0xFu : mode ? 0x0u : 0xCu);
#else
    (void)mode;
#endif
}

/* One control transfer on endpoint 0. dir_in: data stage IN. Returns bytes of data moved
   or -1 (the TD condition code is printed). */
static int control(uint8_t rtype, uint8_t req, uint16_t val, uint16_t idx, uint16_t len) {
    const int in = (rtype & 0x80) != 0;
    if (len > sizeof(g_cbuf))
        len = sizeof(g_cbuf);
    g_setup[0] = rtype;
    g_setup[1] = req;
    g_setup[2] = (uint8_t)val;
    g_setup[3] = (uint8_t)(val >> 8);
    g_setup[4] = (uint8_t)idx;
    g_setup[5] = (uint8_t)(idx >> 8);
    g_setup[6] = (uint8_t)len;
    g_setup[7] = (uint8_t)(len >> 8);
    struct td *t0 = &g_td[0], *t1 = &g_td[1], *t2 = &g_td[2], *t3 = &g_td[3];
    memset((void*)g_td, 0, sizeof(g_td));
    t0->ctrl = TD_CC_NA | TD_DATA0 | TD_SETUP | TD_NOINT;
    t0->cbp = ptr(g_setup);
    t0->be = ptr(g_setup) + 7;
    t0->next = ptr(len ? t1 : t2);
    if (len) {
        t1->ctrl = TD_CC_NA | TD_DATA1 | (in ? TD_IN : TD_OUT) | TD_NOINT | TD_ROUND;
        t1->cbp = ptr(g_cbuf);
        t1->be = ptr(g_cbuf) + len - 1;
        t1->next = ptr(t2);
    }
    t2->ctrl = TD_CC_NA | TD_DATA1 | ((in && len) ? TD_OUT : TD_IN) | TD_NOINT;
    t2->next = ptr(t3);
    g_ed_ctrl.tailp = ptr(t3);
    g_ed_ctrl.headp = ptr(t0);
    g_ed_ctrl.ctrl &= ~ED_SKIP;
    mb();
    R(HC_CMDSTATUS) = CMD_CLF;
    const uint32_t t = now_samples();
    for (;;) {
        flush_cpu_dcache();
        const uint32_t h = g_ed_ctrl.headp;
        if ((h & ~0xFu) == ptr(t3))
            break;
        if ((h & 1) || ms_since(t) > 500) {
            printf("USB: control %02x %02x failed (%s, cc %lu %lu %lu)\n", rtype, req, (h & 1) ? "halted" : "timeout",
                   (unsigned long)TD_CC(t0->ctrl), (unsigned long)TD_CC(t1->ctrl), (unsigned long)TD_CC(t2->ctrl));
            g_ed_ctrl.headp = ptr(t3);
            g_ed_ctrl.ctrl |= ED_SKIP;
            ++g_u.errors;
            return -1;
        }
    }
    g_ed_ctrl.ctrl |= ED_SKIP;
    if (TD_CC(t0->ctrl) || (len && TD_CC(t1->ctrl) && TD_CC(t1->ctrl) != 9) || TD_CC(t2->ctrl)) {
        printf("USB: control %02x %02x: cc %lu %lu %lu\n", rtype, req, (unsigned long)TD_CC(t0->ctrl),
               (unsigned long)TD_CC(t1->ctrl), (unsigned long)TD_CC(t2->ctrl));
        ++g_u.errors;
        return -1;
    }
    if (!len)
        return 0;
    return t1->cbp ? (int)(t1->cbp - ptr(g_cbuf)) : len;
}

static int get_desc(uint8_t type, uint8_t index, uint16_t len) {
    return control(0x80, 6, (uint16_t)((type << 8) | index), type == 3 ? 0x0409 : 0, len);
}

/* string descriptor -> ASCII */
static void get_string(uint8_t index, char* out, int n) {
    out[0] = 0;
    if (!index)
        return;
    const int got = get_desc(3, index, 64);
    const int lim = got < g_cbuf[0] ? got : g_cbuf[0];
    int k = 0;
    for (int i = 2; i + 1 < lim && k < n - 1; i += 2) /* UTF-16LE, ASCII kept */
        out[k++] = (g_cbuf[i + 1] || g_cbuf[i] < 0x20 || g_cbuf[i] > 0x7E) ? '?' : (char)g_cbuf[i];
    out[k] = 0;
}

/* Bulk IN: TD a is queued, b is the dummy at the tail. */
static void bulk_queue(int a) {
    struct td* t = &g_btd[a];
    struct td* d = &g_btd[a ^ 1];
    memset((void*)d, 0, sizeof(*d));
    t->ctrl = TD_CC_NA | TD_IN | TD_NOINT | TD_ROUND; /* toggle from the ED */
    t->cbp = ptr(g_bbuf[a]);
    t->be = ptr(g_bbuf[a]) + g_u.in_mps - 1;
    t->next = ptr(d);
    g_ed_bulk.tailp = ptr(d);
    mb();
    R(HC_CMDSTATUS) = CMD_BLF;
}

static int enumerate(void) {
    g_ed_ctrl.ctrl = ED_SKIP | (g_u.lowspeed ? ED_LOWSPEED : 0) | (8u << 16);   /* address 0, EP 0, mps 8 */
    if (get_desc(1, 0, 8) < 8)
        return -1;
    g_u.mps0 = g_cbuf[7] ? g_cbuf[7] : 8;
    if (control(0x00, 5, 1, 0, 0) < 0) /* SET_ADDRESS 1 */
        return -1;
    busy_wait(10);
    g_ed_ctrl.ctrl = ED_SKIP | (g_u.lowspeed ? ED_LOWSPEED : 0) | ((uint32_t)g_u.mps0 << 16) | 1u;
    if (get_desc(1, 0, 18) < 18)
        return -1;
    g_u.vid = (uint16_t)(g_cbuf[8] | (g_cbuf[9] << 8));
    g_u.pid = (uint16_t)(g_cbuf[10] | (g_cbuf[11] << 8));
    const uint8_t iproduct = g_cbuf[15];
    if (get_desc(2, 0, 9) < 9)
        return -1;
    uint16_t total = (uint16_t)(g_cbuf[2] | (g_cbuf[3] << 8));
    if (total > sizeof(g_cbuf))
        total = sizeof(g_cbuf);
    const int got = get_desc(2, 0, total);
    if (got < 9)
        return -1;
    g_u.cfg_len = (uint16_t)got;
    g_u.cfg = g_cbuf[5];
    /* the MIDIStreaming interface (audio class 1, subclass 3), else any interface with a
       bulk IN endpoint (vendor-specific MIDI interfaces use the same 4-byte packets) */
    uint8_t in_ep = 0, out_ep = 0, alt = 0;
    uint16_t in_mps = 64;
    for (int pass = 0; pass < 2 && !in_ep; ++pass) {
        int match = 0;
        for (int i = 0; i + 2 <= got && g_cbuf[i] >= 2 && !(in_ep && !match); i += g_cbuf[i]) {
            const uint8_t* d = &g_cbuf[i];
            if (d[1] == 4 && d[0] >= 9) { /* interface */
                if (in_ep)
                    break;           /* the matching interface ended */
                match = pass == 1 || (d[5] == 1 && d[6] == 3);
                out_ep = 0;
                g_u.iface = d[2];
                alt = d[3];
                g_u.cls = d[5];
            } else if (d[1] == 5 && d[0] >= 7 && match && (d[3] & 3) == 2) { /* bulk endpoint */
                if (d[2] & 0x80) {
                    if (!in_ep) {
                        in_ep = d[2];
                        in_mps = (uint16_t)((d[4] | (d[5] << 8)) & 0x7FF);
                    }
                } else if (!out_ep)
                    out_ep = d[2];
            }
        }
    }
    get_string(iproduct, g_u.product, sizeof(g_u.product));
    if (control(0x00, 9, g_u.cfg, 0, 0) < 0) /* SET_CONFIGURATION */
        return -1;
    if (alt && control(0x01, 11, alt, g_u.iface, 0) < 0) /* SET_INTERFACE */
        return -1;
    if (!in_ep) {
        printf("USB: %04x:%04x %s has no bulk IN endpoint (not a MIDI device?)\n", g_u.vid, g_u.pid, g_u.product);
        return -1;
    }
    g_u.in_ep = in_ep;
    g_u.out_ep = out_ep;
    g_u.in_mps = in_mps > 64 || !in_mps ? 64 : in_mps;
    g_ed_bulk.ctrl = (g_u.lowspeed ? ED_LOWSPEED : 0) | ((uint32_t)g_u.in_mps << 16) | ((uint32_t)(in_ep & 0xF) << 7) | 1u;
    memset((void*)g_btd, 0, sizeof(g_btd));
    g_ed_bulk.headp = ptr(&g_btd[0]); /* toggle carry 0 (DATA0) */
    g_u.active = 0;
    bulk_queue(0);
    printf("USB: %04x:%04x \"%s\", MIDI in on EP%u (%s interface %u, %u-byte packets)\n", g_u.vid, g_u.pid,
           g_u.product, in_ep & 0xF, g_u.cls == 1 ? "MIDIStreaming" : "vendor", g_u.iface, g_u.in_mps);
    return 0;
}

/* USB-MIDI event packets: [cable << 4 | CIN, MIDI bytes...] */
static void parse(const uint8_t* p, int n) {
    for (int i = 0; i + 4 <= n; i += 4, p += 4) {
        const uint8_t cin = p[0] & 0xF;
        if (!(p[0] | p[1] | p[2] | p[3]))
            continue; /* padding */
        uint8_t st = p[1], d1 = p[2], d2 = p[3];
        switch (cin) {
        case 0x8: case 0x9: case 0xA: case 0xB: case 0xE: break;
        case 0xC: case 0xD: d2 = 0; break;
        case 0xF: d1 = d2 = 0; break; /* single byte: clock, start, stop ... */
        default: continue;            /* sysex, system common */
        }
        ++g_u.msgs;
        if (g_midi_monitor && st != 0xF8)
            printf("[midi %02x %3u %3u  ch %u]\n", st, d1, d2, (st & 0xF) + 1);
        mapping_midi(st, d1, d2);
    }
}

void usbh_init(void) {
    memset(&g_u, 0, sizeof(g_u));
    usbh_set_pull(0);
    R(HC_CMDSTATUS) = CMD_HCR;
    for (int i = 0; i < 100 && (R(HC_CMDSTATUS) & CMD_HCR); ++i)
        busy_wait(1);
    if ((R(HC_REVISION) & 0xFF) != 0x10) {
        printf("USB: no OHCI controller (revision %08lx)\n", (unsigned long)R(HC_REVISION));
        g_u.state = ST_OFF;
        return;
    }
    memset(g_hcca, 0, sizeof(g_hcca));
    memset((void*)&g_ed_ctrl, 0, sizeof(g_ed_ctrl));
    memset((void*)&g_ed_bulk, 0, sizeof(g_ed_bulk));
    g_ed_ctrl.ctrl = ED_SKIP;
    g_ed_bulk.ctrl = ED_SKIP;
    mb();
    R(HC_HCCA) = ptr(g_hcca);
    R(HC_CTRL_HEAD) = ptr(&g_ed_ctrl);
    R(HC_BULK_HEAD) = ptr(&g_ed_bulk);
    R(HC_INTDISABLE) = 0xC000007Fu;
    R(HC_INTSTATUS) = 0x7Fu;
    const uint32_t fi = 11999;
    R(HC_FMINTERVAL) = ((((6 * (fi - 210)) / 7) & 0x7FFF) << 16) | fi;
    R(HC_PERIODICSTART) = fi * 9 / 10;
    R(HC_LSTHRESHOLD) = 0x628;
    R(HC_CONTROL) = CTL_OPER | CTL_CBSR_4 | CTL_CLE | CTL_BLE;
    R(HC_RHDESCA) = R(HC_RHDESCA) | (1u << 9); /* no power switching */
    R(HC_RHSTATUS) = 1u << 16;                 /* global power on */
    R(HC_RHPORT1) = PS_PPS;
    g_u.state = ST_IDLE;
    g_u.t0 = now_samples();
}

void usbh_poll(void) {
    if (g_u.state == ST_OFF)
        return;
    const uint32_t ps = R(HC_RHPORT1);
    if (g_u.state != ST_IDLE && !(ps & PS_CCS)) { /* unplugged */
        if (g_u.state == ST_DEBOUNCE)
            ++g_u.bounces; /* connected for less than the debounce time */
        else
            printf("USB: device unplugged\n");
        g_ed_bulk.ctrl |= ED_SKIP;
        R(HC_RHPORT1) = PS_CSC | PS_PESC | PS_PRSC;
        g_u.state = ST_IDLE;
        return;
    }
    switch (g_u.state) {
    case ST_IDLE:
        if (ps & PS_CCS) {
            R(HC_RHPORT1) = PS_CSC;
            g_u.state = ST_DEBOUNCE;
            g_u.t0 = now_samples();
            ++g_u.attaches;
        }
        break;
    case ST_DEBOUNCE:
        if (ms_since(g_u.t0) >= 100) {
            R(HC_RHPORT1) = PS_PRS;
            g_u.state = ST_RESET;
            g_u.t0 = now_samples();
        }
        break;
    case ST_RESET:
        if ((ps & PS_PRSC) && ms_since(g_u.t0) >= 20) {
            R(HC_RHPORT1) = PS_PRSC | PS_CSC | PS_PESC;
            g_u.lowspeed = (ps & PS_LSDA) != 0;
            busy_wait(10);
            if (enumerate() == 0)
                g_u.state = ST_RUN;
            else {
                g_u.state = ST_FAIL;
                g_u.t0 = now_samples();
            }
        } else if (ms_since(g_u.t0) > 500) {
            printf("USB: port reset did not finish (port status %08lx)\n", (unsigned long)ps);
            g_u.state = ST_FAIL;
            g_u.t0 = now_samples();
        }
        break;
    case ST_FAIL:
        if (ms_since(g_u.t0) > 3000) { /* try again */
            R(HC_RHPORT1) = PS_PRS;
            g_u.state = ST_RESET;
            g_u.t0 = now_samples();
        }
        break;
    case ST_RUN: {
        R(HC_INTSTATUS) = 2u; /* WritebackDoneHead: let the controller report again */
        flush_cpu_dcache();
        const uint32_t h = g_ed_bulk.headp;
        const struct td* t = &g_btd[g_u.active];
        if (h & 1) { /* halted: drop the TD, carry on */
            ++g_u.errors;
            ++g_u.cc_count[TD_CC(t->ctrl) & 15];
            g_ed_bulk.headp = ptr(&g_btd[g_u.active ^ 1]) | (h & 2);
            g_u.active ^= 1;
            bulk_queue(g_u.active);
            break;
        }
        if ((h & ~0xFu) != ptr(&g_btd[g_u.active ^ 1])) {
            R(HC_CMDSTATUS) = CMD_BLF; /* still pending (NAKs): keep the bulk list going */
            break;
        }
        const int a = g_u.active;
        const int n = t->cbp ? (int)(t->cbp - ptr(g_bbuf[a])) : g_u.in_mps;
        if (TD_CC(t->ctrl) == 0 || TD_CC(t->ctrl) == 9) {
            ++g_u.packets;
            parse(g_bbuf[a], n);
        } else {
            ++g_u.errors;
            ++g_u.cc_count[TD_CC(t->ctrl) & 15];
        }
        g_u.active ^= 1;
        bulk_queue(g_u.active);
        break;
    }
    default:
        break;
    }
}

void usbh_status(void) {
    const uint32_t ps = g_u.state == ST_OFF ? 0 : R(HC_RHPORT1);
    printf("usb: %s  port %08lx (%s%s)", g_st_name[g_u.state], (unsigned long)ps, (ps & PS_CCS) ? "connected" : "empty",
           (ps & PS_CCS) ? ((ps & PS_LSDA) ? ", low speed" : ", full speed") : "");
    if (g_u.state == ST_RUN)
        printf("  %04x:%04x \"%s\" EP%u", g_u.vid, g_u.pid, g_u.product, g_u.in_ep & 0xF);
    printf("\n     attaches %lu (bounced %lu)  packets %lu  midi messages %lu  errors %lu\n",
           (unsigned long)g_u.attaches, (unsigned long)g_u.bounces, (unsigned long)g_u.packets,
           (unsigned long)g_u.msgs, (unsigned long)g_u.errors);
    static const char* const cc_name[16] = {"ok", "CRC", "bit stuffing", "toggle mismatch", "stall",
        "no response", "PID check", "unexpected PID", "data overrun", "data underrun", "?", "?",
        "buffer overrun", "buffer underrun", "not accessed", "not accessed"};
    for (int c = 1; c < 16; ++c)
        if (g_u.cc_count[c])
            printf("     %lu x %s\n", (unsigned long)g_u.cc_count[c], cc_name[c]);
}

/* Debug: controller and bulk-IN descriptor state */
void usbh_debug(void) {
    if (g_u.state == ST_OFF)
        return;
    flush_cpu_dcache();
    printf("HcControl %08lx CmdStatus %08lx IntStatus %08lx FmNumber %lu BulkHead %08lx BulkCur %08lx\n",
           (unsigned long)R(HC_CONTROL), (unsigned long)R(HC_CMDSTATUS), (unsigned long)R(HC_INTSTATUS),
           (unsigned long)(R(HC_FMNUMBER) & 0xFFFF), (unsigned long)R(HC_BULK_HEAD), (unsigned long)R(HC_BULK_CUR));
    printf("bulk ED %08lx: ctrl %08lx tail %08lx head %08lx next %08lx\n", (unsigned long)ptr(&g_ed_bulk),
           (unsigned long)g_ed_bulk.ctrl, (unsigned long)g_ed_bulk.tailp, (unsigned long)g_ed_bulk.headp,
           (unsigned long)g_ed_bulk.next);
    for (int i = 0; i < 2; ++i)
        printf("TD%d %08lx: ctrl %08lx cbp %08lx next %08lx be %08lx%s\n", i, (unsigned long)ptr(&g_btd[i]),
               (unsigned long)g_btd[i].ctrl, (unsigned long)g_btd[i].cbp, (unsigned long)g_btd[i].next,
               (unsigned long)g_btd[i].be, i == g_u.active ? "  (active)" : "");
}

/* Debug: sample the port's connect bit for ms milliseconds, print how long each connection
   and each gap lasted (engine sample clock, 20.8 us). */
void usbh_watch(int ms) {
    if (g_u.state == ST_OFF)
        return;
    const uint32_t t0 = now_samples();
    uint32_t last = t0, n = 0;
    int cur = (R(HC_RHPORT1) & PS_CCS) != 0;
    printf("connect bit %d at start\n", cur);
    while (now_samples() - t0 < (uint32_t)ms * 48 && n < 40) {
        const int c = (R(HC_RHPORT1) & PS_CCS) != 0;
        if (c != cur) {
            const uint32_t t = now_samples();
            printf("  %s after %lu us\n", c ? "connect   " : "disconnect", (unsigned long)((t - last) * 125 / 6));
            R(HC_RHPORT1) = PS_CSC;
            last = t;
            cur = c;
            ++n;
        }
    }
}

void usbh_descriptors(void) {
    if (g_u.state != ST_RUN) {
        printf("usb: no device enumerated\n");
        return;
    }
    const int got = get_desc(2, 0, g_u.cfg_len);
    for (int i = 0; i + 2 <= got && g_cbuf[i] >= 2; i += g_cbuf[i]) {
        printf("  len %2u type %02x:", g_cbuf[i], g_cbuf[i + 1]);
        for (int k = 2; k < g_cbuf[i] && i + k < got; ++k)
            printf(" %02x", g_cbuf[i + k]);
        printf("\n");
    }
}

#else
void usbh_init(void) {}
void usbh_poll(void) {}
void usbh_status(void) { printf("usb: no USB host in this gateware\n"); }
void usbh_descriptors(void) {}
void usbh_set_pull(int mode) { (void)mode; }
void usbh_watch(int ms) { (void)ms; }
void usbh_debug(void) {}
#endif
