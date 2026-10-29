/* MicroPython module "cutter": the break cutter (esp/cutter/cutter.c) on the ULX3S's on-board
 * ESP32, talking to the FPGA (fw/esplink.c) at 3 Mbaud in the SLIP + CRC-8 event frames of
 * include/sampler_events.h.
 *
 * The ESP's UART0 pins (GPIO1 TX -> FPGA K4, GPIO3 RX <- FPGA K3) are the FPGA link. MicroPython's
 * REPL starts on them (115200, reachable through host/esp32/passthru.v); cutter.start() moves the
 * pins to UART1 at 3 Mbaud for the link and cuts UART0's RX off (constant high), so frames never
 * reach the REPL. cutter.stop() gives the pins back to UART0.
 *
 * A task on core 1 (above MicroPython's) does all the cutting: it wakes on received bytes, on a
 * one-shot esp_timer when a delayed frame (ratchet, tape stop) is due, and twice a second to send
 * the hello. The Python calls take the same mutex.
 */
#include <string.h>

#include "py/runtime.h"

#include "driver/uart.h"
#include "esp_random.h"
#include "esp_task.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h" /* xTaskCreatePinnedToCore */
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/gpio_sig_map.h"

#include "cutter.h"

#define LINK_UART UART_NUM_1
#define LINK_BAUD 3000000
#define PIN_TX 1           /* ESP U0TXD pin -> FPGA K4 */
#define PIN_RX 3           /* ESP U0RXD pin <- FPGA K3 */
#define CONST_HIGH_IN 0x38 /* GPIO matrix: constant 1 as an input */
#define MARK_TIMER 1       /* our events in the UART event queue (type UART_EVENT_MAX) */
#define MARK_HELLO 2

static struct cutter g_c;
static int g_inited, g_running;
static SemaphoreHandle_t g_mutex;
static QueueHandle_t g_q;
static esp_timer_handle_t g_due, g_hello;
static TaskHandle_t g_task;
static struct {
    uint8_t buf[8 * 16 + 1];
    unsigned len;
    int esc, in_frame;
} g_rx;
static struct {
    uint32_t rx_frames, rx_bad, tx_frames, ticks;
    int64_t last_rx_us;
} g_st;

static double now_s(void) { return (double)esp_timer_get_time() / 1e6; }

/* ---- link ---------------------------------------------------------------------------------- */
static void send(const struct smp_event* ev, int n) {
    uint8_t raw[8 * CUT_MAX_EV + 1], out[2 * sizeof(raw) + 2];
    memcpy(raw, ev, (size_t)n * 8);
    raw[8 * n] = smp_crc8(raw, (unsigned)(8 * n));
    unsigned k = 0;
    out[k++] = 0xC0;
    for (int i = 0; i <= 8 * n; ++i) {
        const uint8_t c = raw[i];
        if (c == 0xC0) {
            out[k++] = 0xDB;
            out[k++] = 0xDC;
        } else if (c == 0xDB) {
            out[k++] = 0xDB;
            out[k++] = 0xDD;
        } else
            out[k++] = c;
    }
    out[k++] = 0xC0;
    uart_write_bytes(LINK_UART, out, k);
    ++g_st.tx_frames;
}

static void hello(void) {
    const struct smp_event e = {EV_HELLO, 0, 'E', 'S', 1};
    send(&e, 1);
}

static void frame_in(void) {
    const unsigned n = g_rx.len;
    if (n < 9 || (n - 1) % 8 || smp_crc8(g_rx.buf, n - 1) != g_rx.buf[n - 1]) {
        ++g_st.rx_bad;
        return;
    }
    ++g_st.rx_frames;
    g_st.last_rx_us = esp_timer_get_time();
    cut_clock(&g_c, now_s());
    for (unsigned i = 0; i + 8 < n; i += 8) {
        struct smp_event e;
        memcpy(&e, g_rx.buf + i, 8);
        if (e.type == EV_TICK)
            ++g_st.ticks;
        cut_rx(&g_c, &e);
    }
}

static void byte_in(uint8_t c) {
    if (c == 0xC0) {
        if (g_rx.in_frame && g_rx.len)
            frame_in();
        g_rx.in_frame = 1; /* an END opens the next frame too */
        g_rx.len = 0;
        g_rx.esc = 0;
        return;
    }
    if (!g_rx.in_frame)
        return;
    if (g_rx.esc) {
        g_rx.esc = 0;
        c = c == 0xDC ? 0xC0 : c == 0xDD ? 0xDB : c;
    } else if (c == 0xDB) {
        g_rx.esc = 1;
        return;
    }
    if (g_rx.len < sizeof(g_rx.buf))
        g_rx.buf[g_rx.len++] = c;
    else
        g_rx.in_frame = 0; /* too long: drop it */
}

/* send what is due, arm the timer for the next one */
static void flush_due(void) {
    struct cut_frame f;
    const double t = now_s();
    while (cut_pop(&g_c, t, &f))
        send(f.ev, f.n);
    const double next = cut_next_due(&g_c);
    esp_timer_stop(g_due);
    if (next >= 0) {
        const double us = (next - t) * 1e6;
        esp_timer_start_once(g_due, us < 50 ? 50 : (uint64_t)us);
    }
}

static void mark(void* arg) {
    uart_event_t e = {.type = UART_EVENT_MAX, .size = (size_t)(uintptr_t)arg};
    xQueueSend(g_q, &e, 0);
}

static void link_task(void* arg) {
    (void)arg;
    uint8_t buf[256];
    uart_event_t ev;
    while (g_running) {
        if (!xQueueReceive(g_q, &ev, pdMS_TO_TICKS(100)))
            continue;
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        switch (ev.type) {
        case UART_DATA:
            for (;;) {
                const int n = uart_read_bytes(LINK_UART, buf, sizeof(buf), 0);
                if (n <= 0)
                    break;
                for (int i = 0; i < n; ++i)
                    byte_in(buf[i]);
            }
            break;
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            uart_flush_input(LINK_UART);
            g_rx.in_frame = 0;
            break;
        case UART_EVENT_MAX:
            if (ev.size == MARK_HELLO)
                hello();
            break;
        default:
            break;
        }
        flush_due();
        xSemaphoreGive(g_mutex);
    }
    g_task = NULL;
    vTaskDelete(NULL);
}

static void ensure_init(void) {
    if (g_inited)
        return;
    cut_init(&g_c, esp_random());
    g_mutex = xSemaphoreCreateMutex();
    const esp_timer_create_args_t due = {.callback = mark, .arg = (void*)MARK_TIMER, .name = "cut_due"};
    const esp_timer_create_args_t hel = {.callback = mark, .arg = (void*)MARK_HELLO, .name = "cut_hello"};
    esp_timer_create(&due, &g_due);
    esp_timer_create(&hel, &g_hello);
    g_inited = 1;
}

/* ---- Python ---------------------------------------------------------------------------------- */
static mp_obj_t cutter_start(void) {
    ensure_init();
    if (g_running)
        return mp_const_none;
    const uart_config_t cfg = {
        .baud_rate = LINK_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_driver_install(LINK_UART, 4096, 4096, 32, &g_q, 0) != ESP_OK)
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("UART1 driver"));
    uart_param_config(LINK_UART, &cfg);
    uart_set_pin(LINK_UART, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    esp_rom_gpio_connect_in_signal(CONST_HIGH_IN, U0RXD_IN_IDX, false); /* the REPL hears nothing */
    memset(&g_rx, 0, sizeof(g_rx));
    g_running = 1;
    xTaskCreatePinnedToCore(link_task, "cutter", 4096, NULL, ESP_TASK_PRIO_MIN + 5, &g_task, 1);
    esp_timer_start_periodic(g_hello, 500000);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(cutter_start_obj, cutter_start);

static mp_obj_t cutter_stop(void) {
    if (!g_running)
        return mp_const_none;
    esp_timer_stop(g_hello);
    esp_timer_stop(g_due);
    g_running = 0;
    while (g_task)
        vTaskDelay(pdMS_TO_TICKS(10));
    uart_driver_delete(LINK_UART);
    /* the pins back to UART0: the REPL again */
    esp_rom_gpio_connect_out_signal(PIN_TX, U0TXD_OUT_IDX, false, false);
    esp_rom_gpio_connect_in_signal(PIN_RX, U0RXD_IN_IDX, false);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(cutter_stop_obj, cutter_stop);

static void lock(void) {
    ensure_init();
    xSemaphoreTake(g_mutex, portMAX_DELAY);
}
static void unlock(void) { xSemaphoreGive(g_mutex); }

static const char* const k_knob[CK_NUM] = {"energy", "density", "syncopation", "anchor",  "build",
                                            "character", "release", "memory", "surprise"};

static int knob_index(mp_obj_t k) {
    if (mp_obj_is_int(k))
        return mp_obj_get_int(k);
    const char* s = mp_obj_str_get_str(k);
    for (int i = 0; i < CK_NUM; ++i)
        if (!strcmp(s, k_knob[i]))
            return i;
    mp_raise_ValueError(MP_ERROR_TEXT("knob"));
}

/* knob(name) -> value, knob(name, value) sets it */
static mp_obj_t cutter_knob(size_t n, const mp_obj_t* args) {
    const int k = knob_index(args[0]);
    if (k < 0 || k >= CK_NUM)
        mp_raise_ValueError(MP_ERROR_TEXT("knob"));
    lock();
    if (n > 1)
        cut_set_knob(&g_c, k, mp_obj_get_float(args[1]));
    const float v = g_c.p.knob[k];
    unlock();
    return mp_obj_new_float(v);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cutter_knob_obj, 1, 2, cutter_knob);

static mp_obj_t cutter_preset(size_t n, const mp_obj_t* args) {
    lock();
    if (n > 0)
        cut_preset(&g_c, mp_obj_get_int(args[0]));
    const int p = g_c.preset;
    unlock();
    return mp_obj_new_str(cut_preset_name(p), strlen(cut_preset_name(p)));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cutter_preset_obj, 0, 1, cutter_preset);

static mp_obj_t cutter_conductor(size_t n, const mp_obj_t* args) {
    lock();
    if (n > 0)
        cut_set_conductor(&g_c, mp_obj_is_true(args[0]));
    const int on = g_c.conductor;
    unlock();
    return mp_obj_new_bool(on);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cutter_conductor_obj, 0, 1, cutter_conductor);

static mp_obj_t cutter_phrase(mp_obj_t bars, mp_obj_t sections) {
    lock();
    cut_set_phrase(&g_c, mp_obj_get_int(bars), mp_obj_get_int(sections));
    unlock();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(cutter_phrase_obj, cutter_phrase);

/* status() -> dict */
static mp_obj_t cutter_status(void) {
    lock();
    mp_obj_t d = mp_obj_new_dict(0);
#define PUT(k, v) mp_obj_dict_store(d, mp_obj_new_str(k, strlen(k)), v)
    PUT("running", mp_obj_new_bool(g_running));
    PUT("link", mp_obj_new_bool(g_st.last_rx_us && esp_timer_get_time() - g_st.last_rx_us < 2000000));
    PUT("rx", mp_obj_new_int(g_st.rx_frames));
    PUT("rx_bad", mp_obj_new_int(g_st.rx_bad));
    PUT("tx", mp_obj_new_int(g_st.tx_frames));
    PUT("ticks", mp_obj_new_int(g_st.ticks));
    PUT("step", mp_obj_new_int(g_c.step));
    PUT("bpm", mp_obj_new_float(g_c.bpm));
    PUT("owner", mp_obj_new_bool(g_c.owner));
    PUT("playing", mp_obj_new_bool(g_c.playing));
    PUT("n16", mp_obj_new_int(g_c.n16));
    PUT("preset", mp_obj_new_str(cut_preset_name(g_c.preset), strlen(cut_preset_name(g_c.preset))));
    PUT("conductor", mp_obj_new_bool(g_c.conductor));
    PUT("bars", mp_obj_new_int(g_c.e.phrase_bars));
    mp_obj_t moves = mp_obj_new_dict(0);
    for (int m = 0; m < CUT_NMOVES; ++m)
        if (g_c.stats_hits[m])
            mp_obj_dict_store(moves, mp_obj_new_str(cut_move_name(m), strlen(cut_move_name(m))),
                              mp_obj_new_int(g_c.stats_hits[m]));
    PUT("moves", moves);
#undef PUT
    unlock();
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(cutter_status_obj, cutter_status);

static mp_obj_t cutter_knobs(void) {
    mp_obj_t l = mp_obj_new_list(0, NULL);
    for (int i = 0; i < CK_NUM; ++i)
        mp_obj_list_append(l, mp_obj_new_str(k_knob[i], strlen(k_knob[i])));
    return l;
}
static MP_DEFINE_CONST_FUN_OBJ_0(cutter_knobs_obj, cutter_knobs);

static const mp_rom_map_elem_t cutter_globals[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_cutter)},
    {MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&cutter_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&cutter_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_knob), MP_ROM_PTR(&cutter_knob_obj)},
    {MP_ROM_QSTR(MP_QSTR_knobs), MP_ROM_PTR(&cutter_knobs_obj)},
    {MP_ROM_QSTR(MP_QSTR_preset), MP_ROM_PTR(&cutter_preset_obj)},
    {MP_ROM_QSTR(MP_QSTR_conductor), MP_ROM_PTR(&cutter_conductor_obj)},
    {MP_ROM_QSTR(MP_QSTR_phrase), MP_ROM_PTR(&cutter_phrase_obj)},
    {MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&cutter_status_obj)},
};
static MP_DEFINE_CONST_DICT(cutter_globals_dict, cutter_globals);

const mp_obj_module_t cutter_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t*)&cutter_globals_dict,
};
MP_REGISTER_MODULE(MP_QSTR_cutter, cutter_module);
