/* Console UART driver: LiteX libbase uart.c with large rings.
 *
 * Linked ahead of libbase, so these symbols replace libbase's. The stock 128-byte RX
 * ring drops bytes after ~1 ms at 1 Mbaud whenever the main loop is busy (SD loading);
 * here the ISR moves the hardware FIFO into an 8 KB ring (~80 ms at 1 Mbaud), and a
 * 1 KB TX ring keeps printf from stalling the loop. */
#include <libbase/uart.h>

#include <generated/csr.h>
#include <irq.h>

#ifdef CSR_UART_BASE

#define RX_SIZE 8192
#define TX_SIZE 1024

static char rx_buf[RX_SIZE];
static volatile unsigned int rx_produce;
static unsigned int rx_consume;
static char tx_buf[TX_SIZE];
static unsigned int tx_produce;
static volatile unsigned int tx_consume;
volatile unsigned int uart_rx_overflows;

void uart_isr(void) {
    const unsigned int stat = uart_ev_pending_read() & uart_ev_enable_read();
    if (stat & UART_EV_RX) {
        while (!uart_rxempty_read()) {
            const char c = uart_rxtx_read();
            const unsigned int next = (rx_produce + 1) & (RX_SIZE - 1);
            if (next != rx_consume) {
                rx_buf[rx_produce] = c;
                rx_produce = next;
            } else
                ++uart_rx_overflows;
#ifndef CONFIG_UART_RX_FIFO_RX_WE
            uart_ev_pending_write(UART_EV_RX);
#endif
        }
    }
    if (stat & UART_EV_TX) {
        while (tx_consume != tx_produce && !uart_txfull_read()) {
            uart_rxtx_write(tx_buf[tx_consume]);
            tx_consume = (tx_consume + 1) & (TX_SIZE - 1);
        }
        if (tx_consume == tx_produce)
            uart_ev_enable_write(UART_EV_RX); /* TX interrupt off */
    }
}

char uart_read(void) {
    if (irq_getie())
        while (rx_consume == rx_produce)
            ;
    else
        while (rx_consume == rx_produce)
            uart_isr();
    const char c = rx_buf[rx_consume];
    rx_consume = (rx_consume + 1) & (RX_SIZE - 1);
    return c;
}

int uart_read_nonblock(void) {
    if (!irq_getie() && rx_consume == rx_produce)
        uart_isr();
    return rx_consume != rx_produce;
}

void uart_write(char c) {
    const unsigned int next = (tx_produce + 1) & (TX_SIZE - 1);
    if (irq_getie())
        while (next == tx_consume)
            ;
    else
        while (next == tx_consume)
            uart_isr();
    const unsigned int oldmask = irq_getmask();
    irq_setmask(oldmask & ~(1 << UART_INTERRUPT));
    if (tx_consume != tx_produce || uart_txfull_read()) {
        tx_buf[tx_produce] = c;
        tx_produce = next;
        uart_ev_enable_write(UART_EV_TX | UART_EV_RX);
    } else
        uart_rxtx_write(c);
    irq_setmask(oldmask);
}

void uart_init(void) {
    rx_produce = rx_consume = 0;
    tx_produce = tx_consume = 0;
    uart_ev_enable_write(UART_EV_RX);
    if (irq_attach)
        irq_attach(UART_INTERRUPT, uart_isr);
    irq_setmask(irq_getmask() | (1 << UART_INTERRUPT));
}

void uart_sync(void) {
    while (tx_consume != tx_produce)
        if (!irq_getie())
            uart_isr();
}

#endif
