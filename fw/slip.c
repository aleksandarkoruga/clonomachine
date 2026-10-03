#include "slip.h"

#include "../include/sampler_events.h"

int slip_feed(struct slip* d, uint8_t c) {
    if (c == SLIP_END) {
        const unsigned n = d->len;
        d->len = 0;
        d->esc = 0;
        if (!d->in_frame || n == 0) { /* opening END (or END END): start a frame */
            d->in_frame = 1;
            return 0;
        }
        d->in_frame = 0;              /* closing END: back to idle (text allowed) */
        if (d->overflow || n < 9 || (n - 1) % 8 != 0 || smp_crc8(d->buf, n - 1) != d->buf[n - 1]) {
            d->overflow = 0;
            ++d->crc_errors;
            return 0;
        }
        d->len = n;
        ++d->frames;
        return 1;
    }
    if (!d->in_frame)
        return 0;
    if (d->esc) {
        d->esc = 0;
        c = (c == SLIP_ESC_END) ? SLIP_END : (c == SLIP_ESC_ESC) ? SLIP_ESC : c;
    } else if (c == SLIP_ESC) {
        d->esc = 1;
        return 0;
    }
    if (d->len < sizeof(d->buf))
        d->buf[d->len++] = c;
    else
        d->overflow = 1;
    return 0;
}
