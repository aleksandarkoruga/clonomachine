/* SLIP + CRC-8 event framing (include/sampler_events.h). One decoder per link. */
#ifndef SLIP_H
#define SLIP_H

#include <stdint.h>

#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

struct slip {
    uint8_t buf[8 * 16 + 1];
    unsigned len;
    int esc, in_frame, overflow;
    uint32_t frames, crc_errors;
};

/* Feeds one byte. Returns 1 when a complete frame passed the CRC: events are then
 * buf[0 .. len-2] (len-1 bytes, a multiple of 8). */
int slip_feed(struct slip* d, uint8_t c);

#endif
