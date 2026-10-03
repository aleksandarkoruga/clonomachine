/* CSR access to the sampler engine (soc/sampler.py). */
#ifndef ENGINE_H
#define ENGINE_H

#include <stdint.h>
#include <generated/csr.h>
#include <generated/soc.h>
#include "../include/sampler_regs.h"

#ifndef AUDIO_CLK_FREQ
#define AUDIO_CLK_FREQ 49152000
#endif
#define ENGINE_CYCLES_PER_SAMPLE 1024

static inline void eng_write(unsigned addr, uint32_t v) {
    while (!sampler_cmd_ready_read())
        ;
    sampler_cmd_data_write(v);
    sampler_cmd_addr_write(addr);
}

static inline void eng_slot_write(int slot, int reg, uint32_t v) { eng_write(SMP_ADDR(slot, reg), v); }

static inline uint32_t eng_status(unsigned idx) {
    sampler_stat_addr_write(idx);
    sampler_stat_addr_read(); /* one CSR round trip covers the RAM read latency */
    return sampler_stat_data_read();
}

/* Engine sample rate as Q16.16 Hz (48014.32 Hz for the 49.1667 MHz PLL). */
static inline uint32_t eng_fs_q16(void) {
    return (uint32_t)(((uint64_t)AUDIO_CLK_FREQ << 16) / ENGINE_CYCLES_PER_SAMPLE);
}

#endif
