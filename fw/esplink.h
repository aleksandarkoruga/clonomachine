/* The on-board ESP32 (the break cutter's brain) on its own UART, 3 Mbaud: both ways the SLIP +
 * CRC-8 event frames of include/sampler_events.h. See fw/esplink.c. */
#ifndef ESPLINK_H
#define ESPLINK_H

#include <stdint.h>

struct smp_event;

void esplink_poll(void);                    /* main loop: receive, link timeout, engine ticks */
void esplink_drain(void);                   /* inside SD reads: buffer received bytes only    */
void esplink_midi_tick(uint32_t x);         /* MIDI clock index since start (lost ticks in)   */
void esplink_transport(int start);          /* start / stop, from any source                  */
void esplink_owner(int slot, int cutter);   /* who plays the slot: the cutter or MIDI         */
void esplink_forward_midi(uint8_t st, uint8_t d1, uint8_t d2); /* the cutter's MIDI channel */
void esplink_status(void);                  /* console "esp"                                  */
int esplink_sim(void);                      /* "esp sim 1": the PC stands in for the ESP         */
void esplink_set_sim(int on);
void esplink_sim_event(const struct smp_event* e); /* console frames while the PC stands in   */

#endif
