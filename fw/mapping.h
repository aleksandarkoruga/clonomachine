#ifndef MAPPING_H
#define MAPPING_H

#include "../include/sampler_events.h"

void mapping_init(void);                       /* builds the CC/bend LUTs (boot) */
void mapping_event(const struct smp_event* e);

/* MIDI from the US2 USB host (fw/usbh.c): pages from /midimap.json (fw/midimap.h) drive the
   voices; other channels go through the Monomachine table of mapping_event(); MIDI clock
   sets the tempo (g_clock_follow). */
void mapping_midi(uint8_t status, uint8_t d1, uint8_t d2);
void mapping_midi_at(uint8_t status, uint8_t d1, uint8_t d2, uint32_t t); /* t: arrival (engine samples), for the clock */
void mapping_midi_status(void);
void mapping_midimap_load(void);     /* /midimap.json; written with the defaults if missing */
void mapping_midimap_print(void);
extern int g_clock_follow;           /* 1: MIDI clock sets the global bpm                 */
extern int g_md_slot0_external;      /* pitch bend 'cutter' below the centre: voice 0 is
                                        played from OSC (the SC cutter), MIDI notes ignored */

#endif
