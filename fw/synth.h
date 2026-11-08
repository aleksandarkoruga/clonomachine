/* The wavetable synth on the board (gateware with the synth: CSR sampler_synth_*): its tables in
 * the top 8 MB of the SDRAM (/wavetables/NN_name.wtl prepared on the PC by host/prepare_wt.sh, read
 * only; a few built-ins without them), the control (fw/synth_ctl.c) at its tick, MIDI, OSC, console.
 * Without the synth in the gateware these do nothing. */
#ifndef SYNTH_H
#define SYNTH_H

#include <stdint.h>

struct smp_event;
struct mm_synth;

void synth_init(void);              /* after the card is mounted: tables, registers */
void synth_service(void);           /* main loop: the control tick */
void synth_bpm(uint32_t bpm_milli);
/* MIDI on the synth's channel (the page from /midimap.json) */
void synth_midi(const struct mm_synth* pg, uint8_t status, uint8_t d1, uint8_t d2);
void synth_event(const struct smp_event* e); /* EV_SYN_NOTE, EV_SYN_PARAM */
void synth_console(int argc, char** argv);   /* console "synth ..." */
int synth_present(void);
void synth_esp_state(void);                  /* the harmony generator's parameters to the ESP */
/* the harmony generator's steps: a note-on on any channel (harmTrig's: a trig), the ESP link's
   32nd ticks (harmTrig 0: the pending offset's step), start / stop */
void synth_note_trig(int chan);
void synth_harm_tick(uint32_t step);
void synth_harm_transport(void);

#endif
