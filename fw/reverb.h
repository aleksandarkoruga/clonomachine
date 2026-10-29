/* The plate reverb (rtl/reverb_unit.v) from the soft core: its control (fw/reverb_ctl.c) every
 * 256 samples with the rails' registers written, and the knobs (MIDI channel 8 / console "rev",
 * 0..127 each, the RK_* controls of fw/midimap.h). */
#ifndef REVERB_H
#define REVERB_H

void reverb_init(void);
void reverb_service(void);       /* main loop: a control block every 256 samples */
void reverb_knob(int k, int v);  /* RK_*, 0..127 */
int reverb_knob_value(int k);
void reverb_status(void);        /* console "rev" */

#endif
