/* USB host on the ULX3S US2 port: one USB-MIDI device (Elektron TM-1 or any class-compliant
 * USB-MIDI interface) through the OHCI controller in the gateware. Polled from the main loop;
 * every MIDI message goes to mapping_midi() (fw/mapping.c). */
#ifndef USBH_H
#define USBH_H

#include <stdint.h>

void usbh_init(void);
void usbh_poll(void);            /* main loop / loader idle: attach, enumerate, MIDI in */
void usbh_status(void);          /* console: state, device, counters                   */
void usbh_descriptors(void);     /* console: dump the device's descriptors              */
void usbh_set_pull(int mode);    /* US2 resistors: 0 pull-downs (host), 1 released      */
void usbh_watch(int ms);         /* debug: time the device's connects / disconnects     */
void usbh_debug(void);           /* debug: controller registers, bulk-IN descriptors     */

extern int g_midi_monitor;       /* print every incoming MIDI message                    */

#endif
