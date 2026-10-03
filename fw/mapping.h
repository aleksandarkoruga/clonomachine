#ifndef MAPPING_H
#define MAPPING_H

#include "../include/sampler_events.h"

void mapping_init(void);                       /* builds the CC/bend LUTs (boot) */
void mapping_event(const struct smp_event* e);

#endif
