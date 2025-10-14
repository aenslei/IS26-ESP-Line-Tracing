
#ifndef ULTRASONIC_INTERRUPT_H
#define ULTRASONIC_INTERRUPT_H

#include "pico/stdlib.h"

void ultrasonic_init(uint trig_pin, uint echo_pin);
bool ultrasonic_trigger();
bool ultrasonic_read_cm(uint16_t* distance_cm);

#endif
