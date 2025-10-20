#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "pico/stdlib.h"

// Initialize ultrasonic trigger and echo pins
void ultrasonic_init(uint trig_pin, uint echo_pin);

// Measure distance in centimeters (returns 0.0 if timeout)
float ultrasonic_get_distance_cm(void);

#endif
