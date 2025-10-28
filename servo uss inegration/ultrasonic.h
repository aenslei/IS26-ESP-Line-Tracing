#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "pico/stdlib.h"
#include <stdbool.h>

// Initialize pins
void ultrasonic_init(uint trig_pin, uint echo_pin);

// Get single-shot distance (no filter)
float ultrasonic_get_distance_cm(void);

// Get median-filtered distance (5 samples)
float ultrasonic_get_distance_med5(void);

// Validate if reading is within realistic range
bool ultrasonic_is_valid(float cm);

#endif
