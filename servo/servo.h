#ifndef SERVO_H
#define SERVO_H

#include "pico/stdlib.h"

// Initialize the servo on specified GPIO
void servo_init(uint pin);

// Instantly set absolute angle (0–180°)
void servo_set_absolute(uint8_t angle);

// Instantly set relative angle from center (0° = center, - = left, + = right)
void servo_set_relative(int8_t relative_angle);

// Smoothly move between two relative angles (degrees from center)
void servo_smooth_move_relative(int8_t from_rel, int8_t to_rel, uint8_t step, uint delay_ms);

#endif
