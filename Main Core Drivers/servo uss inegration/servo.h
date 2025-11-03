#ifndef SERVO_H
#define SERVO_H

#include "pico/stdlib.h"

// Initialize servo PWM on a specific pin
void servo_init(uint pin);

// Move servo to an absolute angle (0–180°)
void servo_set_absolute(uint8_t angle);

// Move servo relative to center = 0°
// left: negative (−), right: positive (+)
void servo_set_relative(int8_t relative_angle);

// Smoothly move between two relative angles
void servo_smooth_move_relative(int8_t from_rel, int8_t to_rel, uint8_t step, uint delay_ms);

#endif
