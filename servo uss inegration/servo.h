#ifndef SERVO_H
#define SERVO_H

#include "pico/stdlib.h"

// Initialize PWM for servo on specified GPIO pin
void servo_init(uint pin);

// Move servo to target angle (0–180°)
void servo_set_angle(uint8_t angle);

// Convert angle to equivalent pulse width (1ms–2ms)
uint16_t servo_angle_to_us(uint8_t angle);

// Smooth transition from one angle to another
// step_size: angle increment (e.g., 2–5°)
// delay_ms: pause between steps (e.g., 10–20 ms)
void servo_smooth_move(uint8_t from_angle, uint8_t to_angle, uint8_t step_size, uint delay_ms);

#endif
