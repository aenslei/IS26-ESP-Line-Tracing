/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

static char event_str[128];

void gpio_event_string(char *buf, uint32_t events);

void gpio_callback(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    gpio_event_string(event_str, events);
    printf("GPIO %d %s\n", gpio, event_str);
}

int main() {
    stdio_init_all();

    printf("Hello GPIO IRQ\n");
    gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Wait forever
    while (1);
}


static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

*/

/**
 * Wheel Encoder Driver Example
 * 
 * Counts edges from an encoder to measure distance
 * and calculates speed from pulse width (time between edges).
 *
 * Tested on Raspberry Pi Pico SDK.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#define GPIO_ENCODER_PIN 2
#define WHEEL_CIRCUMFERENCE_MM 210.0f   // Example wheel circumference (mm)
#define ENCODER_NOTCHES 26              // Number of notches (pulses) per revolution

#define GPIO_LED_PIN 3   // Onboard LED on Raspberry Pi Pico
// Distance per notch
#define DIST_PER_PULSE_MM (WHEEL_CIRCUMFERENCE_MM / ENCODER_NOTCHES)

static volatile uint32_t pulse_count = 0;       // total pulses detected
static volatile absolute_time_t last_pulse;     // time of last pulse
static volatile float last_speed_mm_per_s = 0;  // last computed speed

// Callback for GPIO interrupts
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == GPIO_ENCODER_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        pulse_count++;  // Count every rising edge (1 notch detected)

         // Toggle LED each pulse
        gpio_xor_mask(1u << GPIO_LED_PIN);

        // Get current time
        absolute_time_t now = get_absolute_time();

        if (!is_nil_time(last_pulse)) {
            // Time difference in microseconds
            int64_t dt_us = absolute_time_diff_us(last_pulse, now);
            if (dt_us > 0) {
                // Convert time per pulse → speed
                // speed = distance per pulse / time per pulse
                last_speed_mm_per_s = (DIST_PER_PULSE_MM / dt_us) * 1e6f;
            }
        }

        last_pulse = now;
    }
}

int main() {
    stdio_init_all();

    printf("Wheel Encoder Driver Starting...\n");

    // Init GPIO pin
    gpio_init(GPIO_ENCODER_PIN);
    gpio_set_dir(GPIO_ENCODER_PIN, false); // input
    gpio_pull_up(GPIO_ENCODER_PIN);        // pull-up if open collector encoder


    // Init LED output
    gpio_init(GPIO_LED_PIN);
    gpio_set_dir(GPIO_LED_PIN, true); // output
    gpio_put(GPIO_LED_PIN, 0);        // start OFF

    // Enable IRQ on rising edge, with callback
    gpio_set_irq_enabled_with_callback(GPIO_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    last_pulse = nil_time; // Initialize

    // Main loop: print distance and speed periodically
    while (1) {
        //sleep_ms(5000); // update once per second

        float distance_mm = pulse_count * DIST_PER_PULSE_MM;
        float time = distance_mm/last_speed_mm_per_s;

        printf("Pulses: %lu | Distance: %.2f mm | Speed: %.2f mm/s |time: %.2f\n",
               pulse_count, distance_mm, last_speed_mm_per_s, time);
    }
}
