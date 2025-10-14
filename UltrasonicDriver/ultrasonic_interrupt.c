#include "ultrasonic_interrupt.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>
#include <stdlib.h>

static uint TRIG_PIN;
static uint ECHO_PIN;

static absolute_time_t echo_start, echo_end;
static volatile bool pulse_started = false;
static volatile bool pulse_done = false;

// --- Median filter buffer ---
#define SAMPLE_SIZE 5
static uint16_t distance_buffer[SAMPLE_SIZE];
static int buffer_index = 0;

// Interrupt handler for ECHO pin
void echo_callback(uint gpio, uint32_t events) {
    if (gpio != ECHO_PIN) return;

    if (events & GPIO_IRQ_EDGE_RISE) {
        echo_start = get_absolute_time();
        pulse_started = true;
    } else if ((events & GPIO_IRQ_EDGE_FALL) && pulse_started) {
        echo_end = get_absolute_time();
        pulse_done = true;
    }
}

void ultrasonic_init(uint trig_pin, uint echo_pin) {
    TRIG_PIN = trig_pin;
    ECHO_PIN = echo_pin;

    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    gpio_set_irq_enabled_with_callback(ECHO_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &echo_callback);

    // Initialize median buffer
    for (int i = 0; i < SAMPLE_SIZE; i++) distance_buffer[i] = 0;
}

bool ultrasonic_trigger() {
    if (pulse_done || pulse_started) return false;

    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);
    return true;
}

// Comparison function for qsort
static int compare_u16(const void *a, const void *b) {
    uint16_t ua = *(const uint16_t*)a;
    uint16_t ub = *(const uint16_t*)b;
    return (ua > ub) - (ua < ub);
}

bool ultrasonic_read_cm(uint16_t* distance_cm) {
    if (!pulse_done) return false;

    pulse_done = false;
    pulse_started = false;

    int64_t pulse_duration = absolute_time_diff_us(echo_start, echo_end);
    uint16_t new_reading = (uint16_t)(pulse_duration / 58);  // Convert to cm

    // Add to buffer
    distance_buffer[buffer_index] = new_reading;
    buffer_index = (buffer_index + 1) % SAMPLE_SIZE;

    // Copy and sort for median
    uint16_t sorted[SAMPLE_SIZE];
    for (int i = 0; i < SAMPLE_SIZE; i++) sorted[i] = distance_buffer[i];
    qsort(sorted, SAMPLE_SIZE, sizeof(uint16_t), compare_u16);

    // Return median
    *distance_cm = sorted[SAMPLE_SIZE / 2];
    return true;
}

