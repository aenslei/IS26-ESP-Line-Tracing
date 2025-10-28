#include "ultrasonic.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <math.h>
#include <stdio.h>

static uint trig_pin, echo_pin;

void ultrasonic_init(uint trig, uint echo) {
    trig_pin = trig;
    echo_pin = echo;

    gpio_init(trig_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_put(trig_pin, 0);

    gpio_init(echo_pin);
    gpio_set_dir(echo_pin, GPIO_IN);
}

// --- Basic raw distance measurement (in cm) ---
float ultrasonic_get_distance_cm(void) {
    // Send 10 µs trigger pulse
    gpio_put(trig_pin, 1);
    sleep_us(10);
    gpio_put(trig_pin, 0);

    // Wait for echo to start
    absolute_time_t start = get_absolute_time();
    while (!gpio_get(echo_pin)) {
        if (absolute_time_diff_us(start, get_absolute_time()) > 30000)
            return -1.0f;
    }

    absolute_time_t echo_start = get_absolute_time();

    // Wait for echo to end
    while (gpio_get(echo_pin)) {
        if (absolute_time_diff_us(echo_start, get_absolute_time()) > 30000)
            return -1.0f;
    }

    absolute_time_t echo_end = get_absolute_time();

    int64_t pulse_len = absolute_time_diff_us(echo_start, echo_end);
    float distance_cm = (pulse_len / 2.0f) / 29.1f;
    return distance_cm;
}

// --- Median-of-5 measurement with outlier rejection ---
float ultrasonic_get_distance_med5(void) {
    float samples[5];

    // Collect samples
    for (int i = 0; i < 5; i++) {
        samples[i] = ultrasonic_get_distance_cm();
        sleep_ms(10);
    }

    // Sort (simple insertion sort)
    for (int i = 1; i < 5; i++) {
        float key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }

    // Compute median
    float median = samples[2];

    // Optional: reject outliers that deviate >1.5× median
    for (int i = 0; i < 5; i++) {
        if (samples[i] > median * 1.5f || samples[i] < median * 0.5f)
            samples[i] = median; // replace with median
    }

    // Average remaining (to smooth)
    float sum = 0.0f;
    for (int i = 0; i < 5; i++) sum += samples[i];
    return sum / 5.0f;
}

// --- Validity check (0–400 cm typical for HC-SR04) ---
bool ultrasonic_is_valid(float cm) {
    return (cm > 0.0f && cm < 400.0f);
}
