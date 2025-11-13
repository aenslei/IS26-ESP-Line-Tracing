#include "ultrasonic.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <math.h>
#include <stdio.h>

static uint trig_pin, echo_pin;
static float prev_valid_distance_ = 0.0f;

// ---------------- Constants (Maker Pi Pico + HC-SR04 tuned) ----------------
#define MAX_DISTANCE_CM        350.0f        // up to ~3.5 m reliable range
#define SOUND_SPEED_CM_PER_US  0.0343f
#define TIMEOUT_US             (MAX_DISTANCE_CM * 2.0f / SOUND_SPEED_CM_PER_US) // ≈20.4 ms
#define SAMPLE_DELAY_MS        10
#define SAMPLE_COUNT           5

// --------------------------------------------------------------------------
void ultrasonic_init(uint trig, uint echo) {
    trig_pin = trig;
    echo_pin = echo;

    gpio_init(trig_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_put(trig_pin, 0);

    gpio_init(echo_pin);
    gpio_set_dir(echo_pin, GPIO_IN);
}

/**
 * Perform one ultrasonic pulse measurement.
 * Returns distance in cm, or 0.0f if timeout.
 */
static float ultrasonic_read_once(void) {
    absolute_time_t start, end;

    // precise 10 µs trigger pulse
    gpio_put(trig_pin, 1);
    busy_wait_us_32(10);
    gpio_put(trig_pin, 0);

    // wait for echo start
    start = get_absolute_time();
    while (gpio_get(echo_pin) == 0) {
        if (absolute_time_diff_us(start, get_absolute_time()) > TIMEOUT_US)
            return 0.0f;
        tight_loop_contents();
    }

    // measure echo duration
    start = get_absolute_time();
    while (gpio_get(echo_pin) == 1) {
        if (absolute_time_diff_us(start, get_absolute_time()) > TIMEOUT_US)
            return 0.0f;
        tight_loop_contents();
    }
    end = get_absolute_time();

    uint64_t pulse_us = absolute_time_diff_us(start, end);
    return (pulse_us * SOUND_SPEED_CM_PER_US) / 2.0f;
}

/**
 * Take multiple readings, apply median filter + adaptive outlier rejection.
 */
float ultrasonic_get_distance_med5(void) {
    float samples[SAMPLE_COUNT];
    int valid = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        float d = ultrasonic_read_once();

        if (d > 2.0f && d < MAX_DISTANCE_CM) {
            samples[valid++] = d;
        }

        sleep_ms(SAMPLE_DELAY_MS);
    }

    if (valid == 0)
        return 0.0f;

    // sort valid samples (insertion sort)
    for (int i = 1; i < valid; i++) {
        float key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }

    // compute median
    float median = samples[valid / 2];

    // compute average absolute deviation
    float dev_sum = 0.0f;
    for (int i = 0; i < valid; i++)
        dev_sum += fabsf(samples[i] - median);
    float avg_dev = dev_sum / valid;

    // --- Adaptive outlier rejection ---
    // if median differs greatly from previous but noise is low, confirm twice
    static int confirm_count = 0;
    if (prev_valid_distance_ > 0.0f &&
        fabsf(median - prev_valid_distance_) > 3 * avg_dev &&
        avg_dev < 20.0f) {

        confirm_count++;
        if (confirm_count < 2)
            return prev_valid_distance_;   // wait for second confirmation
        confirm_count = 0;
    } else {
        confirm_count = 0;
    }

    prev_valid_distance_ = median;
    return median;
}

/**
 * Simple validity check (0–400 cm typical)
 */
bool ultrasonic_is_valid(float cm) {
    return (cm > 0.0f && cm < 400.0f);
}
