#include "ultrasonic.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <math.h>

static uint trig_pin_;
static uint echo_pin_;
static float prev_valid_distance_ = 0.0f;

// 2 m max detection distance
#define MAX_DISTANCE_CM 200.0f
#define SOUND_SPEED_CM_PER_US 0.0343f
#define TIMEOUT_US (MAX_DISTANCE_CM * 2.0f / SOUND_SPEED_CM_PER_US)  // ≈ 11660 µs

void ultrasonic_init(uint trig_pin, uint echo_pin) {
    trig_pin_ = trig_pin;
    echo_pin_ = echo_pin;

    gpio_init(trig_pin_);
    gpio_init(echo_pin_);
    gpio_set_dir(trig_pin_, GPIO_OUT);
    gpio_set_dir(echo_pin_, GPIO_IN);
    gpio_put(trig_pin_, 0);
}

static float ultrasonic_read_once(void) {
    absolute_time_t start, end;
    uint64_t pulse_time_us;

    gpio_put(trig_pin_, 1);
    sleep_us(10);
    gpio_put(trig_pin_, 0);

    start = get_absolute_time();
    while (gpio_get(echo_pin_) == 0) {
        if (absolute_time_diff_us(start, get_absolute_time()) > TIMEOUT_US)
            return 0.0f;
    }

    start = get_absolute_time();
    while (gpio_get(echo_pin_) == 1) {
        if (absolute_time_diff_us(start, get_absolute_time()) > TIMEOUT_US)
            return 0.0f;
    }
    end = get_absolute_time();

    pulse_time_us = absolute_time_diff_us(start, end);
    float dist = (pulse_time_us * SOUND_SPEED_CM_PER_US) / 2.0f;
    return dist;
}

float ultrasonic_get_distance_cm(void) {
    const int N = 5;
    float samples[N];
    int valid = 0;
    const float MAX_DEVIATION = 30.0f;

    for (int i = 0; i < N; i++) {
        float d = ultrasonic_read_once();
        if (d > 2.0f && d < MAX_DISTANCE_CM) {
            if (prev_valid_distance_ > 0.0f && fabs(d - prev_valid_distance_) > MAX_DEVIATION)
                continue;
            samples[valid++] = d;
        }
        sleep_ms(20);
    }

    if (valid == 0) return 0.0f;

    // insertion sort
    for (int i = 1; i < valid; i++) {
        float key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }

    float median = samples[valid / 2];
    prev_valid_distance_ = median;
    return median;
}
