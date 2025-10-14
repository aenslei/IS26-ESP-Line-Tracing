#include "ultrasonic_interrupt.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

static uint trig_pin, echo_pin;
static volatile absolute_time_t echo_start, echo_end;
static volatile bool echo_received = false;

static void echo_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        echo_start = get_absolute_time();
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        echo_end = get_absolute_time();
        echo_received = true;
    }
}

void ultrasonic_init(uint trig, uint echo) {
    trig_pin = trig;
    echo_pin = echo;

    gpio_init(trig_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_put(trig_pin, 0);

    gpio_init(echo_pin);
    gpio_set_dir(echo_pin, GPIO_IN);
    gpio_pull_down(echo_pin);

    gpio_set_irq_enabled_with_callback(echo_pin,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &echo_callback);
}

bool ultrasonic_read_cm(uint16_t* distance_cm) {
    echo_received = false;

    // Send 10us pulse
    gpio_put(trig_pin, 1);
    busy_wait_us_32(10);
    gpio_put(trig_pin, 0);

    // Wait for echo to complete or timeout
    absolute_time_t timeout = make_timeout_time_ms(40);  // ~680cm max
    while (!echo_received) {
        if (absolute_time_diff_us(get_absolute_time(), timeout) <= 0) {
            return false;  // Timed out
        }
    }

    int64_t pulse_duration = absolute_time_diff_us(echo_start, echo_end);
    // Distance in cm: time_us / 58
    if (pulse_duration > 0 && pulse_duration < 40000) {
        *distance_cm = pulse_duration / 58;
        return true;
    }

    return false;  // Invalid reading
}

