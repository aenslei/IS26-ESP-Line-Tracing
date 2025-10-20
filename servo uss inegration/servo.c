#include "servo.h"
#include "hardware/pwm.h"
#include "pico/time.h"

static uint servo_pin;
static uint servo_slice;

// Convert angle (0–180°) to pulse width in microseconds
uint16_t servo_angle_to_us(uint8_t angle) {
    if (angle > 180) angle = 180;
    return 1000 + ((angle * 1000) / 180);  // 1ms → 2ms pulse
}

// Initialize servo pin and PWM configuration
void servo_init(uint pin) {
    servo_pin = pin;

    gpio_set_function(servo_pin, GPIO_FUNC_PWM);
    servo_slice = pwm_gpio_to_slice_num(servo_pin);

    pwm_set_clkdiv(servo_slice, 125.0f);   // 1 µs per tick
    pwm_set_wrap(servo_slice, 20000);      // 20ms period = 50Hz
    pwm_set_enabled(servo_slice, true);
}

// Move servo instantly to a specific angle
void servo_set_angle(uint8_t angle) {
    uint16_t pulse = servo_angle_to_us(angle);
    pwm_set_chan_level(servo_slice, pwm_gpio_to_channel(servo_pin), pulse);
}

// Smooth transition between two angles
void servo_smooth_move(uint8_t from_angle, uint8_t to_angle, uint8_t step_size, uint delay_ms) {
    if (from_angle == to_angle) {
        servo_set_angle(to_angle);
        return;
    }

    // Determine direction (increasing or decreasing)
    if (from_angle < to_angle) {
        for (uint8_t angle = from_angle; angle <= to_angle; angle += step_size) {
            servo_set_angle(angle);
            sleep_ms(delay_ms);
        }
    } else {
        for (int angle = from_angle; angle >= to_angle; angle -= step_size) {
            servo_set_angle(angle);
            sleep_ms(delay_ms);
        }
    }
}
