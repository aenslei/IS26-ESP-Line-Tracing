#include "servo.h"
#include "hardware/pwm.h"
#include "pico/time.h"

static uint servo_pin;
static uint servo_slice;

// Convert logical angle (absolute 0–180°) to PWM pulse width (1000–2000 µs)
static uint16_t servo_angle_to_us(uint8_t angle) {
    if (angle > 180) angle = 180;
    return 1000 + ((angle * 1000) / 180);  // 1ms–2ms range
}

// Initialize servo PWM on a specific GPIO pin
void servo_init(uint pin) {
    servo_pin = pin;
    gpio_set_function(servo_pin, GPIO_FUNC_PWM);
    servo_slice = pwm_gpio_to_slice_num(servo_pin);

    pwm_set_clkdiv(servo_slice, 125.0f);   // 125MHz / 125 = 1MHz (1µs per tick)
    pwm_set_wrap(servo_slice, 20000);      // 20ms period = 50Hz
    pwm_set_enabled(servo_slice, true);
}

// Move servo instantly to an absolute angle (0–180°)
void servo_set_absolute(uint8_t angle) {
    uint16_t pulse = servo_angle_to_us(angle);
    pwm_set_chan_level(servo_slice, pwm_gpio_to_channel(servo_pin), pulse);
}

// Move servo relative to center = 0°
// left: negative (−), right: positive (+)
// maps −60° → 30°, 0° → 90°, +60° → 150°
void servo_set_relative(int8_t relative_angle) {
    int absolute_angle = 90 + relative_angle;
    if (absolute_angle < 0) absolute_angle = 0;
    if (absolute_angle > 180) absolute_angle = 180;
    servo_set_absolute((uint8_t)absolute_angle);
}

// Smoothly move between two relative angles
void servo_smooth_move_relative(int8_t from_rel, int8_t to_rel, uint8_t step, uint delay_ms) {
    if (from_rel == to_rel) {
        servo_set_relative(to_rel);
        return;
    }

    if (from_rel < to_rel) {
        for (int8_t a = from_rel; a <= to_rel; a += step) {
            servo_set_relative(a);
            sleep_ms(delay_ms);
        }
    } else {
        for (int8_t a = from_rel; a >= to_rel; a -= step) {
            servo_set_relative(a);
            sleep_ms(delay_ms);
        }
    }
}
