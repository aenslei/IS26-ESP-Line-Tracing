#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define SERVO_PIN 15  // GP15 signal pin

uint16_t angle_to_pulse_us(uint8_t angle) {
    if (angle > 180) angle = 180;
    return 1000 + ((angle * 1000) / 180); // 1–2 ms pulse
}

void set_servo_angle(uint8_t angle) {
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    uint16_t pulse = angle_to_pulse_us(angle);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(SERVO_PIN), pulse);
}

int main() {
    stdio_init_all();
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_clkdiv(slice, 125.0f);  // 1 µs per tick
    pwm_set_wrap(slice, 20000);     // 20 ms period = 50 Hz
    pwm_set_enabled(slice, true);

    const uint step = 10;      // finer scan for Demo 3
    const uint hold_ms = 500;  // allow USS time later
    const uint left_limit  = 30;
    const uint right_limit = 150;

    printf("=== Servo Integration Test (Obstacle Scan Pattern) ===\n");

    // 1. Center
    set_servo_angle(90);
    printf("[CENTER] Servo initialized at 90°\n");
    sleep_ms(1000);

    while (true) {
        // 2. Sweep Left
        for (int angle = 90; angle >= left_limit; angle -= step) {
            printf("[LEFT ] → %3d°\n", angle);
            set_servo_angle(angle);
            sleep_ms(hold_ms);
        }

        // 3. Sweep Right
        for (int angle = left_limit; angle <= right_limit; angle += step) {
            printf("[RIGHT] → %3d°\n", angle);
            set_servo_angle(angle);
            sleep_ms(hold_ms);
        }

        // 4. Return to center
        set_servo_angle(90);
        printf("[CENTER] Returned to 90°\n");
        sleep_ms(800);
    }
}

