#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define SERVO_PIN 2  // GP2 for signal

// Convert angle (0-180°) to duty cycle (1000–2000us)
uint16_t angle_to_pulse_us(uint8_t angle) {
    return 1000 + ((angle * 1000) / 180);
}

// Send servo pulse of given width (in microseconds)
void set_servo_angle(uint8_t angle) {
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    uint16_t pulse_width = angle_to_pulse_us(angle);
    // 125MHz / 50Hz = 2.5M cycles per period, 16-bit max is 65535
    // So set wrap to 25000 for 20ms period (1 cycle = 1us at /125 divider)
    pwm_set_chan_level(slice, pwm_gpio_to_channel(SERVO_PIN), pulse_width);
}

int main() {
    stdio_init_all();
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_wrap(slice, 20000);         // 20ms period (50Hz)
    pwm_set_clkdiv(slice, 125.0f);      // 125MHz / 125 = 1MHz (1us resolution)
    pwm_set_enabled(slice, true);

    while (true) {
        for (uint8_t angle = 0; angle <= 180; angle += 30) {
            set_servo_angle(angle);
            printf("Angle: %d°\n", angle);
            sleep_ms(800);
        }

        for (int angle = 180; angle >= 0; angle -= 30) {
            set_servo_angle(angle);
            printf("Angle: %d°\n", angle);
            sleep_ms(800);
        }
    }
}
