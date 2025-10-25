#include "stdio.h"
#include "pico/stdlib.h"
#include "servo.h"

#define SERVO_PIN 15

#define LEFT_LIMIT   -70   // degrees left
#define RIGHT_LIMIT  +70   // degrees right
#define STEP_SIZE    10
#define STEP_DELAY   2000  // 2 seconds per step (ms)

int main() {
    stdio_init_all();
    servo_init(SERVO_PIN);

    printf("=== Centered Servo USS-Compatible Sweep (±70°, 2s per step, Flipped) ===\n");
    printf("GP15 = Servo PWM pin\n");

    servo_set_relative(0);
    printf("[CENTER] Servo at 0° (absolute 90°)\n");
    sleep_ms(2000);

    while (true) {
        // --- LEFT SWEEP (flipped) ---
        printf("\n[SCAN] Sweeping LEFT side (0° → -70°)...\n");
        for (int angle = 0; angle >= LEFT_LIMIT; angle -= STEP_SIZE) {
            servo_set_relative(-angle);  // 🔁 flip direction
            printf("[LEFT ] → %+4d° (abs: %3d°)\n", angle, 90 - angle);
            sleep_ms(STEP_DELAY);
        }

        servo_set_relative(0);
        printf("[CENTER] Reset to 0° before right scan.\n");
        sleep_ms(2000);

        // --- RIGHT SWEEP (flipped) ---
        printf("[SCAN] Sweeping RIGHT side (0° → +70°)...\n");
        for (int angle = 0; angle <= RIGHT_LIMIT; angle += STEP_SIZE) {
            servo_set_relative(-angle);  // 🔁 flip direction
            printf("[RIGHT] → %+4d° (abs: %3d°)\n", angle, 90 - angle);
            sleep_ms(STEP_DELAY);
        }

        servo_set_relative(0);
        printf("[CENTER] Returned to 0° (absolute 90°)\n");
        sleep_ms(3000);
    }
}


