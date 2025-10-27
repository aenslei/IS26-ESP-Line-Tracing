#include "stdio.h"
#include "pico/stdlib.h"
#include "servo.h"

#define SERVO_PIN    15
#define LEFT_LIMIT   -50     // still relative to servo's logical left
#define RIGHT_LIMIT  +50
#define STEP_SIZE    5
#define STEP_DELAY   2000    // 2s per step for USS readings

int main() {
    stdio_init_all();
    servo_init(SERVO_PIN);

    printf("=== Servo USS-Compatible Sweep (±50°, Mirrored Output) ===\n");
    printf("GP15 = Servo PWM pin\n");

    servo_set_relative(0);
    printf("[CENTER] Servo at 0° (absolute 90°)\n");
    sleep_ms(2000);

    while (true) {
        // --- PHYSICALLY RIGHT (servo turns left logically) ---
        printf("\n[SCAN] Sweeping PHYSICALLY RIGHT (servo -50°)...\n");
        for (int angle = 0; angle >= LEFT_LIMIT; angle -= STEP_SIZE) {
            servo_set_relative(angle);
            printf("[RIGHT] → %+4d° (abs: %3d°)\n", -angle, 90 - angle);
            sleep_ms(STEP_DELAY);
        }

        printf("[RETURN] Returning to center...\n");
        servo_smooth_move_relative(LEFT_LIMIT, 0, STEP_SIZE, STEP_DELAY / 2);
        printf("[CENTER] Reset to 0° before next scan.\n");
        sleep_ms(2000);

        // --- PHYSICALLY LEFT (servo turns right logically) ---
        printf("[SCAN] Sweeping PHYSICALLY LEFT (servo +50°)...\n");
        for (int angle = 0; angle <= RIGHT_LIMIT; angle += STEP_SIZE) {
            servo_set_relative(angle);
            printf("[LEFT ] → %+4d° (abs: %3d°)\n", -angle, 90 - angle);
            sleep_ms(STEP_DELAY);
        }

        printf("[RETURN] Returning to center...\n");
        servo_smooth_move_relative(RIGHT_LIMIT, 0, STEP_SIZE, STEP_DELAY / 2);
        printf("[CENTER] Returned to 0° (absolute 90°)\n");
        sleep_ms(3000);
    }
}


