#include "stdio.h"
#include "pico/stdlib.h"
#include "ultrasonic.h"

#define TRIG_PIN 1
#define ECHO_PIN 0
#define SAFE_DISTANCE_CM 15.0f

int main() {
    stdio_init_all();
    ultrasonic_init(TRIG_PIN, ECHO_PIN);

    printf("=== Ultrasonic Sensor Test (Outlier-Rejected + Smoothed) ===\n");

    float smooth = 0.0f;
    const float alpha = 0.3f; // smoothing factor

    while (true) {
        float raw = ultrasonic_get_distance_cm();

        if (raw > 0.0f) {
            if (smooth == 0.0f) smooth = raw;
            smooth = (1 - alpha) * smooth + alpha * raw;

            printf("Smoothed: %.2f cm  |  Raw: %.2f cm", smooth, raw);
            if (smooth <= SAFE_DISTANCE_CM)
                printf("  ⚠️  Obstacle detected!\n");
            else
                printf("\n");
        } else {
            printf("No valid reading\n");
        }

        sleep_ms(200);
    }
}
