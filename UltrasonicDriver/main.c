#include <stdio.h>
#include "pico/stdlib.h"
#include "ultrasonic_interrupt.h"
#include "kalman_filter.h"


#define TRIG_PIN 1
#define ECHO_PIN 0
#define MEDIAN_SIZE 5
#define MAX_VALID_DISTANCE_CM 35

int main() {
    stdio_init_all();
    ultrasonic_init(TRIG_PIN, ECHO_PIN);

    uint16_t readings[MEDIAN_SIZE] = {0};
    uint reading_index = 0;

    while (true) {
        uint16_t raw_distance;
        if (ultrasonic_read_cm(&raw_distance)) {
            if (raw_distance == 0 || raw_distance > 1000) {
                printf("Out of range or invalid reading: %d cm\n", raw_distance);
                sleep_ms(100);
                continue;
            }

            // Store reading in circular buffer
            readings[reading_index++ % MEDIAN_SIZE] = raw_distance;
            if (reading_index >= MEDIAN_SIZE) {
                // Copy buffer and sort to compute median
                uint16_t sorted[MEDIAN_SIZE];
                for (int i = 0; i < MEDIAN_SIZE; i++) {
                    sorted[i] = readings[i];
                }

                // Simple bubble sort (since array is small)
                for (int i = 0; i < MEDIAN_SIZE - 1; i++) {
                    for (int j = 0; j < MEDIAN_SIZE - i - 1; j++) {
                        if (sorted[j] > sorted[j + 1]) {
                            uint16_t temp = sorted[j];
                            sorted[j] = sorted[j + 1];
                            sorted[j + 1] = temp;
                        }
                    }
                }

                uint16_t median = sorted[MEDIAN_SIZE / 2];
                if (median <= MAX_VALID_DISTANCE_CM) {
                    printf("Distance: %d cm\n", median);
                } else {
                    printf("Out of range (>35cm): %d cm\n", median);
                }

                static KalmanFilter kf;
                static bool kalman_initialized = false;

                if (!kalman_initialized) {
                    kalman_init(&kf, median, 1.0f, 0.1f, 4.0f); // initial, P, Q, R
                    kalman_initialized = true;
                }

                float kalman_filtered = kalman_update(&kf, median);

                if (kalman_filtered <= MAX_VALID_DISTANCE_CM) {
                    printf("Kalman Distance: %.2f cm\n", kalman_filtered);
                } else {
                    printf("Out of range (>35cm): %.2f cm\n", kalman_filtered);
                }

            }
        } else {
            printf("No valid reading or out of range.\n");
        }

        sleep_ms(100);
    }

    return 0;
}
