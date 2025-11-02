#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "servo.h"
#include "ultrasonic.h"

// ---------------- Configuration ----------------
#define TRIG_PIN             1
#define ECHO_PIN             0
#define SERVO_PIN           15

#define STEP_SIZE            5        // step per scan (deg)
#define LEFT_LIMIT         -50        // physical LEFT
#define RIGHT_LIMIT         50        // physical RIGHT
#define EDGE_THRESHOLD_CM    8.0f     // sudden jump threshold → edge detected
#define DETECT_RANGE_CM     10.0f     // obstacle detection threshold
#define WARNING_RANGE_CM     5.0f     // too close, immediate reverse
#define SCAN_DELAY_MS     2000        // delay between servo steps (ms)
#define DEFAULT_WIDTH_CM    -1.0f     // invalid marker for width
#define SERVO_CENTER_OFFSET  0        // calibration offset if needed
// ------------------------------------------------

// Utility
static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }

// Convert a physical angle into the servo's internal logical rotation
static inline int physical_to_servo_angle(int physical_angle) {
    // Because servo movement is mirrored relative to USS
    return -physical_angle + SERVO_CENTER_OFFSET;
}

// Compute obstacle width (physical layer)
float obstacle_width_from_edges(float left_deg, float left_cm,
                                float right_deg, float right_cm) {
    float yL = left_cm  * sinf(deg2rad(left_deg));
    float yR = right_cm * sinf(deg2rad(right_deg));
    return fabsf(yR - yL);
}

int main() {
    stdio_init_all();
    servo_init(SERVO_PIN);
    ultrasonic_init(TRIG_PIN, ECHO_PIN);

    printf("\n=== Servo + Ultrasonic Integration (Physical-Layer Angles) ===\n");
    printf("Convention: Negative = LEFT, Positive = RIGHT, 0 = Center\n\n");

    while (true) {
        // ---------- 1. Front distance check ----------
        float front_dist = ultrasonic_get_distance_med5();

        if (!ultrasonic_is_valid(front_dist)) {
            printf("[FRONT] Invalid USS reading.\n");
            sleep_ms(500);
            continue;
        }

        printf("[FRONT] Distance: %.2f cm\n", front_dist);

        if (front_dist <= WARNING_RANGE_CM) {
            float reverse_cm = WARNING_RANGE_CM - front_dist;
            printf("[WARNING] Too close! Reverse %.2f cm to gain room.\n", reverse_cm);
            sleep_ms(2000);
            continue;
        }
        else if (front_dist <= DETECT_RANGE_CM) {
            printf("[DETECTED] Obstacle %.2f cm ahead — initiating scan.\n", front_dist);
        }
        else {
            printf("[CLEAR] Path clear (%.2f cm)\n", front_dist);
            sleep_ms(500);
            continue;
        }

        // ---------- 2. Setup variables ----------
        bool left_edge_found = false, right_edge_found = false;
        int left_edge_angle = 0, right_edge_angle = 0;
        float left_edge_dist = 0.0f, right_edge_dist = 0.0f;
        float obstacle_width = DEFAULT_WIDTH_CM;

        printf("\n[RESET] Centering servo before scan...\n");
        servo_smooth_move_relative(physical_to_servo_angle(0),
                                   physical_to_servo_angle(0),
                                   STEP_SIZE, 100);
        sleep_ms(500);

        // ============================================================
        //  PHYSICAL LEFT SCAN (negative angles)
        // ============================================================
        printf("\n[SCAN] LEFT side (physical):\n");
        float prev_dist = ultrasonic_get_distance_med5();
        int prev_angle = 0;

        for (int angle = -STEP_SIZE; angle >= LEFT_LIMIT; angle -= STEP_SIZE) {
            servo_set_relative(physical_to_servo_angle(angle));
            sleep_ms(SCAN_DELAY_MS);

            float dist = ultrasonic_get_distance_med5();
            printf("[SCAN] Dir=LEFT  | Angle=%+3d deg | Dist=%.2f cm\n", angle, dist);

            if (ultrasonic_is_valid(prev_dist) && ultrasonic_is_valid(dist)) {
                float jump = fabsf(dist - prev_dist);
                if (jump > EDGE_THRESHOLD_CM && !left_edge_found) {
                    left_edge_found = true;
                    left_edge_angle = prev_angle;
                    left_edge_dist = prev_dist;
                    printf(" [EDGE] LEFT boundary @ %d deg (%.2f -> %.2f)\n",
                           prev_angle, prev_dist, dist);
                }
            }

            prev_dist = dist;
            prev_angle = angle;
        }

        servo_smooth_move_relative(physical_to_servo_angle(LEFT_LIMIT),
                                   physical_to_servo_angle(0),
                                   STEP_SIZE, 200);
        sleep_ms(600);

        // ============================================================
        //  PHYSICAL RIGHT SCAN (positive angles)
        // ============================================================
        printf("\n[SCAN] RIGHT side (physical):\n");
        prev_dist = ultrasonic_get_distance_med5();
        prev_angle = 0;

        for (int angle = STEP_SIZE; angle <= RIGHT_LIMIT; angle += STEP_SIZE) {
            servo_set_relative(physical_to_servo_angle(angle));
            sleep_ms(SCAN_DELAY_MS);

            float dist = ultrasonic_get_distance_med5();
            printf("[SCAN] Dir=RIGHT | Angle=%+3d deg | Dist=%.2f cm\n", angle, dist);

            if (ultrasonic_is_valid(prev_dist) && ultrasonic_is_valid(dist)) {
                float jump = fabsf(dist - prev_dist);
                if (jump > EDGE_THRESHOLD_CM && !right_edge_found) {
                    right_edge_found = true;
                    right_edge_angle = prev_angle;
                    right_edge_dist = prev_dist;
                    printf(" [EDGE] RIGHT boundary @ %d deg (%.2f -> %.2f)\n",
                           prev_angle, prev_dist, dist);
                }
            }

            prev_dist = dist;
            prev_angle = angle;
        }

        servo_smooth_move_relative(physical_to_servo_angle(RIGHT_LIMIT),
                                   physical_to_servo_angle(0),
                                   STEP_SIZE, 200);
        sleep_ms(1000);

        // ============================================================
        //  RESULTS + DECISION
        // ============================================================
        printf("\n[RESULT] Edge summary (physical angles):\n");
        printf("  LEFT  : %s @ %d deg (%.2f cm)\n",
               left_edge_found ? "FOUND" : "NOT FOUND",
               left_edge_angle, left_edge_dist);
        printf("  RIGHT : %s @ %d deg (%.2f cm)\n",
               right_edge_found ? "FOUND" : "NOT FOUND",
               right_edge_angle, right_edge_dist);

        if (left_edge_found && right_edge_found) {
            obstacle_width = obstacle_width_from_edges(left_edge_angle, left_edge_dist,
                                                       right_edge_angle, right_edge_dist);
            printf("[WIDTH] Estimated obstacle width ≈ %.2f cm\n", obstacle_width);

            if (right_edge_dist > left_edge_dist + 0.01f) {
                printf("[DECISION] More space RIGHT -> turn RIGHT %d deg (%.2f cm)\n",
                       right_edge_angle, right_edge_dist);
            }
            else if (left_edge_dist > right_edge_dist + 0.01f) {
                printf("[DECISION] More space LEFT -> turn LEFT %d deg (%.2f cm)\n",
                       left_edge_angle, left_edge_dist);
            }
            else {
                printf("[DECISION] Equal clearance -> default RIGHT.\n");
            }
        }
        else if (left_edge_found) {
            printf("[DECISION] Only LEFT edge found -> turn LEFT to clear.\n");
        }
        else if (right_edge_found) {
            printf("[DECISION] Only RIGHT edge found -> turn RIGHT to clear.\n");
        }
        else {
            printf("[DECISION] No edges -> reverse and rescan.\n");
        }

        printf("------------------------------------------------------------\n\n");
        sleep_ms(5000);
    }
}
