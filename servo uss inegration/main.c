#include "stdio.h"
#include "math.h"
#include "pico/stdlib.h"
#include "servo.h"
#include "ultrasonic.h"

#define SERVO_PIN 15
#define TRIG_PIN  1
#define ECHO_PIN  0

#define LEFT_LIMIT   30
#define RIGHT_LIMIT  150
#define STEP_ANGLE   10
#define STEP_SIZE    2
#define STEP_DELAY   10
#define SETTLE_MS    200
#define SAFE_DISTANCE_CM 15.0f
#define OBSTACLE_THRESHOLD_CM 30.0f

#define MAX_POINTS ((RIGHT_LIMIT - LEFT_LIMIT) / STEP_ANGLE + 1)

typedef struct {
    uint8_t angle;
    float distance;
} ScanPoint;

ScanPoint scan_data[MAX_POINTS];

void print_scan_summary(int count) {
    printf("\n--- Scan Summary (%d points) ---\n", count);
    for (int i = 0; i < count; i++) {
        if (scan_data[i].distance > 0.0f)
            printf("Angle: %3d° | Dist: %.2f cm%s\n",
                   scan_data[i].angle,
                   scan_data[i].distance,
                   (scan_data[i].distance <= SAFE_DISTANCE_CM) ? "  ⚠️ Close!" : "");
        else
            printf("Angle: %3d° | Invalid\n", scan_data[i].angle);
    }
    printf("-------------------------------\n\n");
}

int main() {
    stdio_init_all();
    servo_init(SERVO_PIN);
    ultrasonic_init(TRIG_PIN, ECHO_PIN);

    printf("=== Servo + Ultrasonic Mapping + Width + Clear-Side ===\n");
    printf("GP15=Servo | GP1=Trig | GP0=Echo (via divider)\n");

    uint8_t current_angle = 90;
    servo_set_angle(current_angle);
    sleep_ms(1000);

    while (true) {
        printf("\n[SCAN] Left → Right mapping...\n");
        int count = 0;

        // Sweep and record
        for (int angle = LEFT_LIMIT; angle <= RIGHT_LIMIT; angle += STEP_ANGLE) {
            servo_smooth_move(current_angle, angle, STEP_SIZE, STEP_DELAY);
            current_angle = angle;
            sleep_ms(SETTLE_MS);

            float dist = ultrasonic_get_distance_cm();
            scan_data[count].angle = angle;
            scan_data[count].distance = dist;
            count++;

            if (dist > 0.0f)
                printf("Angle: %3d° | Distance: %.2f cm%s\n",
                       angle, dist, (dist <= SAFE_DISTANCE_CM) ? "  ⚠️ Close!" : "");
            else
                printf("Angle: %3d° | No valid reading\n", angle);
        }

        // Return to center
        servo_smooth_move(current_angle, 90, STEP_SIZE, STEP_DELAY);
        current_angle = 90;

        // --- Analyse results ---
        print_scan_summary(count);

        // Compute left/right averages
        float left_avg = 0, right_avg = 0;
        int left_cnt = 0, right_cnt = 0;
        for (int i = 0; i < count; i++) {
            if (scan_data[i].angle < 90 && scan_data[i].distance > 0) {
                left_avg += scan_data[i].distance; left_cnt++;
            } else if (scan_data[i].angle > 90 && scan_data[i].distance > 0) {
                right_avg += scan_data[i].distance; right_cnt++;
            }
        }
        if (left_cnt) left_avg /= left_cnt;
        if (right_cnt) right_avg /= right_cnt;

        printf("Left avg: %.2f cm | Right avg: %.2f cm\n", left_avg, right_avg);
        if (left_avg > right_avg)
            printf("🡄 Clearer on the LEFT side.\n");
        else if (right_avg > left_avg)
            printf("🡆 Clearer on the RIGHT side.\n");
        else
            printf("🡅 Equal clearance on both sides.\n");

        // --- Estimate obstacle width ---
        int left_edge_index = -1, right_edge_index = -1;
        for (int i = 0; i < count; i++) {
            if (scan_data[i].distance > 0 &&
                scan_data[i].distance <= OBSTACLE_THRESHOLD_CM) {
                if (left_edge_index == -1)
                    left_edge_index = i;
                right_edge_index = i;
            }
        }

        if (left_edge_index != -1 && right_edge_index != -1 &&
            right_edge_index > left_edge_index) {
            uint8_t left_angle  = scan_data[left_edge_index].angle;
            uint8_t right_angle = scan_data[right_edge_index].angle;
            float avg_distance = 0.0f;
            int mid_count = 0;
            for (int i = left_edge_index; i <= right_edge_index; i++) {
                if (scan_data[i].distance > 0) {
                    avg_distance += scan_data[i].distance;
                    mid_count++;
                }
            }
            if (mid_count > 0) avg_distance /= mid_count;

            float angle_diff_deg = right_angle - left_angle;
            float angle_diff_rad = angle_diff_deg * (M_PI / 180.0f);
            float obstacle_width = 2.0f * avg_distance * sinf(angle_diff_rad / 2.0f);

            printf("Obstacle detected between %d°–%d°\n", left_angle, right_angle);
            printf("Average distance: %.2f cm\n", avg_distance);
            printf("Estimated obstacle width: %.2f cm\n", obstacle_width);
        } else {
            printf("No significant obstacle detected.\n");
        }

        printf("\nReturning to center (90°)...\n");
        sleep_ms(1500);
    }
}

