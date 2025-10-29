#include "obstacle_scan_driver.h"
#include "ultrasonic.h"
#include "servo.h"
#include <math.h>
#include <stdio.h>

#define STEP_SIZE            5
#define LEFT_LIMIT         -50
#define RIGHT_LIMIT         50
#define EDGE_THRESHOLD_CM    8.0f
#define DETECT_RANGE_CM     10.0f
#define WARNING_RANGE_CM     5.0f
#define SCAN_DELAY_MS     2000
#define DEFAULT_WIDTH_CM    -1.0f
#define SERVO_CENTER_OFFSET  0

static uint g_servo_pin, g_trig_pin, g_echo_pin;

static inline int physical_to_servo_angle(int physical_angle) {
    return -physical_angle + SERVO_CENTER_OFFSET;
}

static inline float deg2rad(float d) {
    return d * (float)M_PI / 180.0f;
}

static float obstacle_width_from_edges(float left_deg, float left_cm,
                                       float right_deg, float right_cm) {
    float yL = left_cm * sinf(deg2rad(left_deg));
    float yR = right_cm * sinf(deg2rad(right_deg));
    return fabsf(yR - yL);
}

void scan_system_init(ScanHardwareConfig config) {
    g_servo_pin = config.servo_pin;
    g_trig_pin = config.trig_pin;
    g_echo_pin = config.echo_pin;

    servo_init(g_servo_pin);
    ultrasonic_init(g_trig_pin, g_echo_pin);
}

void scan_obstacle_and_decide(ObstacleScanResult *result) {
    float front_dist = ultrasonic_get_distance_med5();

    if (!ultrasonic_is_valid(front_dist)) {
        result->decision = REVERSE_AND_RESCAN;
        return;
    }

    if (front_dist <= WARNING_RANGE_CM) {
        result->decision = REVERSE_AND_RESCAN;
        return;
    } else if (front_dist > DETECT_RANGE_CM) {
        result->decision = TURN_DEFAULT_RIGHT;
        return;
    }

    *result = (ObstacleScanResult){
        .left_edge_found = false,
        .right_edge_found = false,
        .estimated_width_cm = DEFAULT_WIDTH_CM
    };

    servo_smooth_move_relative(physical_to_servo_angle(0),
                               physical_to_servo_angle(0), STEP_SIZE, 100);
    sleep_ms(500);

    float prev_dist = ultrasonic_get_distance_med5();
    int prev_angle = 0;
    for (int angle = -STEP_SIZE; angle >= LEFT_LIMIT; angle -= STEP_SIZE) {
        servo_set_relative(physical_to_servo_angle(angle));
        sleep_ms(SCAN_DELAY_MS);
        float dist = ultrasonic_get_distance_med5();
        if (ultrasonic_is_valid(prev_dist) && ultrasonic_is_valid(dist)) {
            if (fabsf(dist - prev_dist) > EDGE_THRESHOLD_CM && !result->left_edge_found) {
                result->left_edge_found = true;
                result->left_edge_angle = prev_angle;
                result->left_edge_dist = prev_dist;
            }
        }
        prev_dist = dist;
        prev_angle = angle;
    }

    servo_smooth_move_relative(physical_to_servo_angle(LEFT_LIMIT),
                               physical_to_servo_angle(0), STEP_SIZE, 200);
    sleep_ms(600);

    prev_dist = ultrasonic_get_distance_med5();
    prev_angle = 0;
    for (int angle = STEP_SIZE; angle <= RIGHT_LIMIT; angle += STEP_SIZE) {
        servo_set_relative(physical_to_servo_angle(angle));
        sleep_ms(SCAN_DELAY_MS);
        float dist = ultrasonic_get_distance_med5();
        if (ultrasonic_is_valid(prev_dist) && ultrasonic_is_valid(dist)) {
            if (fabsf(dist - prev_dist) > EDGE_THRESHOLD_CM && !result->right_edge_found) {
                result->right_edge_found = true;
                result->right_edge_angle = prev_angle;
                result->right_edge_dist = prev_dist;
            }
        }
        prev_dist = dist;
        prev_angle = angle;
    }

    servo_smooth_move_relative(physical_to_servo_angle(RIGHT_LIMIT),
                               physical_to_servo_angle(0), STEP_SIZE, 200);
    sleep_ms(1000);

    if (result->left_edge_found && result->right_edge_found) {
        result->estimated_width_cm = obstacle_width_from_edges(
            result->left_edge_angle, result->left_edge_dist,
            result->right_edge_angle, result->right_edge_dist);

        if (result->right_edge_dist > result->left_edge_dist + 0.01f)
            result->decision = TURN_RIGHT;
        else if (result->left_edge_dist > result->right_edge_dist + 0.01f)
            result->decision = TURN_LEFT;
        else
            result->decision = TURN_DEFAULT_RIGHT;
    } else if (result->left_edge_found) {
        result->decision = TURN_LEFT;
    } else if (result->right_edge_found) {
        result->decision = TURN_RIGHT;
    } else {
        result->decision = REVERSE_AND_RESCAN;
    }
}

/*
// Optional standalone test
#ifdef STANDALONE_MODE
int main() {
    return 0;
}
#endif
*/
