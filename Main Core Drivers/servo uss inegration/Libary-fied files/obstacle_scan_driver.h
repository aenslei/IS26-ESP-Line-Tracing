#ifndef OBSTACLE_SCAN_DRIVER_H
#define OBSTACLE_SCAN_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include <pico/stdlib.h>

// Public data structures
typedef struct {
    uint trig_pin;
    uint echo_pin;
    uint servo_pin;
} ScanHardwareConfig;

typedef struct {
    bool left_edge_found;
    int left_edge_angle;
    float left_edge_dist;

    bool right_edge_found;
    int right_edge_angle;
    float right_edge_dist;

    float estimated_width_cm;

    enum {
        TURN_LEFT,
        TURN_RIGHT,
        TURN_DEFAULT_RIGHT,
        REVERSE_AND_RESCAN
    } decision;
} ObstacleScanResult;

// Public function declarations
void scan_system_init(ScanHardwareConfig config);
void scan_obstacle_and_decide(ObstacleScanResult *result);

#endif // OBSTACLE_SCAN_DRIVER_H
