#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <stdbool.h>
#include <stdint.h>

bool wifi_connect(const char *ssid, const char *password);

// Start MQTT: sets up LWT, connects, subscribes to cmd topic
void mqtt_start(const char* broker_ip,
                uint16_t broker_port,
                const char* team,
                const char* car_id);

// Publish a 1 Hz heartbeat (call once per second)
void mqtt_publish_heartbeat(void);

// --- Demo 1 telemetry publishers ---
void tel_publish_speed(float mps, float rpm_l, float rpm_r);       // 5–10 Hz
void tel_publish_distance(float meters);                           // 1–2 Hz
void tel_publish_heading(float deg);                               // 5–10 Hz
void tel_publish_imu_raw(float ax, float ay, float az,
                         float gx, float gy, float gz,
                         float mx, float my, float mz);            // 5–10 Hz
void tel_publish_imu_filtered(float roll, float pitch, float yaw,
                              float heading_deg);                  // 5–10 Hz

// Demo 2
void tel_publish_state(const char* state);                 // retain=1, QoS1
void tel_publish_barcode(const char* code, int junction);  // QoS1
void tel_publish_linepos(float offset_mm, float error);    // 5 Hz

// Demo 3
void tel_publish_obstacle_scan(float left_cm, float right_cm);              // on each sweep
void tel_publish_obstacle_decision(const char* side, float width_cm);      // QoS1
void tel_publish_recovery(const char* status);                              // retain=1, QoS1



#endif
