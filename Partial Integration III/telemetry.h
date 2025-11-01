#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// Call once after mqtt_connect()
void telemetry_init(const char *team_id, const char *car_id);

// Convenience: builds "robot/<team>/<car>/<path>" into out
int telemetry_topic(char *out, size_t out_sz, const char *path);

// Publish helpers (no one else needs to touch MQTT)
bool telemetry_pub_str(const char *path, const char *s, uint8_t qos, bool retain);
bool telemetry_pub_i32(const char *path, int32_t v, uint8_t qos, bool retain);
bool telemetry_pub_f32(const char *path, float v, uint8_t qos, bool retain);

// Subscribe to a command path, e.g., "cmd/servo/angle"
typedef void (*telemetry_cmd_cb_t)(const char *full_topic,
                                   const uint8_t *payload, uint16_t len, void *user);
bool telemetry_sub_cmd(const char *path, uint8_t qos, telemetry_cmd_cb_t cb, void *user);

#endif
