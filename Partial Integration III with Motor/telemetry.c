#include "telemetry.h"
#include "functions.h"
#include <stdio.h>
#include <string.h>

static char g_team[24] = "team";
static char g_car[24]  = "car";

void telemetry_init(const char *team_id, const char *car_id) {
    if (team_id && *team_id) snprintf(g_team, sizeof(g_team), "%s", team_id);
    if (car_id && *car_id)   snprintf(g_car,  sizeof(g_car),  "%s", car_id);
}

int telemetry_topic(char *out, size_t out_sz, const char *path) {
    if (!out || !out_sz || !path) return -1;
    while (*path == '/') path++;
    return mqtt_topicf(out, out_sz, "robot/%s/%s/%s", g_team, g_car, path);
}

bool telemetry_pub_str(const char *path, const char *s, uint8_t qos, bool retain) {
    if (!mqtt_bus_is_connected()) return false;
    char topic[96];
    if (telemetry_topic(topic, sizeof(topic), path) < 0) return false;
    return mqtt_bus_publish_str(topic, s, qos, retain);
}

bool telemetry_pub_i32(const char *path, int32_t v, uint8_t qos, bool retain) {
    char buf[32]; snprintf(buf, sizeof(buf), "%ld", (long)v);
    return telemetry_pub_str(path, buf, qos, retain);
}

bool telemetry_pub_f32(const char *path, float v, uint8_t qos, bool retain) {
    char buf[40]; snprintf(buf, sizeof(buf), "%.3f", v);
    return telemetry_pub_str(path, buf, qos, retain);
}

static void bridge_cb(const char *topic, const uint8_t *payload, uint16_t len, void *user) {
    telemetry_cmd_cb_t cb = (telemetry_cmd_cb_t)user;
    if (cb) cb(topic, payload, len, NULL);
}

bool telemetry_sub_cmd(const char *path, uint8_t qos, telemetry_cmd_cb_t cb, void *user) {
    (void)user;
    char topic[96];
    if (telemetry_topic(topic, sizeof(topic), path) < 0) return false;
    return mqtt_bus_subscribe(topic, qos, bridge_cb, (void*)cb);
}
