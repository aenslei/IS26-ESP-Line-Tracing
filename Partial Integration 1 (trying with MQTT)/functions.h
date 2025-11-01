#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- Wi-Fi ---
bool wifi_connect(const char *ssid, const char *password);

// --- MQTT Bus (no fixed topics) ---
typedef struct {
    const char *broker_ip;    // e.g. "172.20.10.3"
    uint16_t    broker_port;  // e.g. 1883
    const char *client_id;    // optional; NULL => auto-generate
    // Optional LWT (all NULL/0 to disable)
    const char *will_topic;
    const char *will_msg;
    uint8_t     will_qos;     // 0/1/2
    bool        will_retain;
    // Keepalive seconds (0 = 30s)
    uint16_t    keep_alive_s;
} mqtt_bus_config_t;

bool mqtt_bus_connect(const mqtt_bus_config_t *cfg);
bool mqtt_bus_is_connected(void);

// Publish helpers
bool mqtt_bus_publish(const char *topic, const void *payload, size_t len, uint8_t qos, bool retain);
bool mqtt_bus_publish_str(const char *topic, const char *str, uint8_t qos, bool retain);
bool mqtt_bus_publishf(const char *topic, uint8_t qos, bool retain, const char *fmt, ...);

// Subscribe (exact topic match)
typedef void (*mqtt_msg_cb_t)(const char *topic, const uint8_t *payload, uint16_t len, void *user_data);
bool mqtt_bus_subscribe(const char *topic, uint8_t qos, mqtt_msg_cb_t cb, void *user_data);

// Utility
int mqtt_topicf(char *out, size_t out_sz, const char *fmt, ...);

#ifdef __cplusplus
}
#endif
#endif
