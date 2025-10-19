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

#endif
