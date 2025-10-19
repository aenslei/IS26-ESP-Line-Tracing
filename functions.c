#include "functions.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip_addr.h"
#include "lwip/apps/mqtt.h"

static mqtt_client_t *g_client = NULL;
static bool g_mqtt_up = false;

static char topic_status[96];
static char topic_heart[96];
static char topic_cmdnav[96];
static char last_topic[128];

static int ci_equal(const char* a, const char* b) {
    while (*a && *b) {
        if (tolower((unsigned char)*a++) != tolower((unsigned char)*b++)) return 0;
    }
    return *a == *b;
}

bool wifi_connect(const char *ssid, const char *password) {
    if (cyw43_arch_init()) { printf("WiFi init failed\n"); return false; }
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 15000)) {
        printf("WiFi connect failed\n");
        return false;
    }
    printf("WiFi connected\n");
    return true;
}

static void req_cb(void *arg, err_t err) {
    (void)arg;
    if (err != ERR_OK) printf("[MQTT] req_cb err=%d\n", err);
}

static void inpub_cb(void *arg, const char *topic, u32_t tot_len) {
    (void)arg; (void)tot_len;
    snprintf(last_topic, sizeof(last_topic), "%s", topic);
    printf("[MQTT] incoming publish on %s\n", last_topic);
}

static void indata_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    (void)arg;
    char msg[128] = {0};
    if (len >= sizeof(msg)) len = sizeof(msg) - 1;
    memcpy(msg, data, len);
    if (flags & MQTT_DATA_FLAG_LAST) {
        printf("[MQTT] payload: %s\n", msg);
        if (strcmp(last_topic, topic_cmdnav) == 0) {
            if      (ci_equal(msg, "LEFT"))  printf("CMD: LEFT\n");
            else if (ci_equal(msg, "RIGHT")) printf("CMD: RIGHT\n");
            else if (ci_equal(msg, "STOP"))  printf("CMD: STOP\n");
            else if (ci_equal(msg, "UTURN")) printf("CMD: UTURN\n");
        }
    }
}

static void conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        g_mqtt_up = true;
        printf("[MQTT] connected\n");

        mqtt_publish(client, topic_status, "true", 4, 1, 1, req_cb, NULL);
        mqtt_set_inpub_callback(client, inpub_cb, indata_cb, NULL);
        mqtt_subscribe(client, topic_cmdnav, 1, req_cb, NULL);
    } else {
        g_mqtt_up = false;
        printf("[MQTT] connect failed, status=%d\n", status);
    }
}

void mqtt_start(const char* broker_ip, uint16_t broker_port,
                const char* team, const char* car_id)
{
    snprintf(topic_status, sizeof(topic_status), "robot/%s/%s/status/online", team, car_id);
    snprintf(topic_heart, sizeof(topic_heart),   "robot/%s/%s/telemetry/heartbeat", team, car_id);
    snprintf(topic_cmdnav, sizeof(topic_cmdnav), "robot/%s/%s/cmd/nav", team, car_id);

    ip_addr_t broker;
    if (!ipaddr_aton(broker_ip, &broker)) {
        printf("[MQTT] bad broker ip\n");
        return;
    }

    g_client = mqtt_client_new();
    if (!g_client) { printf("[MQTT] alloc failed\n"); return; }

    static struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id   = "teamA-car1";
    ci.keep_alive  = 30;
    ci.will_topic  = topic_status;
    ci.will_msg    = "false";
    ci.will_qos    = 1;
    ci.will_retain = 1;

    printf("[MQTT] connecting to %s:%u …\n", broker_ip, broker_port);
    mqtt_client_connect(g_client, &broker, broker_port, conn_cb, NULL, &ci);
}

void mqtt_publish_heartbeat(void) {
    if (!g_client || !g_mqtt_up) return;
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"t_ms\":%lu}", to_ms_since_boot(get_absolute_time()));
    mqtt_publish(g_client, topic_heart, payload, strlen(payload), 0, 0, req_cb, NULL);
}