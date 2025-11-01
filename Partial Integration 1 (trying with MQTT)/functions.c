// functions.c — Minimal MQTT "bus" for Pico W + lwIP
// - No fixed topics
// - Clean publish/subscribe API
// - Auto-resubscribe on reconnect
// - Uses pico_cyw43_arch_lwip_threadsafe_background

#include "functions.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip_addr.h"
#include "lwip/apps/mqtt.h"

// ---------- Wi-Fi ----------
bool wifi_connect(const char *ssid, const char *password) {
    if (cyw43_arch_init()) { printf("[WiFi] cyw43 init failed\n"); return false; }
    cyw43_arch_enable_sta_mode();
    printf("[WiFi] connecting to %s ...\n", ssid);
    int rc = cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 30000);
    if (rc) { printf("[WiFi] connect failed rc=%d\n", rc); return false; }
    printf("[WiFi] connected\n");
    return true;
}

// ---------- MQTT Bus internals ----------
static mqtt_client_t *g_client = NULL;
static bool           g_up     = false;

static ip_addr_t g_broker_addr;
static u16_t     g_broker_port = 1883;
static struct mqtt_connect_client_info_t g_ci;

#define MAX_SUBS 10
typedef struct {
    char          topic[96];
    uint8_t       qos;
    mqtt_msg_cb_t cb;
    void         *user;
    bool          in_use;
} sub_entry_t;

static sub_entry_t g_subs[MAX_SUBS];
static char        g_last_topic[128];

static void req_cb(void *arg, err_t result) {
    (void)arg; if (result != ERR_OK) printf("[MQTT] request err=%d\n", result);
}

static void inpub_cb(void *arg, const char *topic, u32_t tot_len) {
    (void)arg; (void)tot_len;
    strncpy(g_last_topic, topic, sizeof(g_last_topic)-1);
    g_last_topic[sizeof(g_last_topic)-1] = 0;
}

static void indata_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    (void)arg;
    if (flags & MQTT_DATA_FLAG_LAST) {
        for (int i = 0; i < MAX_SUBS; ++i) {
            if (g_subs[i].in_use && strcmp(g_last_topic, g_subs[i].topic) == 0) {
                if (g_subs[i].cb) g_subs[i].cb(g_last_topic, data, len, g_subs[i].user);
                return;
            }
        }
        printf("[MQTT] %s: %.*s\n", g_last_topic, (int)len, (const char*)data);
    }
}

static void resubscribe_all(mqtt_client_t *client) {
    for (int i = 0; i < MAX_SUBS; ++i)
        if (g_subs[i].in_use) mqtt_subscribe(client, g_subs[i].topic, g_subs[i].qos, req_cb, NULL);
}

static void conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        g_up = true;
        printf("[MQTT] CONNECTED\n");
        mqtt_set_inpub_callback(client, inpub_cb, indata_cb, NULL);
        resubscribe_all(client);
    } else {
        g_up = false;
        printf("[MQTT] connection lost (status=%d). Reconnecting...\n", status);
        sleep_ms(1000);
        mqtt_client_connect(client, &g_broker_addr, g_broker_port, conn_cb, NULL, &g_ci);
    }
}

// ---------- MQTT Bus API ----------
bool mqtt_bus_connect(const mqtt_bus_config_t *cfg) {
    if (!cfg || !cfg->broker_ip || !cfg->broker_port) { printf("[MQTT] invalid config\n"); return false; }
    if (!ipaddr_aton(cfg->broker_ip, &g_broker_addr)) { printf("[MQTT] bad broker IP: %s\n", cfg->broker_ip); return false; }
    g_broker_port = cfg->broker_port;

    g_client = mqtt_client_new();
    if (!g_client) { printf("[MQTT] client alloc failed\n"); return false; }

    memset(&g_ci, 0, sizeof(g_ci));
    static char client_id[64];
    if (cfg->client_id && cfg->client_id[0]) snprintf(client_id, sizeof(client_id), "%s", cfg->client_id);
    else snprintf(client_id, sizeof(client_id), "pico-%lu", (unsigned long)to_ms_since_boot(get_absolute_time()));
    g_ci.client_id  = client_id;
    g_ci.keep_alive = cfg->keep_alive_s ? cfg->keep_alive_s : 30;

    if (cfg->will_topic && cfg->will_msg) {
        g_ci.will_topic  = cfg->will_topic;
        g_ci.will_msg    = cfg->will_msg;
        g_ci.will_qos    = (cfg->will_qos <= 2) ? cfg->will_qos : 0;
        g_ci.will_retain = cfg->will_retain ? 1 : 0;
    }

    printf("[MQTT] connecting to %s:%u ...\n", cfg->broker_ip, cfg->broker_port);
    mqtt_client_connect(g_client, &g_broker_addr, g_broker_port, conn_cb, NULL, &g_ci);
    return true;
}

bool mqtt_bus_is_connected(void) { return g_up; }

bool mqtt_bus_publish(const char *topic, const void *payload, size_t len, uint8_t qos, bool retain) {
    if (!g_client || !g_up || !topic || !payload) return false;
    if (qos > 2) qos = 0;
    err_t e = mqtt_publish(g_client, topic, payload, (u16_t)len, qos, retain ? 1 : 0, req_cb, NULL);
    if (e != ERR_OK) { printf("[MQTT] publish err=%d\n", e); return false; }
    return true;
}

bool mqtt_bus_publish_str(const char *topic, const char *str, uint8_t qos, bool retain) {
    if (!str) return false;
    return mqtt_bus_publish(topic, str, strlen(str), qos, retain);
}

bool mqtt_bus_publishf(const char *topic, uint8_t qos, bool retain, const char *fmt, ...) {
    if (!fmt) return false;
    char buf[256];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return mqtt_bus_publish(topic, buf, strlen(buf), qos, retain);
}

bool mqtt_bus_subscribe(const char *topic, uint8_t qos, mqtt_msg_cb_t cb, void *user_data) {
    if (!topic || !topic[0]) return false;

    int slot = -1;
    for (int i = 0; i < MAX_SUBS; ++i)
        if (g_subs[i].in_use && strcmp(g_subs[i].topic, topic) == 0) { slot = i; break; }
    if (slot < 0) for (int i = 0; i < MAX_SUBS; ++i) if (!g_subs[i].in_use) { slot = i; break; }
    if (slot < 0) { printf("[MQTT] no sub slots left\n"); return false; }

    strncpy(g_subs[slot].topic, topic, sizeof(g_subs[slot].topic)-1);
    g_subs[slot].topic[sizeof(g_subs[slot].topic)-1] = 0;
    g_subs[slot].qos  = (qos <= 2) ? qos : 0;
    g_subs[slot].cb   = cb;
    g_subs[slot].user = user_data;
    g_subs[slot].in_use = true;

    if (g_client && g_up) {
        err_t e = mqtt_subscribe(g_client, g_subs[slot].topic, g_subs[slot].qos, req_cb, NULL);
        if (e != ERR_OK) { printf("[MQTT] sub err=%d\n", e); return false; }
    }
    return true;
}

int mqtt_topicf(char *out, size_t out_sz, const char *fmt, ...) {
    if (!out || !out_sz || !fmt) return -1;
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(out, out_sz, fmt, ap);
    va_end(ap);
    return n;
}
