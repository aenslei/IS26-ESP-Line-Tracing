// functions.c  — Pico W + lwIP MQTT (Demo 1 telemetry)
// Works with functions.h that declares: wifi_connect, mqtt_start, mqtt_publish_heartbeat

#include "functions.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/ip_addr.h"
#include "lwip/apps/mqtt.h"

// ==============================
// Globals
// ==============================
static mqtt_client_t *g_client = NULL;
static bool g_mqtt_up = false;

static ip_addr_t g_broker_addr;
static u16_t     g_broker_port = 1883;

// Saved so we can reconnect cleanly
static struct mqtt_connect_client_info_t g_ci;

// identity strings for topics
static char g_team[32] = {0};
static char g_car[32]  = {0};

// Topics
static char topic_status[96];
static char topic_cmdnav[96];
static char topic_heart[96];

static char topic_speed[96];
static char topic_distance[96];
static char topic_heading[96];
static char topic_imu_raw[96];
static char topic_imu_filtered[96];

static char topic_state[96];
static char topic_barcode[96];
static char topic_linepos[96];
static char topic_obst_scan[96];
static char topic_obst_decision[96];
static char topic_recovery[96];


// For incoming publish
static char last_topic[128];

// ==============================
// Helper callbacks
// ==============================
static void req_cb(void *arg, err_t result) {
    (void)arg;
    if (result != ERR_OK) {
        printf("[MQTT] request error=%d\n", result);
    }
}

static int ci_equal(const char* a, const char* b) {
    while (*a && *b) {
        if (tolower((unsigned char)*a++) != tolower((unsigned char)*b++)) return 0;
    }
    return *a == *b;
}

static void inpub_cb(void *arg, const char *topic, u32_t tot_len) {
    (void)arg; (void)tot_len;
    strncpy(last_topic, topic, sizeof(last_topic)-1);
    last_topic[sizeof(last_topic)-1] = 0;
    printf("[MQTT] incoming publish on %s (len=%lu)\n", topic, (unsigned long)tot_len);
}

static void indata_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    (void)arg;
    char buf[256] = {0};
    if (len >= sizeof(buf)) len = sizeof(buf)-1;
    memcpy(buf, data, len);

    if (flags & MQTT_DATA_FLAG_LAST) {
        printf("[MQTT] payload: %s\n", buf);
        if (strcmp(last_topic, topic_cmdnav) == 0) {
            if      (ci_equal(buf, "LEFT"))  printf("[CMD] LEFT\n");
            else if (ci_equal(buf, "RIGHT")) printf("[CMD] RIGHT\n");
            else if (ci_equal(buf, "STOP"))  printf("[CMD] STOP\n");
            else if (ci_equal(buf, "UTURN")) printf("[CMD] UTURN\n");
            else                              printf("[CMD] unknown: %s\n", buf);
        }
    }
}

// Forward declaration
static void conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);

// ==============================
// Wi-Fi
// ==============================
bool wifi_connect(const char *ssid, const char *password) {
    if (cyw43_arch_init()) {
        printf("[WiFi] cyw43 init failed\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();

    printf("[WiFi] connecting to %s ...\n", ssid);
    int rc = cyw43_arch_wifi_connect_timeout_ms(
        ssid, password, CYW43_AUTH_WPA2_AES_PSK, 30000);
    if (rc) {
        printf("[WiFi] connect failed rc=%d\n", rc);
        return false;
    }
    printf("[WiFi] connected\n");
    return true;
}

// ==============================
// MQTT connection callback
// ==============================
static void conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] CONNECTED\n");
        g_mqtt_up = true;

        // Publish retained online=true
        const char *msg_true = "true";
        mqtt_publish(client, topic_status, msg_true, strlen(msg_true), 1, 1, req_cb, NULL);

        // Subscribe and set incoming handlers
        mqtt_set_inpub_callback(client, inpub_cb, indata_cb, NULL);
        mqtt_subscribe(client, topic_cmdnav, 1, req_cb, NULL);
    } else {
        g_mqtt_up = false;
        printf("[MQTT] connection lost (status=%d). Reconnecting...\n", status);
        sleep_ms(1000);
        // Attempt reconnect using same settings
        mqtt_client_connect(client, &g_broker_addr, g_broker_port, conn_cb, NULL, &g_ci);
    }
}

// ==============================
/* Start MQTT:
   - Sets identity (team/car), builds topics
   - Configures LWT (status/online=false retained)
   - Connects and subscribes to cmd/nav
*/
void mqtt_start(const char* broker_ip,
                uint16_t broker_port,
                const char* team,
                const char* car_id) {

    // Save identity
    snprintf(g_team, sizeof(g_team), "%s", team);
    snprintf(g_car,  sizeof(g_car),  "%s", car_id);

    // Build topic strings
    snprintf(topic_status,        sizeof(topic_status),        "robot/%s/%s/status/online",          g_team, g_car);
    snprintf(topic_cmdnav,        sizeof(topic_cmdnav),        "robot/%s/%s/cmd/nav",                g_team, g_car);
    snprintf(topic_heart,         sizeof(topic_heart),         "robot/%s/%s/telemetry/heartbeat",    g_team, g_car);

    snprintf(topic_speed,         sizeof(topic_speed),         "robot/%s/%s/telemetry/speed",        g_team, g_car);
    snprintf(topic_distance,      sizeof(topic_distance),      "robot/%s/%s/telemetry/distance",     g_team, g_car);
    snprintf(topic_heading,       sizeof(topic_heading),       "robot/%s/%s/telemetry/heading",      g_team, g_car);
    snprintf(topic_imu_raw,       sizeof(topic_imu_raw),       "robot/%s/%s/telemetry/imu/raw",      g_team, g_car);
    snprintf(topic_imu_filtered,  sizeof(topic_imu_filtered),  "robot/%s/%s/telemetry/imu/filtered", g_team, g_car);

    snprintf(topic_state,         sizeof(topic_state),         "robot/%s/%s/state/current",          g_team, g_car);
    snprintf(topic_barcode,       sizeof(topic_barcode),       "robot/%s/%s/events/barcode",         g_team, g_car);
    snprintf(topic_linepos,       sizeof(topic_linepos),       "robot/%s/%s/telemetry/line_position",g_team, g_car);
    snprintf(topic_obst_scan,     sizeof(topic_obst_scan),     "robot/%s/%s/obstacle/scan",          g_team, g_car);
    snprintf(topic_obst_decision, sizeof(topic_obst_decision), "robot/%s/%s/obstacle/decision",      g_team, g_car);
    snprintf(topic_recovery,      sizeof(topic_recovery),      "robot/%s/%s/state/recovery",         g_team, g_car);

    // Resolve broker
    if (!ipaddr_aton(broker_ip, &g_broker_addr)) {
        printf("[MQTT] invalid broker IP: %s\n", broker_ip);
        return;
    }
    g_broker_port = broker_port;

    // Create client
    g_client = mqtt_client_new();
    if (!g_client) {
        printf("[MQTT] client alloc failed\n");
        return;
    }

    // Client info (including LWT)
    memset(&g_ci, 0, sizeof(g_ci));
    static char client_id[64];
    snprintf(client_id, sizeof(client_id), "%s-%s-%lu",
             g_team, g_car, (unsigned long)to_ms_since_boot(get_absolute_time()));
    g_ci.client_id  = client_id;
    g_ci.keep_alive = 30;

    g_ci.will_topic  = topic_status;
    g_ci.will_msg    = "false";
    g_ci.will_qos    = 1;
    g_ci.will_retain = 1;

    printf("[MQTT] connecting to %s:%u ...\n", broker_ip, broker_port);
    mqtt_client_connect(g_client, &g_broker_addr, broker_port, conn_cb, NULL, &g_ci);
}



// ==============================
// Demo-1 Telemetry publishers
// (main.c calls these at 10 Hz / 2 Hz)
// ==============================
static void pub0(const char* topic, const char* json) {
    if (!g_client || !g_mqtt_up) return;
    mqtt_publish(g_client, topic, json, strlen(json), 0, 0, req_cb, NULL);
}

void tel_publish_speed(float mps, float rpm_l, float rpm_r) {
    char p[128];
    snprintf(p, sizeof(p),
        "{\"t_ms\":%lu,\"mps\":%.3f,\"rpm_l\":%.1f,\"rpm_r\":%.1f}",
        to_ms_since_boot(get_absolute_time()), mps, rpm_l, rpm_r);
    pub0(topic_speed, p);
}

void tel_publish_distance(float meters) {
    char p[96];
    snprintf(p, sizeof(p),
        "{\"t_ms\":%lu,\"m\":%.3f}",
        to_ms_since_boot(get_absolute_time()), meters);
    pub0(topic_distance, p);
}

void tel_publish_heading(float deg) {
    char p[96];
    snprintf(p, sizeof(p),
        "{\"t_ms\":%lu,\"deg\":%.2f,\"src\":\"fused\"}",
        to_ms_since_boot(get_absolute_time()), deg);
    pub0(topic_heading, p);
}

void tel_publish_imu_raw(float ax, float ay, float az,
                         float gx, float gy, float gz,
                         float mx, float my, float mz) {
    char p[192];
    snprintf(p, sizeof(p),
        "{\"t_ms\":%lu,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
        "\"mx\":%.3f,\"my\":%.3f,\"mz\":%.3f}",
        to_ms_since_boot(get_absolute_time()),
        ax, ay, az, gx, gy, gz, mx, my, mz);
    pub0(topic_imu_raw, p);
}

void tel_publish_imu_filtered(float roll, float pitch, float yaw, float heading_deg) {
    char p[160];
    snprintf(p, sizeof(p),
        "{\"t_ms\":%lu,\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"heading_deg\":%.2f}",
        to_ms_since_boot(get_absolute_time()),
        roll, pitch, yaw, heading_deg);
    pub0(topic_imu_filtered, p);
}

// QoS1 helper (retain flag param)
static void pub1(const char* topic, const char* json, int retain) {
    if (!g_client || !g_mqtt_up) return;
    mqtt_publish(g_client, topic, json, strlen(json), 1, retain ? 1 : 0, req_cb, NULL);
}

// --- Demo 2 ---
void tel_publish_state(const char* state) {
    char p[96];
    snprintf(p, sizeof(p), "{\"t_ms\":%lu,\"state\":\"%s\"}",
             to_ms_since_boot(get_absolute_time()), state);
    pub1(topic_state, p, /*retain=*/1);
}

void tel_publish_barcode(const char* code, int junction) {
    char p[96];
    snprintf(p, sizeof(p), "{\"t_ms\":%lu,\"code\":\"%s\",\"junction_id\":%d}",
             to_ms_since_boot(get_absolute_time()), code, junction);
    pub1(topic_barcode, p, /*retain=*/0);
}

void tel_publish_linepos(float offset_mm, float error) {
    char p[120];
    snprintf(p, sizeof(p), "{\"t_ms\":%lu,\"offset_mm\":%.1f,\"err\":%.3f}",
             to_ms_since_boot(get_absolute_time()), offset_mm, error);
    pub0(topic_linepos, p);
}

// --- Demo 3 ---
void tel_publish_obstacle_scan(float left_cm, float right_cm) {
    char p[96];
    snprintf(p, sizeof(p), "{\"t_ms\":%lu,\"left_cm\":%.1f,\"right_cm\":%.1f}",
             to_ms_since_boot(get_absolute_time()), left_cm, right_cm);
    pub0(topic_obst_scan, p);
}

void tel_publish_obstacle_decision(const char* side, float width_cm) {
    char p[96];
    snprintf(p, sizeof(p), "{\"t_ms\":%lu,\"chosen_side\":\"%s\",\"width_cm\":%.1f}",
             to_ms_since_boot(get_absolute_time()), side, width_cm);
    pub1(topic_obst_decision, p, /*retain=*/0);
}

void tel_publish_recovery(const char* status) {
    char p[96];
    snprintf(p, sizeof(p), "{\"t_ms\":%lu,\"status\":\"%s\"}",
             to_ms_since_boot(get_absolute_time()), status);
    pub1(topic_recovery, p, /*retain=*/1);
}
