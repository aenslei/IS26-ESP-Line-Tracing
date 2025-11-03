#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "functions.h"
#include "servo.h"
#include "ultrasonic.h"
#include "WithPID.h"

// ---------------- Wi-Fi/MQTT config ----------------
#ifndef WIFI_SSID
#define WIFI_SSID   "X3X200Ultra"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS   "iwAnn@sl33pZZZ"
#endif
#ifndef BROKER_IP
#define BROKER_IP   "192.168.244.251"
#endif
#ifndef BROKER_PORT
#define BROKER_PORT 1883
#endif
#ifndef CLIENT_ID
#define CLIENT_ID   "pico-car"
#endif

// ---------------- Hardware + Scan Params ----------------
#define TRIG_PIN            26
#define ECHO_PIN            6
#define SERVO_PIN          15
#define STEP_SIZE           5
#define LEFT_LIMIT        -50
#define RIGHT_LIMIT        50
#define EDGE_THRESHOLD_CM   8.0f
#define DETECT_RANGE_CM    15.0f
#define WARNING_RANGE_CM    5.0f
#define SCAN_SETTLE_MS    250
#define SCAN_DELAY_MS     2000
#define SERVO_CENTER_OFFSET 0

static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }
static inline bool dist_valid(float cm) { return cm > 0.0f; }
static inline int  phys_to_servo(int phys_deg) { return phys_deg + SERVO_CENTER_OFFSET; }

static void pub_state_online(bool on) {
    mqtt_bus_publish_str("state/online", on ? "1" : "0", 0, true);
}
static void pub_state_scan(bool on) {
    mqtt_bus_publish_str("state/scan", on ? "1" : "0", 0, true);
}
static void pub_scan_point(const char *topic, int angle, float cm) {
    mqtt_bus_publishf(topic, 0, false, "%d,%.2f", angle, cm);
}

static volatile bool g_scan = false;

static void on_cmd_scan(const char *topic, const uint8_t *payload, uint16_t len, void *u) {
    (void)topic; (void)u;
    char buf[8]; uint16_t n = (len < sizeof(buf)-1) ? len : (sizeof(buf)-1);
    memcpy(buf, payload, n); buf[n] = 0;
    bool start = (buf[0]=='1'||buf[0]=='t'||buf[0]=='T'||buf[0]=='y'||buf[0]=='Y');
    g_scan = start;
    pub_state_scan(g_scan);
    printf("[CMD] scan=%s\n", g_scan ? "ON" : "OFF");
}

static void on_cmd_servo_angle(const char *topic, const uint8_t *payload, uint16_t len, void *u) {
    (void)topic; (void)u;
    char tmp[32]; uint16_t n = (len < sizeof(tmp)-1) ? len : (sizeof(tmp)-1);
    memcpy(tmp, payload, n); tmp[n] = 0;

    int angle = atoi(tmp);
    if (angle < LEFT_LIMIT)  angle = LEFT_LIMIT;
    if (angle > RIGHT_LIMIT) angle = RIGHT_LIMIT;

    servo_set_relative(phys_to_servo(angle));
    sleep_ms(SCAN_SETTLE_MS);
    float d = ultrasonic_get_distance_med5();
    pub_scan_point("sensors/ultrasonic/point", angle, d);
    printf("[POINT] angle=%+d dist=%.2f\n", angle, d);
}

static float obstacle_width_from_edges(float left_deg, float left_cm, float right_deg, float right_cm) {
    float yL = left_cm  * sinf(deg2rad(left_deg));
    float yR = right_cm * sinf(deg2rad(right_deg));
    return fabsf(yR - yL);
}

int main(void) {
    stdio_init_all();
    sleep_ms(300);

    if (!wifi_connect(WIFI_SSID, WIFI_PASS)) {
        printf("[WiFi] connect failed\n");
        while (1) sleep_ms(2000);
    }

    mqtt_bus_config_t cfg = {
        .broker_ip    = BROKER_IP,
        .broker_port  = BROKER_PORT,
        .client_id    = CLIENT_ID,
        .will_topic   = "state/online",
        .will_msg     = "0",
        .will_qos     = 0,
        .will_retain  = true,
        .keep_alive_s = 30
    };
    mqtt_bus_connect(&cfg);
    while (!mqtt_bus_is_connected()) sleep_ms(20);
    pub_state_online(true);

    telemetry_init("team", "car");  // Ensures debug topics are valid

    mqtt_bus_subscribe("cmd/scan", 0, on_cmd_scan, NULL);
    mqtt_bus_subscribe("cmd/servo/angle", 1, on_cmd_servo_angle, NULL);

    servo_init(SERVO_PIN);
    ultrasonic_init(TRIG_PIN, ECHO_PIN);
    servo_set_relative(phys_to_servo(0));

    //init motor system 
    if (!motor_system_init()) {
        printf("Error: Failed to initialize motor system.\n");
        return 1; // Exit the program with an error code
    }

    printf("\n=== Servo + Ultrasonic Integration (with Debug Headers) ===\n");

    printf("\n=== Servo + Ultrasonic Integration (with MQTT) ===\n");

    bool led = false;
    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led); led = !led;

        float front = ultrasonic_get_distance_med5();
        mqtt_bus_publishf("sensors/front_cm", 0, false, "%.2f", front);

        if (!dist_valid(front)) {
            printf("[FRONT] Invalid USS reading.\n");
            mqtt_bus_publish_str("Invalid","USS Reading",0,false);
            sleep_ms(500);
            continue;
        }

        printf("[FRONT] Distance: %.2f cm\n", front);

        if (front <= WARNING_RANGE_CM) {
            float reverse_cm = WARNING_RANGE_CM - front;
            mqtt_bus_publishf("sensors/reverse_cm", 0, false, "%.2f", reverse_cm);
            mqtt_bus_publish_str("decision", "REVERSE", 0, false);
            printf("[WARNING] Too close! Reverse %.2f cm to gain room.\n", reverse_cm);
            sleep_ms(500);
            continue;

        } else if (front <= DETECT_RANGE_CM) {
            printf("[DETECTED] Obstacle %.2f cm ahead - initiating scan.\n", front);
        } else {
            printf("[CLEAR] Path clear (%.2f cm)\n", front);
            robot_movement(0.35f, 0.35f);
            sleep_ms(500);
            continue;
        }
        
        emergency_stop();
        pub_state_scan(true);

        bool left_edge_found=false, right_edge_found=false;
        int left_edge_angle=0, right_edge_angle=0;
        float left_edge_dist=0.0f, right_edge_dist=0.0f;

        printf("\n[SCAN] LEFT side (physical):\n");
        mqtt_bus_publish_str("scan_header", "LEFT SCAN: Angle (deg), Distance (cm)", 0, false);
        float prev_dist = ultrasonic_get_distance_med5();
        int   prev_angle = 0;

        for (int angle = STEP_SIZE; angle <= RIGHT_LIMIT; angle += STEP_SIZE) {
            servo_set_relative(phys_to_servo(angle));
            sleep_ms(SCAN_SETTLE_MS);
            sleep_ms(SCAN_DELAY_MS);

            float dist = ultrasonic_get_distance_med5();

            // flip angle to negative for left (physical left = negative)
            int physical_angle = -angle;
            pub_scan_point("sensors/ultrasonic/scan", physical_angle, dist);
            printf("[SCAN] Dir=LEFT  | Angle=%+3d deg | Dist=%.2f cm\n", physical_angle, dist);

            if (dist_valid(prev_dist) && dist_valid(dist)) {
                float jump = fabsf(dist - prev_dist);
                if (jump > EDGE_THRESHOLD_CM && !left_edge_found) {
                    left_edge_found = true;
                    int physical_edge_angle = -prev_angle;
                    left_edge_angle = physical_edge_angle;
                    left_edge_dist  = prev_dist;
                    mqtt_bus_publishf("sensors/edges/left_deg", 0, false, "%d", left_edge_angle);
                    mqtt_bus_publishf("sensors/edges/left_cm",  0, false, "%.2f", left_edge_dist);
                    printf(" [EDGE] LEFT boundary @ %d deg (%.2f → %.2f)\n", prev_angle, prev_dist, dist);
                }
            }
            prev_dist = dist;
            prev_angle = angle;
        }

        printf("\n[SCAN] RIGHT side (physical):\n");
        mqtt_bus_publish_str("scan_header", "RIGHT SCAN: Angle (deg), Distance (cm)", 0, false);
        servo_set_relative(phys_to_servo(0));
        sleep_ms(400);
        prev_dist = ultrasonic_get_distance_med5();
        prev_angle = 0;

        for (int angle = -STEP_SIZE; angle >= LEFT_LIMIT; angle -= STEP_SIZE) {
            servo_set_relative(phys_to_servo(angle));
            sleep_ms(SCAN_SETTLE_MS);
            sleep_ms(SCAN_DELAY_MS);

            float dist = ultrasonic_get_distance_med5();


            // keep angle as postive (Physical right = posititve)
            int physical_angle = -angle;
            pub_scan_point("sensors/ultrasonic/scan", physical_angle, dist);
            printf("[SCAN] Dir=RIGHT | Angle=%+3d deg | Dist=%.2f cm\n", physical_angle, dist);

            if (dist_valid(prev_dist) && dist_valid(dist)) {
                float jump = fabsf(dist - prev_dist);
                if (jump > EDGE_THRESHOLD_CM && !right_edge_found) {
                    right_edge_found = true;
                    int physical_edge_angle = -prev_angle;
                    right_edge_angle = physical_edge_angle;
                    right_edge_dist  = prev_dist;
                    mqtt_bus_publishf("sensors/edges/right_deg", 0, false, "%d", right_edge_angle);
                    mqtt_bus_publishf("sensors/edges/right_cm",  0, false, "%.2f", right_edge_dist);
                    printf(" [EDGE] RIGHT boundary @ %d deg (%.2f → %.2f)\n", prev_angle, prev_dist, dist);
                }
            }
            prev_dist = dist;
            prev_angle = angle;
        }

        // Decision
        printf("\n[RESULT] Edge summary (physical angles):\n");
        printf("  LEFT  : %s @ %d deg (%.2f cm)\n", left_edge_found ? "FOUND" : "NOT FOUND", left_edge_angle, left_edge_dist);
        printf("  RIGHT : %s @ %d deg (%.2f cm)\n", right_edge_found ? "FOUND" : "NOT FOUND", right_edge_angle, right_edge_dist);

        if (left_edge_found && right_edge_found) {
            float width = obstacle_width_from_edges(left_edge_angle, left_edge_dist,
                                                    right_edge_angle, right_edge_dist);
            mqtt_bus_publishf("sensors/obstacle_width_cm", 0, false, "%.2f", width);
            printf("[WIDTH] Estimated obstacle width approx %.2f cm\n", width);

            if (right_edge_dist > left_edge_dist + 0.01f) {
                mqtt_bus_publish_str("decision", "RIGHT", 0, false);
                mqtt_bus_publishf("turn_angle", 0, false, "%d", -right_edge_angle); // ✅ flipped
                printf("[DECISION] More space RIGHT -> turn RIGHT %d deg (%.2f cm)\n", -right_edge_angle, right_edge_dist);
            } else if (left_edge_dist > right_edge_dist + 0.01f) {
                mqtt_bus_publish_str("decision", "LEFT", 0, false);
                mqtt_bus_publishf("turn_angle", 0, false, "%d", -left_edge_angle); // ✅ flipped
                printf("[DECISION] More space LEFT -> turn LEFT %d deg (%.2f cm)\n", -left_edge_angle, left_edge_dist);
            } else {
                mqtt_bus_publish_str("decision", "RIGHT", 0, false);
                mqtt_bus_publishf("turn_angle", 0, false, "0");
                printf("[DECISION] Equal clearance -> default RIGHT.\n");
            }

        } else if (left_edge_found) {
            mqtt_bus_publish_str("decision", "LEFT", 0, false);
            mqtt_bus_publishf("turn_angle", 0, false, "%d", -left_edge_angle);
            printf("[DECISION] Only LEFT edge found -> turn LEFT.\n");

        } else if (right_edge_found) {
            mqtt_bus_publish_str("decision", "RIGHT", 0, false);
            mqtt_bus_publishf("turn_angle", 0, false, "%d", -right_edge_angle);
            printf("[DECISION] Only RIGHT edge found -> turn RIGHT.\n");

        } else {
            mqtt_bus_publish_str("decision", "REVERSE", 0, false);
            mqtt_bus_publishf("turn_angle", 0, false, "0");
            printf("[DECISION] No edges -> reverse and rescan.\n");
        }

        pub_state_scan(false);
        servo_set_relative(phys_to_servo(0));
        printf("------------------------------------------------------------\n\n");
        sleep_ms(5000);
    }
}
