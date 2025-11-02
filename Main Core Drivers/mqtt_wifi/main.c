// main.c — Integrated Week 10 Demo (D1 + D2 + D3) with placeholders
// Streams Demo-1 telemetry continuously; injects Demo-2/3 events in one loop.
// Baud: 115200

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "functions.h" // wifi_connect(), mqtt_start()

// If functions.h doesn't declare these yet, externs keep compile happy:
extern void tel_publish_speed(float mps, float rpm_l, float rpm_r);
extern void tel_publish_distance(float meters);
extern void tel_publish_heading(float deg);
extern void tel_publish_imu_raw(float ax,float ay,float az,float gx,float gy,float gz,float mx,float my,float mz);
extern void tel_publish_imu_filtered(float roll,float pitch,float yaw,float heading_deg);

extern void tel_publish_state(const char* state);
extern void tel_publish_barcode(const char* code, int junction);
extern void tel_publish_linepos(float offset_mm, float err);
extern void tel_publish_obstacle_scan(float left_cm, float right_cm);
extern void tel_publish_obstacle_decision(const char* side, float width_cm);
extern void tel_publish_recovery(const char* status);

// ======= Configure for your setup =======
#define WIFI_SSID   "SINGTEL-HOME(2.4GHz)"
#define WIFI_PASS   "FHWIba#24"
#define BROKER_IP   "192.168.1.47"   // your PC's IPv4
#define BROKER_PORT 1883
#define TEAM "teamA"
#define CAR  "car1"
// ========================================

static inline bool due_ms(absolute_time_t *last, uint32_t period_ms) {
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(*last, now) >= (int64_t)period_ms * 1000) {
        *last = now; return true;
    }
    return false;
}
static float clampf(float x, float lo, float hi){ return x < lo ? lo : (x > hi ? hi : x); }

int main(void) {
    stdio_init_all();
    sleep_ms(800);

    if (!wifi_connect(WIFI_SSID, WIFI_PASS)) {
        printf("[WiFi] connect failed — check SSID/PASS\n");
        while (true) sleep_ms(1000);
    }
    mqtt_start(BROKER_IP, BROKER_PORT, TEAM, CAR);
    printf("\n[SIM] Integrated Week 10 demo started (D1 + D2 + D3)\n");

    // --- Telemetry timers (Hz -> ms)
    absolute_time_t t_speed    = get_absolute_time();   // 5 Hz
    absolute_time_t t_heading  = get_absolute_time();   // 5 Hz
    absolute_time_t t_filt     = get_absolute_time();   // 5 Hz
    absolute_time_t t_raw      = get_absolute_time();   // 1 Hz
    absolute_time_t t_distance = get_absolute_time();   // 1 Hz
    absolute_time_t t_linepos  = get_absolute_time();   // 5 Hz

    // --- Motion (placeholders; replace with real sensors later)
    float     distance_m = 0.0f;
    uint32_t  last_ms    = to_ms_since_boot(get_absolute_time());

    // --- Integrated sequence state machine
    // 0: follow, 1: turn-left, 2: follow, 3: turn-right, 4: pre-obstacle follow,
    // 5: scan+decision, 6: detour, 7: rejoin, then back to 0
    int phase = 0;
    uint32_t phase_start = to_ms_since_boot(get_absolute_time());
    int junction = 1;
    int last_phase = -1; // to run "on-enter" actions once

    while (true) {
        // ==== Base motion model (Demo-1 placeholders) ====
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        float dt = (now_ms - last_ms) / 1000.0f; last_ms = now_ms;
        float t  = now_ms / 1000.0f;

        // --- Replace these with real values later (encoders + IMU) ---
        float mps      = 0.34f + 0.05f * sinf(t);              // speed (m/s)
        float rpm_l    = 118.0f, rpm_r = 119.0f;               // wheel RPM
        float heading  = 90.0f + 2.5f * sinf(0.4f * t);        // heading (deg)
        float offset_mm= 5.0f * sinf(0.8f * t);                // line pos
        float pid_err  = clampf(offset_mm/20.0f, -1.0f, 1.0f); // PID error (-1..1)

        distance_m += mps * dt;

        // ---- Demo-1 telemetry streams (continuous) ----
        if (due_ms(&t_speed,   200)) tel_publish_speed(mps, rpm_l, rpm_r);
        if (due_ms(&t_heading, 200)) tel_publish_heading(heading);
        if (due_ms(&t_filt,    200)) tel_publish_imu_filtered(0,0,0, heading);
        if (due_ms(&t_raw,    1000)) tel_publish_imu_raw(0.02f*sinf(3*t),0,9.81f, 0.01f,0.01f,0.02f, 0,0,0);
        if (due_ms(&t_distance,1000)) tel_publish_distance(distance_m);

        // Publish line position (useful in phases where following/centering matters)
        if (due_ms(&t_linepos, 200)) tel_publish_linepos(offset_mm, pid_err);

        // ===== Integrated Demo-2 & Demo-3 sequence =====
        if (phase != last_phase) { // on-enter hook for each phase
            switch (phase) {
                case 0: tel_publish_state("LINE_FOLLOW"); break;
                case 1: tel_publish_state("TURN_LEFT");   break;
                case 2: tel_publish_state("LINE_FOLLOW"); break;
                case 3: tel_publish_state("TURN_RIGHT");  break;
                case 4: tel_publish_state("LINE_FOLLOW"); break;
                case 6: tel_publish_state("DETOUR");      break; // refined below after decision
                case 7: tel_publish_state("LINE_FOLLOW"); break;
                default: break;
            }
            last_phase = phase;
            phase_start = now_ms;
        }

        uint32_t elapsed = now_ms - phase_start;

        switch (phase) {
            case 0: // follow 4s then barcode LEFT -> turn-left
                if (elapsed > 4000) {
                    tel_publish_barcode("LEFT", junction++);
                    tel_publish_state("TURN_LEFT");
                    phase = 1;
                }
                break;

            case 1: // turning ~2.5s then back to follow
                if (elapsed > 2500) {
                    tel_publish_state("LINE_FOLLOW");
                    phase = 2;
                }
                break;

            case 2: // follow 4s then barcode RIGHT -> turn-right
                if (elapsed > 4000) {
                    tel_publish_barcode("RIGHT", junction++);
                    tel_publish_state("TURN_RIGHT");
                    phase = 3;
                }
                break;

            case 3: // turning ~2.5s then follow
                if (elapsed > 2500) {
                    tel_publish_state("LINE_FOLLOW");
                    phase = 4;
                }
                break;

            case 4: // follow 3s then encounter obstacle
                if (elapsed > 3000) {
                    phase = 5; // go to scan+decision
                }
                break;

            case 5: { // one sweep -> publish scan & decision, set detour side
                float left_cm  = 24.0f + 5.0f * sinf(0.5f*t);
                float right_cm = 12.0f + 3.0f * sinf(0.7f*t);
                tel_publish_obstacle_scan(left_cm, right_cm);

                const char* side = (right_cm > left_cm) ? "right" : "left";
                float width_cm   = fabsf(right_cm - left_cm);
                tel_publish_obstacle_decision(side, width_cm);
                tel_publish_state((right_cm > left_cm) ? "DETOUR_RIGHT" : "DETOUR_LEFT");

                phase = 6; // detouring
                break; }

            case 6: // detour ~3s, then rejoin
                if (elapsed > 3000) {
                    phase = 7; // rejoin
                }
                break;

            case 7: // rejoin & continue for 5s, then loop
                if (elapsed < 200) { // fire once at entry
                    tel_publish_recovery("REJOINED_LINE");
                }
                if (elapsed > 5000) {
                    phase = 0; // loop the whole show again
                }
                break;

            default: phase = 0; break;
        }

        sleep_ms(5);
    }
}
