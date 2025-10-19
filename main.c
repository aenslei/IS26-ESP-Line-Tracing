#include <stdio.h>
#include "pico/stdlib.h"
#include "functions.h"

#define WIFI_SSID "SINGTEL-HOME(2.4GHz)"
#define WIFI_PASS "FHWIba#24"

// Your Windows laptop running Mosquitto:
#define BROKER_IP   "192.168.1.4"
#define BROKER_PORT 1883

int main() {
    stdio_init_all();
    sleep_ms(1000);

    if (!wifi_connect(WIFI_SSID, WIFI_PASS)) {
        while (1) tight_loop_contents();
    }

    mqtt_start(BROKER_IP, BROKER_PORT, "teamA", "car1");

    absolute_time_t last = get_absolute_time();
    while (1) {
        // 1 Hz heartbeat
        if (absolute_time_diff_us(last, get_absolute_time()) >= 1000000) {
            last = get_absolute_time();
            mqtt_publish_heartbeat();
        }
        sleep_ms(10);
    }
}
