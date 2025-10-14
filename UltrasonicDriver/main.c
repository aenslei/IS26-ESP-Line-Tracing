
#include <stdio.h>
#include "pico/stdlib.h"
#include "ultrasonic_interrupt.h"

#define TRIG_PIN 1  // GP1
#define ECHO_PIN 0  // GP0

int main() {
    stdio_init_all();
    ultrasonic_init(TRIG_PIN, ECHO_PIN);

    while (true) {
        if (ultrasonic_trigger()) {
            // Trigger sent
        }

        uint16_t distance;
        if (ultrasonic_read_cm(&distance)) {
            printf("Distance: %u cm\n", distance);
        }

        sleep_ms(200);  // Wait before next trigger
    }
}
