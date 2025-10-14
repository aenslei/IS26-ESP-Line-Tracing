#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

// === IR sensor pin setup ===
#define IR_L 27   // ADC1
#define IR_C 26   // ADC0
#define IR_R 28   // ADC2
#define CH_L 1
#define CH_C 0
#define CH_R 2

#define THRESHOLD 2000   // adjust by calibration

int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("=== Simple 3-IR Line Sensor ===\n");

    // Initialize ADC
    adc_init();
    adc_gpio_init(IR_L);
    adc_gpio_init(IR_C);
    adc_gpio_init(IR_R);

    while (true) {
        // Read 3 IR sensors
        adc_select_input(CH_L);
        uint16_t left = adc_read();

        adc_select_input(CH_C);
        uint16_t center = adc_read();

        adc_select_input(CH_R);
        uint16_t right = adc_read();

        // Decide surface (Black / White)
        const char *L_surf = (left   > THRESHOLD) ? "Black" : "White";
        const char *C_surf = (center > THRESHOLD) ? "Black" : "White";
        const char *R_surf = (right  > THRESHOLD) ? "Black" : "White";

        // Print result
        printf("L=%4u (%s) | C=%4u (%s) | R=%4u (%s)\n",
               left, L_surf, center, C_surf, right, R_surf);

        sleep_ms(200);
    }
}
