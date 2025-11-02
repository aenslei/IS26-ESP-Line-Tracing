#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#define LINE_ADC_PIN 27
#define LINE_DIGITAL_PIN 21

int main() {
    stdio_init_all();
    sleep_ms(3000);

    printf("\n\r========================================\n\r");
    printf("GP27 ADC DIAGNOSTIC TEST\n\r");
    printf("========================================\n\r\n\r");

    // Initialize ADC
    adc_init();
    
    // Initialize GP27 as ADC input
    adc_gpio_init(LINE_ADC_PIN);
    
    // Initialize digital pin
    gpio_init(LINE_DIGITAL_PIN);
    gpio_set_dir(LINE_DIGITAL_PIN, GPIO_IN);
    
    printf("GP27 initialized as ADC1\n\r");
    printf("GP21 initialized as digital input\n\r\n\r");
    
    printf("Reading sensors every 500ms...\n\r");
    printf("Press any key to exit\n\r\n\r");

    while(true) {
        // Select ADC1 (GP27)
        adc_select_input(1);
        sleep_us(100); // Small delay for ADC to settle
        
        // Read ADC value
        uint16_t adc_value = adc_read();
        
        // Read digital pin
        int digital_value = gpio_get(LINE_DIGITAL_PIN);
        
        printf("[GP27] ADC: %4d, Digital(GP21): %d, Status: %s\n\r",
               adc_value,
               digital_value,
               adc_value >= 400 ? "BLACK/LINE" : "WHITE/BG");
        
        sleep_ms(500);
        
        // Check for exit
        int c = getchar_timeout_us(0);
        if(c != PICO_ERROR_TIMEOUT) {
            break;
        }
    }
    
    printf("\n\rTest completed.\n\r");
    return 0;
}
