#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#define BAUD_RATE 115200
#define ADC_PIN 26
#define DIGITAL_PIN 22

// Sensor readings: Below 400 = WHITE (background), Above 400 = BLACK (line)
#define WHITE_THRESHOLD 400
#define ADC_DIFFERENCE_THRESHOLD 50

#define SAMPLE_SIZE 100

// Line following states
typedef enum {
    ON_LINE,      // Sensor is on the black line
    OFF_LINE,     // Sensor is on white background
    UNKNOWN       // Initial state
} line_state_t;

static uint32_t res = 0;
static uint16_t prevAvg = 0;
static int sample_count = 0;
static line_state_t current_state = UNKNOWN;
static line_state_t previous_state = UNKNOWN;

// Statistics
static uint32_t total_samples = 0;
static uint32_t line_transitions = 0;

// Get current line state
line_state_t get_line_state() {
    return current_state;
}

// Get smoothed ADC reading
uint16_t get_adc_value() {
    return prevAvg;
}

// Check if sensor is on the line
bool is_on_line() {
    return current_state == ON_LINE;
}

// ADC interrupt handler
static void ADC_IRQ_FIFO_HANDLER() {
    if(!adc_fifo_is_empty()){
        uint16_t data = adc_fifo_get();
        
        res += data;
        
        if(sample_count < SAMPLE_SIZE){
            sample_count++;
        } else {
            // Calculate average
            uint16_t avg = res / sample_count;
            total_samples++;

            // Smooth the reading to reduce noise
            if(prevAvg == 0){
                prevAvg = avg;
            } else {
                if(abs(prevAvg - avg) > ADC_DIFFERENCE_THRESHOLD){
                    prevAvg = avg;
                } else {
                    avg = prevAvg;
                }
            }
            
            // Reset for next sample batch
            sample_count = 0;
            res = 0;

            // Determine line state using both ADC and digital pin
            previous_state = current_state;
            
            if(avg >= WHITE_THRESHOLD || gpio_get(DIGITAL_PIN) == 1){
                current_state = ON_LINE;  // Black line detected
            } else {
                current_state = OFF_LINE;  // White background
            }

            // Detect state transitions
            if(previous_state != UNKNOWN && previous_state != current_state){
                line_transitions++;
                printf("[TRANSITION %"PRIu32"] %s -> %s (ADC: %"PRIu16", Digital: %d)\n\r",
                       line_transitions,
                       previous_state == ON_LINE ? "LINE" : "BACKGROUND",
                       current_state == ON_LINE ? "LINE" : "BACKGROUND",
                       avg,
                       gpio_get(DIGITAL_PIN));
            }

            // Periodic status output (every 100 samples)
            if(total_samples % 100 == 0){
                printf("[STATUS] State: %s, ADC: %"PRIu16", Digital: %d\n\r",
                       current_state == ON_LINE ? "ON_LINE" : "OFF_LINE",
                       avg,
                       gpio_get(DIGITAL_PIN));
            }
        }
    }
    irq_clear(ADC_IRQ_FIFO);
}

// Initialize the line sensor
void line_sensor_init() {
    // Initialize ADC
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    // Initialize digital pin
    gpio_init(DIGITAL_PIN);
    gpio_set_dir(DIGITAL_PIN, GPIO_IN);

    // Setup ADC FIFO and interrupts
    adc_fifo_setup(true, false, 1, false, false);
    adc_set_clkdiv(0);
    adc_irq_set_enabled(true);

    irq_clear(ADC_IRQ_FIFO);
    irq_set_exclusive_handler(ADC_IRQ_FIFO, ADC_IRQ_FIFO_HANDLER);
    irq_set_enabled(ADC_IRQ_FIFO, true);
    
    // Start ADC conversion
    adc_run(true);
}

// Calibration function - call this while sensor is on white background
void calibrate_white() {
    printf("[CALIBRATE] Place sensor on WHITE background...\n\r");
    sleep_ms(2000);
    
    uint32_t sum = 0;
    int readings = 0;
    
    for(int i = 0; i < 50; i++){
        uint16_t val = get_adc_value();
        if(val > 0){
            sum += val;
            readings++;
        }
        sleep_ms(20);
    }
    
    if(readings > 0){
        uint16_t white_avg = sum / readings;
        printf("[CALIBRATE] White background average: %"PRIu16"\n\r", white_avg);
        printf("[CALIBRATE] Current threshold: %d\n\r", WHITE_THRESHOLD);
    }
}

// Calibration function - call this while sensor is on black line
void calibrate_black() {
    printf("[CALIBRATE] Place sensor on BLACK line...\n\r");
    sleep_ms(2000);
    
    uint32_t sum = 0;
    int readings = 0;
    
    for(int i = 0; i < 50; i++){
        uint16_t val = get_adc_value();
        if(val > 0){
            sum += val;
            readings++;
        }
        sleep_ms(20);
    }
    
    if(readings > 0){
        uint16_t black_avg = sum / readings;
        printf("[CALIBRATE] Black line average: %"PRIu16"\n\r", black_avg);
        printf("[CALIBRATE] Current threshold: %d\n\r", WHITE_THRESHOLD);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(3000);

    printf("\n\r");
    printf("=========================================\n\r");
    printf("   IR LINE FOLLOWING SENSOR\n\r");
    printf("=========================================\n\r");
    printf("Commands:\n\r");
    printf("  'W' - Calibrate white background\n\r");
    printf("  'B' - Calibrate black line\n\r");
    printf("  'S' - Show statistics\n\r");
    printf("=========================================\n\r\n\r");

    // Initialize the line sensor
    line_sensor_init();
    
    sleep_ms(1000);
    printf("[INFO] Line sensor ready!\n\r\n\r");

    while (true) {
        // Check for commands
        int c = getchar_timeout_us(0);
        
        if(c == 'w' || c == 'W'){
            calibrate_white();
        }
        else if(c == 'b' || c == 'B'){
            calibrate_black();
        }
        else if(c == 's' || c == 'S'){
            printf("\n\r[STATISTICS]\n\r");
            printf("Total samples: %"PRIu32"\n\r", total_samples);
            printf("Line transitions: %"PRIu32"\n\r", line_transitions);
            printf("Current state: %s\n\r", 
                   current_state == ON_LINE ? "ON_LINE" : "OFF_LINE");
            printf("Current ADC: %"PRIu16"\n\r\n\r", prevAvg);
        }

        // Your line following logic would go here
        // Example:
        // if(is_on_line()) {
        //     // Motor control: go straight
        // } else {
        //     // Motor control: turn to find line
        // }
        
        sleep_ms(10);
    }

    return 0;
}
