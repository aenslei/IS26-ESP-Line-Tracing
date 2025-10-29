#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#define BAUD_RATE 115200
#define DIGITAL_PIN 1        // GP1 for barcode scanning (digital)
#define LINE_ADC_PIN 28      // GP28 for line following (ADC2)
#define RESET_BUTTON_PIN 21  // GP21 as reset button

#define BARCODE_BUF_SIZE 10
#define BARCODE_ARR_SIZE 9  // Collect exactly 9 bars like the working analog version

#define LINE_THRESHOLD 400   // ADC threshold for black/white detection

uint8_t barcodeFirstChar=0;
uint8_t barcodeSecondChar=0;
uint8_t barcodeThirdChar=0;

enum bartype{
    THICK_BLACK, // 0
    THIN_BLACK, // 1
    THICK_WHITE, // 2
    THIN_WHITE // 3
};

static char* A_ARRAY_MAP = "031312130";
static char* B_ARRAY_MAP = "130213130";
static char* C_ARRAY_MAP = "030213131";
static char* D_ARRAY_MAP = "131203130";
static char* E_ARRAY_MAP = "031203131";
static char* F_ARRAY_MAP = "130203131";
static char* G_ARRAY_MAP = "131213030";
static char* H_ARRAY_MAP = "031213031";
static char* I_ARRAY_MAP = "130213031";
static char* J_ARRAY_MAP = "131203031";
static char* K_ARRAY_MAP = "031213120";
static char* L_ARRAY_MAP = "130213120";
static char* M_ARRAY_MAP = "030213121";
static char* N_ARRAY_MAP = "131203120";
static char* O_ARRAY_MAP = "031203121";
static char* P_ARRAY_MAP = "130203121";
static char* Q_ARRAY_MAP = "131213020";
static char* R_ARRAY_MAP = "031213021";
static char* S_ARRAY_MAP = "130213021";
static char* T_ARRAY_MAP = "131203021";
static char* U_ARRAY_MAP = "021213130";
static char* V_ARRAY_MAP = "120213130";
static char* W_ARRAY_MAP = "020213131";
static char* X_ARRAY_MAP = "121203130";
static char* Y_ARRAY_MAP = "021203131";
static char* Z_ARRAY_MAP = "120303131";
static char* ASTERISK_ARRAY_MAP = "121303031";

static int barcode_arr_index = 0;
static absolute_time_t transition_time;
static int last_state = -1;
static uint32_t transition_count = 0;
static volatile bool scanning_enabled = true;
static volatile bool processing = false;  // Prevent interrupts during decode
static volatile bool ready_to_decode = false;  // Signal main loop to decode
static volatile bool waiting_for_start = true;  // NEW: waiting for * delimiter

// Button debounce variables
static absolute_time_t last_button_press_time;
static const uint32_t DEBOUNCE_DELAY_MS = 200;  // 200ms debounce

// Line following variables
static uint16_t line_adc_value = 0;
static bool on_line = false;  // true = black line detected, false = white background

struct barTransition {
    int isBlack;              // 1 = black, 0 = white
    int64_t duration_us;      // Duration in microseconds
    enum bartype type;
};

static struct barTransition barTransitions[BARCODE_ARR_SIZE];
static char barcodeRead[3] = {0};

static void appendToBarcodeRead(char barcodeChar){
    barcodeRead[0] = barcodeRead[1];
    barcodeRead[1] = barcodeRead[2];
    barcodeRead[2] = barcodeChar;
}

static int isValidBarcode(){
    if(barcodeRead[0] == '*' && barcodeRead[2] == '*'){
        if(barcodeRead[1] != 0)
            return 1;
    }
    return 0;
}

static void clearBarcodeRead(){
    barcodeRead[0] = 0;
    barcodeRead[1] = 0;
    barcodeRead[2] = 0;
}

static char *intArrayToString(int *arr, int size){
    char *str = malloc(size + 1);
    for(int i = 0; i < size; i++){
        str[i] = arr[i] + '0';
    }
    str[size] = '\0';
    return str;
}

// NEW: Function to reverse a string
static char* reverseString(const char* str){
    int len = strlen(str);
    char* reversed = malloc(len + 1);
    for(int i = 0; i < len; i++){
        reversed[i] = str[len - 1 - i];
    }
    reversed[len] = '\0';
    return reversed;
}

static int* thickThinClassification(){
    // Use all 9 bars for classification (like the working analog version)
    typedef struct {
        int64_t length;
        int index;
    } BarLength;
    
    BarLength lengths[BARCODE_ARR_SIZE];
    
    // Collect all bar lengths
    for(int i = 0; i < BARCODE_ARR_SIZE; i++){
        lengths[i].length = barTransitions[i].duration_us;
        lengths[i].index = i;
    }
    
    // Sort by length
    for(int i = 0; i < BARCODE_ARR_SIZE - 1; i++){
        for(int j = i + 1; j < BARCODE_ARR_SIZE; j++){
            if(lengths[i].length > lengths[j].length){
                BarLength temp = lengths[i];
                lengths[i] = lengths[j];
                lengths[j] = temp;
            }
        }
    }
    
    // In Code 39: 6 narrow bars, 3 wide bars
    // Threshold is between the 6th and 7th sorted values
    int64_t threshold = (lengths[5].length + lengths[6].length) / 2;

    printf("[DEBUG] Threshold: %"PRId64" us\n\r", threshold);

    int *barsRead = malloc(BARCODE_ARR_SIZE * sizeof(int));

    for(int i = 0; i < BARCODE_ARR_SIZE; i++){
        if(barTransitions[i].isBlack){
            if(barTransitions[i].duration_us < threshold){
                barTransitions[i].type = THIN_BLACK;
                barsRead[i] = 1;
            }else{
                barTransitions[i].type = THICK_BLACK;
                barsRead[i] = 0;
            }
        }else{
            if(barTransitions[i].duration_us < threshold){
                barTransitions[i].type = THIN_WHITE;
                barsRead[i] = 3;
            }else{
                barTransitions[i].type = THICK_WHITE;
                barsRead[i] = 2;
            }
        }
    }
    
    return barsRead;
}

static void resetScanner(){
    barcode_arr_index = 0;
    transition_count = 0;
    last_state = -1;
    processing = false;
    ready_to_decode = false;
    
    for(int i = 0; i < BARCODE_ARR_SIZE; i++){
        barTransitions[i].isBlack = 0;
        barTransitions[i].duration_us = 0;
        barTransitions[i].type = 0;
    }
}

static char compareTwoArray() {
    int* barsRead = thickThinClassification();

    char* string = intArrayToString(barsRead, BARCODE_ARR_SIZE);

    printf("[DEBUG] Pattern (forward): %s\n\r", string);

    char* barcodes[] = {
        A_ARRAY_MAP, B_ARRAY_MAP, C_ARRAY_MAP, D_ARRAY_MAP,
        E_ARRAY_MAP, F_ARRAY_MAP, G_ARRAY_MAP, H_ARRAY_MAP,
        I_ARRAY_MAP, J_ARRAY_MAP, K_ARRAY_MAP, L_ARRAY_MAP,
        M_ARRAY_MAP, N_ARRAY_MAP, O_ARRAY_MAP, P_ARRAY_MAP,
        Q_ARRAY_MAP, R_ARRAY_MAP, S_ARRAY_MAP, T_ARRAY_MAP,
        U_ARRAY_MAP, V_ARRAY_MAP, W_ARRAY_MAP, X_ARRAY_MAP,
        Y_ARRAY_MAP, Z_ARRAY_MAP, ASTERISK_ARRAY_MAP   
    };

    char characters[] = {
        'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I',
        'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R',
        'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '*'
    };
    
    // Try forward direction first
    for(int i = 0; i < 27; i++){
        if(strncmp(barcodes[i], string, BARCODE_ARR_SIZE) == 0){
            free(string);
            free(barsRead);
            printf("[DEBUG] Matched in FORWARD direction\n\r");
            return characters[i];
        }
    }

    // If no match, try reverse direction
    printf("[DEBUG] No forward match, trying reverse...\n\r");
    char* reversed = reverseString(string);
    printf("[DEBUG] Pattern (reverse): %s\n\r", reversed);
    
    for(int i = 0; i < 27; i++){
        if(strncmp(barcodes[i], reversed, BARCODE_ARR_SIZE) == 0){
            free(string);
            free(reversed);
            free(barsRead);
            printf("[DEBUG] Matched in REVERSE direction\n\r");
            return characters[i];
        }
    }

    free(string);
    free(reversed);
    free(barsRead);
    printf("[DEBUG] No character match found in either direction\n\r");
    return 0;
}

// GPIO interrupt handler for digital pin transitions (barcode scanning)
void gpio_callback(uint gpio, uint32_t events) {
    if(gpio == DIGITAL_PIN) {
        // Barcode sensor interrupt
        if(!scanning_enabled || processing) return;  // Exit immediately if processing
        
        // Read current state: 1 = BLACK, 0 = WHITE
        int current_state = gpio_get(DIGITAL_PIN);
        
        // First reading - just record the state and time
        if(last_state == -1){
            last_state = current_state;
            transition_time = get_absolute_time();
            printf("[START] Initial state: %s\n\r", current_state ? "BLACK" : "WHITE");
            return;
        }
        
        // State changed - we have a transition!
        if(current_state != last_state){
            absolute_time_t now = get_absolute_time();
            int64_t duration = absolute_time_diff_us(transition_time, now);
            
            if(duration < 0) duration = -duration;
            
            // Store the bar that just ended
            if(barcode_arr_index < BARCODE_ARR_SIZE){
                barTransitions[barcode_arr_index].isBlack = last_state;
                barTransitions[barcode_arr_index].duration_us = duration;
                
                transition_count++;
                printf("[TRANSITION %"PRIu32"] %s bar lasted %"PRId64" us\n\r",
                       transition_count,
                       last_state ? "BLACK" : "WHITE",
                       duration);
                
                barcode_arr_index++;
                
                // Check if we've collected 9 bars (one complete character)
                if(barcode_arr_index == BARCODE_ARR_SIZE){
                    printf("\n\r[INFO] 9 bars collected!\n\r");
                    scanning_enabled = false;
                    ready_to_decode = true;  // Signal main loop to decode
                }
            }
            
            // Update for next transition
            last_state = current_state;
            transition_time = now;
        }
    }
}

// Check if button is pressed with debouncing
static bool isButtonPressed(){
    if(gpio_get(RESET_BUTTON_PIN) == 0){  // Button pressed (active LOW)
        absolute_time_t now = get_absolute_time();
        int64_t time_since_last_press = absolute_time_diff_us(last_button_press_time, now) / 1000;  // Convert to ms
        
        if(time_since_last_press > DEBOUNCE_DELAY_MS){
            last_button_press_time = now;
            return true;
        }
    }
    return false;
}

// Read line sensor and update state
static void readLineSensor(){
    // Select ADC2 (GP28)
    adc_select_input(2);
    sleep_us(100);  // Small delay for ADC to settle
    
    // Read ADC value
    line_adc_value = adc_read();
    
    // Determine if on line (black) or background (white)
    on_line = (line_adc_value >= LINE_THRESHOLD);
}

// Get line sensor status string
static const char* getLineSensorStatus(){
    return on_line ? "BLACK/LINE" : "WHITE/BG";
}

int main() {
    stdio_init_all();
    sleep_ms(3000);

    printf("\n\r");
    printf("========================================\n\r");
    printf("  BARCODE SCANNER + LINE FOLLOWER\n\r");
    printf("========================================\n\r");
    printf("Barcode: GPIO %d (Digital Pin)\n\r", DIGITAL_PIN);
    printf("Line Sensor: GPIO %d (ADC2)\n\r", LINE_ADC_PIN);
    printf("Reset Button: GPIO %d\n\r", RESET_BUTTON_PIN);
    printf("\n\r");
    printf("Barcode: Black=HIGH, White=LOW\n\r");
    printf("Line: Threshold=%d ADC units\n\r", LINE_THRESHOLD);
    printf("\n\r");
    printf("Scans in BOTH directions!\n\r");
    printf("Format: *X* (where X is a character)\n\r");
    printf("Press GP21 button to reset scanner\n\r");
    printf("Press 'R' for manual reset via serial\n\r");
    printf("Press 'L' to toggle line sensor display\n\r");
    printf("========================================\n\r\n\r");

    // Initialize ADC for line sensor
    adc_init();
    adc_gpio_init(LINE_ADC_PIN);
    printf("ADC initialized for GP%d\n\r", LINE_ADC_PIN);

    // Initialize GPIO for digital pin (barcode sensor)
    gpio_init(DIGITAL_PIN);
    gpio_set_dir(DIGITAL_PIN, GPIO_IN);
    printf("GPIO %d initialized for barcode\n\r", DIGITAL_PIN);
    
    // Initialize GPIO for reset button
    gpio_init(RESET_BUTTON_PIN);
    gpio_set_dir(RESET_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(RESET_BUTTON_PIN);  // Enable pull-up resistor (button connects to GND)
    printf("GPIO %d initialized for reset button\n\r\n\r", RESET_BUTTON_PIN);
    
    // Enable interrupt on both edges for barcode sensor (rising and falling)
    gpio_set_irq_enabled_with_callback(DIGITAL_PIN, 
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
                                       true, 
                                       &gpio_callback);

    // Initialize button debounce timer
    last_button_press_time = get_absolute_time();

    resetScanner();
    
    printf("System ready!\n\r\n\r");

    bool show_line_sensor = false;  // Toggle for line sensor display
    absolute_time_t last_line_display = get_absolute_time();

    while (true) {
        // Read line sensor continuously
        readLineSensor();
        
        // Display line sensor data if enabled (every 500ms)
        if(show_line_sensor){
            absolute_time_t now = get_absolute_time();
            int64_t time_since_display = absolute_time_diff_us(last_line_display, now) / 1000;  // Convert to ms
            if(time_since_display > 500){
                printf("[LINE] ADC: %4d, Status: %s\n\r", line_adc_value, getLineSensorStatus());
                last_line_display = now;
            }
        }
        
        // Check if ready to decode (set by interrupt)
        if(ready_to_decode){
            ready_to_decode = false;
            
            printf("[INFO] Decoding...\n\r");
            char decoded = compareTwoArray();
            
            if(decoded != 0){
                printf("[DECODED] Character: %c\n\r", decoded);
                
                // Check if we're waiting for the start delimiter
                if(waiting_for_start){
                    // ONLY accept '*' as the first character
                    if(decoded == '*'){
                        printf("[INFO] Start delimiter '*' detected! Waiting for data character...\n\r");
                        waiting_for_start = false;
                        appendToBarcodeRead(decoded);
                        
                        // Continue scanning for next character
                        printf("[INFO] Continuing scan...\n\r\n\r");
                        scanning_enabled = true;
                        resetScanner();
                    } else {
                        // Not a start delimiter - restart scanner
                        printf("[WARNING] Expected '*' but got '%c'. Restarting scanner...\n\r\n\r", decoded);
                        scanning_enabled = true;
                        resetScanner();
                        clearBarcodeRead();
                        waiting_for_start = true;
                    }
                } else {
                    // We've already seen the start '*', now collect data or end delimiter
                    appendToBarcodeRead(decoded);
                    
                    if(decoded == '*'){
                        // It's the end delimiter
                        printf("[INFO] End delimiter '*' detected!\n\r");
                        waiting_for_start = true;  // Reset for next barcode
                    } else {
                        // It's a data character
                        printf("[INFO] Data character collected.\n\r");
                    }
                    
                    // Continue scanning
                    printf("[INFO] Continuing scan...\n\r\n\r");
                    scanning_enabled = true;
                    resetScanner();
                }
            } else {
                // Failed to decode, reset and try again
                printf("[INFO] Decode failed. Restarting scanner...\n\r\n\r");
                scanning_enabled = true;
                resetScanner();
                clearBarcodeRead();
                waiting_for_start = true;
            }
        }
        
        // Check for GP21 button press to reset
        if(isButtonPressed()){
            printf("\n\r[BUTTON] Reset button pressed!\n\r");
            scanning_enabled = true;
            waiting_for_start = true;
            resetScanner();
            clearBarcodeRead();
            printf("[INFO] Scanner reset via button!\n\r\n\r");
        }
        
        // Check for 'R' key to completely reset (serial command backup)
        int c = getchar_timeout_us(0);
        if(c == 'r' || c == 'R'){
            printf("\n\r[INFO] Manual reset...\n\r");
            scanning_enabled = true;
            waiting_for_start = true;
            resetScanner();
            clearBarcodeRead();
            printf("[INFO] Ready to scan!\n\r\n\r");
        }
        else if(c == 'l' || c == 'L'){
            show_line_sensor = !show_line_sensor;
            printf("\n\r[INFO] Line sensor display %s\n\r\n\r", show_line_sensor ? "ENABLED" : "DISABLED");
        }

        // Check for valid barcode (complete *X* pattern)
        if(isValidBarcode()){
            printf("\n\r****************************************\n\r");
            printf("*** VALID BARCODE DETECTED! ***\n\r");
            printf("****************************************\n\r");
            barcodeFirstChar = barcodeRead[0];
            barcodeSecondChar = barcodeRead[1];
            barcodeThirdChar = barcodeRead[2];
            printf("Barcode: %c%c%c\n\r", barcodeFirstChar, barcodeSecondChar, barcodeThirdChar);
            printf("Data character: %c\n\r", barcodeSecondChar);
            printf("****************************************\n\r\n\r");
            
            // Clear for next barcode but keep scanning
            clearBarcodeRead();
            waiting_for_start = true;
        }
        
        sleep_ms(10);
    }
}
