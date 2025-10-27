#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"

#define BAUD_RATE 115200
#define DIGITAL_PIN 26

#define BARCODE_BUF_SIZE 10
#define BARCODE_ARR_SIZE 9  // Collect exactly 9 bars like the working analog version

uint8_t barcodeFirstChar=0;
uint8_t barcodeSecondChar=0;
uint8_t barcodeThirdChar=0;

enum bartype{
    THICK_BLACK, // 0
    THIN_BLACK, // 1
    THICK_WHITE, // 2
    THIN_WHITE // 3
};

static char* A_ARRAY_MAP = "031213130";
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

    printf("[DEBUG] Pattern: %s\n\r", string);

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
    
    for(int i = 0; i < 27; i++){
        if(strncmp(barcodes[i], string, BARCODE_ARR_SIZE) == 0){
            free(string);
            free(barsRead);
            return characters[i];
        }
    }

    free(string);
    free(barsRead);
    printf("[DEBUG] No character match found\n\r");
    return 0;
}

// GPIO interrupt handler for digital pin transitions
void gpio_callback(uint gpio, uint32_t events) {
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

int main() {
    stdio_init_all();
    sleep_ms(3000);

    printf("\n\r");
    printf("========================================\n\r");
    printf("  DIGITAL PIN BARCODE SCANNER\n\r");
    printf("========================================\n\r");
    printf("Using GPIO %d (Digital Pin)\n\r", DIGITAL_PIN);
    printf("Black line = HIGH (1)\n\r");
    printf("White space = LOW (0)\n\r");
    printf("\n\r");
    printf("Pass sensor over barcode...\n\r");
    printf("Press 'S' to restart scanning\n\r");
    printf("========================================\n\r\n\r");

    // Initialize GPIO for digital pin
    gpio_init(DIGITAL_PIN);
    gpio_set_dir(DIGITAL_PIN, GPIO_IN);
    
    // Enable interrupt on both edges (rising and falling)
    gpio_set_irq_enabled_with_callback(DIGITAL_PIN, 
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
                                       true, 
                                       &gpio_callback);

    resetScanner();

    while (true) {
        // Check if ready to decode (set by interrupt)
        if(ready_to_decode){
            ready_to_decode = false;
            
            printf("[INFO] Decoding...\n\r");
            char decoded = compareTwoArray();
            
            if(decoded != 0){
                printf("[DECODED] Character: %c\n\r", decoded);
                appendToBarcodeRead(decoded);
            }
            
            printf("[INFO] Scanning stopped. Press 'S' to scan again.\n\r\n\r");
        }
        
        // Check for 'S' key to restart
        int c = getchar_timeout_us(0);
        if(c == 's' || c == 'S'){
            if(!scanning_enabled){
                printf("\n\r[INFO] Restarting scanner...\n\r");
                scanning_enabled = true;
                resetScanner();
                clearBarcodeRead();
                printf("[INFO] Ready to scan!\n\r\n\r");
            }
        }

        // Check for valid barcode
        if(isValidBarcode()){
            printf("\n\r****************************************\n\r");
            printf("*** VALID BARCODE DETECTED! ***\n\r");
            printf("****************************************\n\r");
            barcodeFirstChar = barcodeRead[0];
            barcodeSecondChar = barcodeRead[1];
            barcodeThirdChar = barcodeRead[2];
            printf("Barcode: %c%c%c\n\r", barcodeFirstChar, barcodeSecondChar, barcodeThirdChar);
            printf("****************************************\n\r\n\r");
            clearBarcodeRead();
            barcodeFirstChar = 0;
            barcodeSecondChar = 0;
            barcodeThirdChar = 0;
        }
        
        sleep_ms(10);
    }
}
