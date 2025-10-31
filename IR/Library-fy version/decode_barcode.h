#ifndef DECODE_BARCODE
#define DECODE_BARCODE

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

struct barTransition {
    int isBlack;              // 1 = black, 0 = white
    int64_t duration_us;      // Duration in microseconds
    enum bartype type;
};

static struct barTransition barTransitions[BARCODE_ARR_SIZE];
static char barcodeRead[3] = {0};

void init_pin_and_button(void);
static void appendToBarcodeRead(char barcodeChar);
static int isValidBarcode();
static void clearBarcodeRead(); 
//static char *intArrayToString(int *arr, int size);
//static int* thickThinClassification();
static void resetScanner();
static char compareTwoArray();
const char* char_to_command(char decoded_char);
static bool btn_to_reset_scanner(); //reset button 



#endif 