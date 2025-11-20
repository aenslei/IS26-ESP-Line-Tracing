#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

// Include our libraries
#include "WithPID.h"
#include "IMUDriver.h"
#include "IR_track.h"
#include "decode_barcode.h"

// ========== FSM DATA STRUCTURES ==========

typedef enum {
    STATE_INIT,
    STATE_LINE_FOLLOWING,     // Normal line following
    STATE_CORRECTION_LEFT,    // Robot drifted right, need to turn left (slow right motor)
    STATE_CORRECTION_RIGHT,   // Robot drifted left, need to turn right (slow left motor) 
    STATE_PRE_TURN,           // Double-check state to confirm if it's a real turn or just correction
    STATE_TURNING_LEFT,       // Sharp left turn or intersection detected
    STATE_TURNING_RIGHT,      // Sharp right turn or intersection detected
    STATE_BARCODE_DETECTED,   // Barcode start detected - slow down for decoding
    STATE_BARCODE_DECODING,   // Reading and decoding barcode data
    STATE_BARCODE_EXECUTING,  // Executing barcode command (direction change)
    STATE_STOPPED
} fsm_state_t;

typedef struct {
    float line_error;        // IR sensor error (-1.0 to 1.0)
    bool line_detected;      // True if line is visible
    float imu_heading;       // Current heading from IMU
    bool barcode_detected;   // Barcode sensor state
    uint32_t timestamp_ms;   // When sensors were last read
    
    // Pattern detection variables
    uint16_t raw_adc_value;  // Raw IR sensor value for intensity analysis
    float heading_rate;      // Rate of heading change (degrees/second)
    uint32_t error_start_time; // When large error first detected
    float initial_heading;   // Heading when large error started
    bool rapid_turn_detected; // Ultra-fast turn signature detection
    
    // Barcode detection and decoding
    char barcode_buffer[16]; // Buffer for barcode data (*X* format)
    uint8_t barcode_index;   // Current position in buffer
    bool barcode_start_found; // True if '*' start delimiter found
    bool barcode_complete;   // True if full barcode (*X*) decoded
    char barcode_command;    // Decoded command character
    float target_heading;    // Target heading for barcode command execution
} sensor_data_t;

// ========== GLOBAL FSM STATE ==========
static fsm_state_t current_state = STATE_INIT;
static sensor_data_t sensors = {0};
static uint32_t state_entry_time = 0;

// ========== PATTERN DETECTION THRESHOLDS ==========
#define CORRECTION_ERROR_THRESHOLD 0.15f   // Small error for corrections
#define TURN_ERROR_THRESHOLD 0.35f         // Large error indicating potential turn  
#define CORRECTION_TIME_LIMIT 150          // Corrections should be < 150ms

// ========== BARCODE DETECTION THRESHOLDS ==========
#define BARCODE_THRESHOLD 1500             // ADC threshold for detecting barcode patterns
#define BARCODE_DECODE_SPEED 0.3f          // Slow speed during barcode decoding
#define BARCODE_TIMEOUT_MS 2000            // Max time to decode barcode
#define PRE_TURN_TIME_LIMIT 250            // Pre-turn confirmation time  
#define CORRECTION_HEADING_LIMIT 10.0f     // Corrections should be < 10° heading change
#define TURN_HEADING_RATE_THRESHOLD 25.0f  // Turn detection: > 25°/second rotation
#define LINE_LOW_THRESHOLD 1500             // IR value indicating line is dropping off

// Pattern tracking variables
static float last_heading = 0.0f;
static uint32_t last_heading_time = 0;

// ========== FUNCTION DECLARATIONS ==========
static void fsm_transition_to(fsm_state_t new_state);
static uint32_t get_state_duration_ms(void);
static void state_update_sensors(void);
static void state_process_state(void);
static float char_to_command(char command_char);
static void barcode_decode_character(char c);
static void barcode_reset_decoder(void);
static void direct_motor_control(float left_speed, float right_speed);
static bool read_magnetometer_safe(int16_t *mag_x, int16_t *mag_y);

// ========== FSM IMPLEMENTATION ==========

static void fsm_transition_to(fsm_state_t new_state) {
    const char* state_names[] = {"INIT", "LINE_FOLLOWING", "CORRECTION_LEFT", "CORRECTION_RIGHT", "PRE_TURN", "TURNING_LEFT", "TURNING_RIGHT", "BARCODE_DETECTED", "BARCODE_DECODING", "BARCODE_EXECUTING", "STOPPED"};
    printf("[FSM] %s → %s\n", state_names[current_state], state_names[new_state]);
    current_state = new_state;
    state_entry_time = to_ms_since_boot(get_absolute_time());
}

static uint32_t get_state_duration_ms(void) {
    return to_ms_since_boot(get_absolute_time()) - state_entry_time;
}

// ========== BARCODE DECODING FUNCTIONS ==========

// Convert barcode character to target heading change (degrees)
static float char_to_command(char command_char) {
    switch (command_char) {
        case 'L': return -90.0f;  // Left turn: -90 degrees
        case 'R': return 90.0f;   // Right turn: +90 degrees  
        case 'U': return 180.0f;  // U-turn: 180 degrees
        case 'S': return 0.0f;    // Straight: no heading change
        case 'B': return -180.0f; // Back/reverse: -180 degrees
        default:
            printf("[BARCODE] Unknown command '%c' - defaulting to straight\n", command_char);
            return 0.0f;
    }
}

// Reset barcode decoder state
static void barcode_reset_decoder(void) {
    sensors.barcode_index = 0;
    sensors.barcode_start_found = false;
    sensors.barcode_complete = false;
    sensors.barcode_command = '\0';
    memset(sensors.barcode_buffer, 0, sizeof(sensors.barcode_buffer));
}

// Process incoming barcode character data
static void barcode_decode_character(char c) {
    // Look for start delimiter '*'
    if (!sensors.barcode_start_found) {
        if (c == '*') {
            sensors.barcode_start_found = true;
            sensors.barcode_index = 0;
            printf("[BARCODE] Start delimiter found\n");
        }
        return;
    }
    
    // Process character after start delimiter
    if (c == '*') {
        // End delimiter found - complete barcode
        if (sensors.barcode_index == 1) {
            // Valid format: *X* (single character command)
            sensors.barcode_command = sensors.barcode_buffer[0];
            sensors.barcode_complete = true;
            printf("[BARCODE] Complete barcode decoded: *%c*\n", sensors.barcode_command);
        } else {
            printf("[BARCODE] Invalid barcode length (%d) - resetting\n", sensors.barcode_index);
            barcode_reset_decoder();
        }
    } else if (sensors.barcode_index < sizeof(sensors.barcode_buffer) - 1) {
        // Store command character
        sensors.barcode_buffer[sensors.barcode_index++] = c;
    } else {
        // Buffer overflow - reset
        printf("[BARCODE] Buffer overflow - resetting decoder\n");
        barcode_reset_decoder();
    }
}

static void state_update_sensors(void) {
    sensors.timestamp_ms = to_ms_since_boot(get_absolute_time());
    
    // Read IR line sensor DIRECTLY to avoid boolean threshold issues
    adc_select_input(2); // GP28 is ADC2
    sleep_us(100);       // ADC settling time
    uint16_t adc_value = adc_read();
    
    // SMOOTH line detection and error calculation
    // ADC: 0-4095, LINE_THRESHOLD=2800, so we expect:
    // - White background: ~1000-2000 ADC 
    // - Black line: ~2800-4000 ADC
    // - Center line: ~3200 ADC
    
    // Store raw ADC value for pattern analysis
    sensors.raw_adc_value = adc_value;
    
    // ULTRA-FAST TURN SIGNATURE DETECTION - Analyze raw ADC pattern before error calculation
    static uint16_t adc_history[4] = {3200, 3200, 3200, 3200}; // Recent readings (initialized to center)
    static int history_index = 0;
    
    // Shift history and add new reading
    adc_history[history_index] = adc_value;
    history_index = (history_index + 1) % 4;
    
    // INSTANT TURN DETECTION: Look for rapid directional ADC trend indicating turn
    uint16_t adc_oldest = adc_history[history_index]; // Oldest reading (4 cycles ago)
    uint16_t adc_newest = adc_value; // Current reading
    int adc_trend = (int)adc_newest - (int)adc_oldest; // Positive = trending right, Negative = trending left
    
    // CONSERVATIVE: Only detect very clear turn signatures to avoid false positives
    bool rapid_turn_signature = (abs(adc_trend) > 800); // Much higher threshold for stability
    sensors.rapid_turn_detected = rapid_turn_signature;
    
    if (sensors.rapid_turn_detected) {
        printf("[TURN_SIG] Conservative turn detected: %d→%d (Δ%d) - TURN INCOMING!\n", 
               adc_oldest, adc_newest, adc_trend);
    }
    
    // Convert to error (-1.0 = far left, 0.0 = centered, +1.0 = far right)
    const float LINE_CENTER = 3200.0f;  // Expected center line reading
    const float LINE_RANGE = 1200.0f;   // Range for normalization (±600 from center)
    
    sensors.line_error = (adc_value - LINE_CENTER) / LINE_RANGE;
    sensors.line_error = fmaxf(-1.0f, fminf(1.0f, sensors.line_error)); // Clamp to ±1.0
    
    // STABLE line detection with hysteresis to prevent flickering
    static bool was_line_detected = false;
    const uint16_t DETECT_THRESHOLD = 1900;   // Very sensitive threshold to detect line (was 2300)
    const uint16_t LOSE_THRESHOLD = 1600;     // Very sensitive threshold to lose line (was 2000)
    
    if (!was_line_detected && adc_value >= DETECT_THRESHOLD) {
        sensors.line_detected = true;   // Line found
        was_line_detected = true;
    } else if (was_line_detected && adc_value < LOSE_THRESHOLD) {
        sensors.line_detected = false;  // Line lost  
        was_line_detected = false;
    } else {
        sensors.line_detected = was_line_detected; // Keep previous state
    }
    
    // Read IMU data using SAFE direct timeout-protected I2C (no driver functions!)
    int16_t mag_x_raw, mag_y_raw;
    if (read_magnetometer_safe(&mag_x_raw, &mag_y_raw)) {
        float mag_x = (float)mag_x_raw;
        float mag_y = (float)mag_y_raw; 
        // Simple heading calculation: atan2 in degrees
        sensors.imu_heading = atan2f(mag_y, mag_x) * 180.0f / M_PI;
        if (sensors.imu_heading < 0) sensors.imu_heading += 360.0f; // Normalize to 0-360°
    } else {
        // Keep previous heading if read fails occasionally
        static float last_good_heading = 0.0f;
        if (sensors.imu_heading != 0.0f) {
            last_good_heading = sensors.imu_heading;
        }
        sensors.imu_heading = last_good_heading;
    }
    // BARCODE DETECTION: Look for barcode patterns in IR data
    // Barcode appears as dark bands (high ADC values) interrupting the line
    static uint16_t prev_adc = 3200;  // Previous ADC reading
    static bool barcode_pattern_active = false;
    
    // Detect barcode start: sudden drop in ADC (dark barcode band)
    int adc_change = (int)adc_value - (int)prev_adc;
    bool barcode_edge_detected = (adc_change < -BARCODE_THRESHOLD) || (adc_change > BARCODE_THRESHOLD);
    
    if (barcode_edge_detected && !barcode_pattern_active) {
        sensors.barcode_detected = true;
        barcode_pattern_active = true;
        printf("[BARCODE] Pattern edge detected: %d → %d (Δ%d)\n", prev_adc, adc_value, adc_change);
    } else if (!barcode_edge_detected && barcode_pattern_active) {
        // End of barcode pattern
        barcode_pattern_active = false;
        sensors.barcode_detected = false;
    } else {
        sensors.barcode_detected = barcode_pattern_active;
    }
    
    prev_adc = adc_value;
    
    // Calculate heading rate for pattern detection (degrees per second)
    if (last_heading_time > 0) {
        uint32_t time_diff = sensors.timestamp_ms - last_heading_time;
        if (time_diff > 0) {
            float heading_diff = sensors.imu_heading - last_heading;
            // Normalize heading difference to [-180, 180]
            while (heading_diff > 180.0f) heading_diff -= 360.0f;
            while (heading_diff < -180.0f) heading_diff += 360.0f;
            sensors.heading_rate = heading_diff * 1000.0f / time_diff; // Convert to degrees/second
        }
    }
    last_heading = sensors.imu_heading;
    last_heading_time = sensors.timestamp_ms;
    
    // Debug sensor readings every 500ms  
    static uint32_t last_sensor_debug = 0;
    if ((sensors.timestamp_ms - last_sensor_debug) > 500) {
        printf("[SENSORS] ADC: %d, Line: %s, Error: %.3f, Heading: %.1f°, Rate: %.1f°/s\n", 
               adc_value, sensors.line_detected ? "FOUND" : "LOST", sensors.line_error,
               sensors.imu_heading, sensors.heading_rate);
        last_sensor_debug = sensors.timestamp_ms;
    }
}

static void direct_motor_control(float left_speed, float right_speed) {
    // Clamp speeds to safe range
    left_speed = fmaxf(0.0f, fminf(1.0f, left_speed));
    right_speed = fmaxf(0.0f, fminf(1.0f, right_speed));
    
    // Convert to PWM values
    uint16_t left_pwm = (uint16_t)(left_speed * 65535);
    uint16_t right_pwm = (uint16_t)(right_speed * 65535);

    // DIRECT PWM control - same as working calibration tool
    pwm_set_gpio_level(8, 0);          // M1A = 0 (right motor)
    pwm_set_gpio_level(9, right_pwm);   // M1B = PWM (right motor)
    pwm_set_gpio_level(10, left_pwm);   // M2A = PWM (left motor)  
    pwm_set_gpio_level(11, 0);          // M2B = 0 (left motor)
}

// Safe magnetometer reading with timeout protection (no blocking!)
static bool read_magnetometer_safe(int16_t *mag_x, int16_t *mag_y) {
    // LSM303DLHC magnetometer registers and address
    const uint8_t MAG_ADDR = 0x1E;
    const uint8_t OUT_X_H_M = 0x03;
    
    // Try to read 6 bytes starting from X_H register (X_H, X_L, Z_H, Z_L, Y_H, Y_L)
    uint8_t reg = OUT_X_H_M;
    uint8_t data[6];
    
    // Write register address with timeout
    int result = i2c_write_timeout_us(i2c0, MAG_ADDR, &reg, 1, true, 10000); // 10ms timeout
    if (result != 1) return false;
    
    // Read magnetometer data with timeout  
    result = i2c_read_timeout_us(i2c0, MAG_ADDR, data, 6, false, 10000); // 10ms timeout
    if (result != 6) return false;
    
    // Convert to signed 16-bit values (MSB first for LSM303DLHC)
    *mag_x = (int16_t)((data[0] << 8) | data[1]); // X_H, X_L
    *mag_y = (int16_t)((data[4] << 8) | data[5]); // Y_H, Y_L (data[2],data[3] is Z)
    
    return true;
}

// ========== STATE HANDLERS ==========

static void state_handle_init(void) {
    // Only run initialization ONCE when first entering this state
    static bool init_done = false;
    
    if (!init_done) {
        printf("[INIT] Starting initialization with IMU error handling...\n");
        
        // SAFE IMU initialization - using direct timeout-protected I2C (avoiding driver functions)
        printf("[INIT] Initializing I2C for IMU (using direct timeout-protected method)...\n");
        i2c_init(i2c0, 100000); // Use same 100kHz speed as diagnostic
        gpio_set_function(16, GPIO_FUNC_I2C); // SDA
        gpio_set_function(17, GPIO_FUNC_I2C); // SCL  
        gpio_pull_up(16);
        gpio_pull_up(17);
        sleep_ms(100); // Allow I2C bus to stabilize
        
        // Skip all IMU driver functions - they cause blocking issues
        printf("[INIT] ✓ I2C bus initialized (bypassing driver functions)\n");
        printf("[INIT] ✓ IMU will use direct magnetometer reading with timeouts\n");
        
        // NOTE: We'll read magnetometer directly using our safe timeout-protected function
        
        printf("[INIT] Setting up PWM for motors...\n");
        setup_pwm_pair(8, 9);   // Right motor (M1A, M1B)
        setup_pwm_pair(10, 11); // Left motor (M2A, M2B)
        printf("[INIT] ✓ PWM initialized!\n");
        
        printf("[INIT] Starting line following with intelligent turn detection in 3 seconds...\n");
        init_done = true; // Mark initialization as complete
    }
    
    // Wait 3 seconds then start line following
    if (get_state_duration_ms() > 3000) {
        fsm_transition_to(STATE_LINE_FOLLOWING);
    }
}

static void state_handle_line_following(void) {
    // BARCODE DETECTION: Highest priority - check for barcode patterns first
    if (sensors.barcode_detected) {
        printf("[LINE_FOLLOWING] 📊 BARCODE DETECTED - Transitioning to decode mode\n");
        barcode_reset_decoder();  // Reset decoder for new barcode
        fsm_transition_to(STATE_BARCODE_DETECTED);
        return;
    }
    
    // STABLE LINE FOLLOWING: Use traditional error-based detection only
    // (Rapid turn detection disabled to prevent false positives during normal following)
    
    // PATTERN-BASED DETECTION: differentiate corrections vs turns
    
    // Check for small corrections (quick, shallow adjustments)
    if (fabs(sensors.line_error) > CORRECTION_ERROR_THRESHOLD && fabs(sensors.line_error) <= TURN_ERROR_THRESHOLD) {
        if (sensors.line_error > 0) {
            printf("[LINE_FOLLOWING] Line detected RIGHT (%.3f) - need CORRECTION_LEFT\n", sensors.line_error);
            fsm_transition_to(STATE_CORRECTION_LEFT);
        } else {
            printf("[LINE_FOLLOWING] Line detected LEFT (%.3f) - need CORRECTION_RIGHT\n", sensors.line_error);
            fsm_transition_to(STATE_CORRECTION_RIGHT);
        }
        return;
    }
    
    // Check for potential turn (large error + pattern analysis needed)
    if (fabs(sensors.line_error) > TURN_ERROR_THRESHOLD) {
        printf("[LINE_FOLLOWING] Large error detected (%.3f) - entering PRE_TURN for analysis\n", sensors.line_error);
        // Initialize pattern tracking
        sensors.error_start_time = sensors.timestamp_ms;
        sensors.initial_heading = sensors.imu_heading;
        fsm_transition_to(STATE_PRE_TURN);
        return;
    }
    
    // Continue following even if line not perfectly detected
    // This allows graceful handling of brief sensor gaps
    
    // NORMAL line following with DIRECT PWM control
    float base_speed = 0.57f;         // Both motors balanced at this speed
    float min_speed = 0.5f;           // Hardware minimum threshold
    float max_correction = 0.02f;     // Small corrections for normal following
    
    // Gentle PI controller for normal operation
    float kp = 0.1f;                  // Small proportional gain
    static float integral_error = 0.0f;
    float ki = 0.03f;                 // Small integral term
    
    // Update integral with windup protection
    integral_error += sensors.line_error * 0.02f; // 20ms sample time
    integral_error = fmaxf(-0.1f, fminf(0.1f, integral_error)); // Limit windup
    
    float raw_correction = kp * sensors.line_error + ki * integral_error;
    float correction = fmaxf(-max_correction, fminf(max_correction, raw_correction));
    
    // Calculate balanced motor speeds
    float left_speed = base_speed - correction;
    float right_speed = base_speed + correction;
    
    // Ensure motors stay above minimum threshold
    left_speed = fmaxf(min_speed, fminf(1.0f, left_speed));
    right_speed = fmaxf(min_speed, fminf(1.0f, right_speed));
    
    // Apply motor control
    direct_motor_control(left_speed, right_speed);
    
    // Debug output every 300ms
    static uint32_t last_debug = 0;
    if (sensors.timestamp_ms - last_debug > 300) {
        printf("[LINE_FOLLOWING] Error: %.3f, I: %.3f, Corr: %.3f, Motors: L=%.3f R=%.3f\n", 
               sensors.line_error, integral_error, correction, left_speed, right_speed);
        last_debug = sensors.timestamp_ms;
    }
}

static void state_handle_pre_turn(void) {
    // PATTERN ANALYSIS: Determine if this is a real turn or just a correction
    
    uint32_t time_in_error = sensors.timestamp_ms - sensors.error_start_time;
    float heading_change = sensors.imu_heading - sensors.initial_heading;
    
    // Normalize heading change
    while (heading_change > 180.0f) heading_change -= 360.0f;
    while (heading_change < -180.0f) heading_change += 360.0f;
    
    // TURN SIGNATURE DETECTION (all conditions from your friend's advice)
    bool large_sustained_error = (fabs(sensors.line_error) > TURN_ERROR_THRESHOLD && time_in_error > PRE_TURN_TIME_LIMIT);
    bool line_dropping_off = (sensors.raw_adc_value < LINE_LOW_THRESHOLD);
    bool significant_rotation = (fabs(sensors.heading_rate) > TURN_HEADING_RATE_THRESHOLD || fabs(heading_change) > 20.0f);
    
    // Decision logic based on pattern analysis
    if (large_sustained_error && (line_dropping_off || significant_rotation)) {
        // CONFIRMED TURN - all signatures match
        if (sensors.line_error > 0) {
            printf("[PRE_TURN] CONFIRMED RIGHT TURN: Error=%.3f, Time=%lums, Heading=%.1f°/s, ADC=%d\n", 
                   sensors.line_error, time_in_error, sensors.heading_rate, sensors.raw_adc_value);
            fsm_transition_to(STATE_TURNING_RIGHT);
        } else {
            printf("[PRE_TURN] CONFIRMED LEFT TURN: Error=%.3f, Time=%lums, Heading=%.1f°/s, ADC=%d\n", 
                   sensors.line_error, time_in_error, sensors.heading_rate, sensors.raw_adc_value);
            fsm_transition_to(STATE_TURNING_LEFT);
        }
        return;
    }
    
    // If error reduces or pattern doesn't match turn signature -> it was just a correction
    if (fabs(sensors.line_error) < CORRECTION_ERROR_THRESHOLD) {
        printf("[PRE_TURN] False alarm - error reduced to %.3f, returning to LINE_FOLLOWING\n", sensors.line_error);
        fsm_transition_to(STATE_LINE_FOLLOWING);
        return;
    }
    
    // Timeout safety - if we're here too long without confirmation, treat as correction
    if (time_in_error > 400) {
        printf("[PRE_TURN] Timeout without confirmation - treating as correction\n");
        if (sensors.line_error > 0) {
            fsm_transition_to(STATE_CORRECTION_RIGHT);
        } else {
            fsm_transition_to(STATE_CORRECTION_LEFT);
        }
        return;
    }
    
    // Continue analysis - maintain gentle following while deciding
    float base_speed = 0.55f;  // Slightly slower while analyzing
    float correction = 0.1f * sensors.line_error;
    float left_speed = base_speed - correction;
    float right_speed = base_speed + correction;
    
    // Keep in safe PWM range
    left_speed = fmaxf(0.5f, fminf(0.65f, left_speed));
    right_speed = fmaxf(0.5f, fminf(0.65f, right_speed));
    
    direct_motor_control(left_speed, right_speed);
    
    // Debug output
    static uint32_t last_debug = 0;
    if (sensors.timestamp_ms - last_debug > 100) {
        printf("[PRE_TURN] Analyzing: Error=%.3f, Time=%lums, HeadingRate=%.1f°/s, ADC=%d\n", 
               sensors.line_error, time_in_error, sensors.heading_rate, sensors.raw_adc_value);
        last_debug = sensors.timestamp_ms;
    }
}

static void state_handle_correction_left(void) {
    // Robot drifted RIGHT, need to turn LEFT (slow down right motor)
    
    // EMERGENCY EXIT: If turn signature detected, immediately switch to turning!
    if (sensors.rapid_turn_detected) {
        printf("[CORRECTION_LEFT] ⚡ TURN SIGNATURE - Was correcting, now TURNING!\n");
        fsm_transition_to(STATE_TURNING_LEFT);
        return;
    }
    
    // Check if error is back to normal - return to line following
    float return_threshold = 0.1f; // Return when error < 10%
    if (fabs(sensors.line_error) < return_threshold) {
        printf("[CORRECTION_LEFT] Error reduced to %.3f - returning to LINE_FOLLOWING\n", sensors.line_error);
        fsm_transition_to(STATE_LINE_FOLLOWING);
        return;
    }
    
    // Pattern-based escalation: if error gets too large or correction takes too long
    if (fabs(sensors.line_error) > TURN_ERROR_THRESHOLD) {
        printf("[CORRECTION_LEFT] Error escalated to %.3f - entering PRE_TURN analysis\n", sensors.line_error);
        sensors.error_start_time = sensors.timestamp_ms;
        sensors.initial_heading = sensors.imu_heading;
        fsm_transition_to(STATE_PRE_TURN);
        return;
    }
    
    // Check if correction has been running too long - escalate to pre-turn analysis
    if (get_state_duration_ms() > CORRECTION_TIME_LIMIT) {
        printf("[CORRECTION_LEFT] Timeout after %lums - entering PRE_TURN analysis\n", get_state_duration_ms());
        sensors.error_start_time = sensors.timestamp_ms;
        sensors.initial_heading = sensors.imu_heading;
        fsm_transition_to(STATE_PRE_TURN);
        return;
    }
    
    // AGGRESSIVE LEFT correction - slow down RIGHT motor significantly
    float left_speed = 0.58f;         // Keep LEFT motor at good speed
    float right_speed = 0.51f;        // Slow RIGHT motor to minimum for sharp left turn
    
    direct_motor_control(left_speed, right_speed);
    
    // Debug output every 200ms
    static uint32_t last_debug = 0;
    if (sensors.timestamp_ms - last_debug > 200) {
        printf("[CORRECTION_LEFT] Error: %.3f, Motors: L=%.3f R=%.3f (turning LEFT)\n", 
               sensors.line_error, left_speed, right_speed);
        last_debug = sensors.timestamp_ms;
    }
}

static void state_handle_correction_right(void) {
    // Robot drifted LEFT, need to turn RIGHT (slow down left motor)
    
    // EMERGENCY EXIT: If turn signature detected, immediately switch to turning!
    if (sensors.rapid_turn_detected) {
        printf("[CORRECTION_RIGHT] ⚡ TURN SIGNATURE - Was correcting, now TURNING!\n");
        fsm_transition_to(STATE_TURNING_RIGHT);
        return;
    }
    
    // Check if error is back to normal - return to line following
    float return_threshold = 0.1f; // Return when error < 10%
    if (fabs(sensors.line_error) < return_threshold) {
        printf("[CORRECTION_RIGHT] Error reduced to %.3f - returning to LINE_FOLLOWING\n", sensors.line_error);
        fsm_transition_to(STATE_LINE_FOLLOWING);
        return;
    }
    
    // Pattern-based escalation: if error gets too large or correction takes too long
    if (fabs(sensors.line_error) > TURN_ERROR_THRESHOLD) {
        printf("[CORRECTION_RIGHT] Error escalated to %.3f - entering PRE_TURN analysis\n", sensors.line_error);
        sensors.error_start_time = sensors.timestamp_ms;
        sensors.initial_heading = sensors.imu_heading;
        fsm_transition_to(STATE_PRE_TURN);
        return;
    }
    
    // Check if correction has been running too long - escalate to pre-turn analysis  
    if (get_state_duration_ms() > CORRECTION_TIME_LIMIT) {
        printf("[CORRECTION_RIGHT] Timeout after %lums - entering PRE_TURN analysis\n", get_state_duration_ms());
        sensors.error_start_time = sensors.timestamp_ms;
        sensors.initial_heading = sensors.imu_heading;
        fsm_transition_to(STATE_PRE_TURN);
        return;
    }
    
    // AGGRESSIVE RIGHT correction - slow down LEFT motor significantly
    float left_speed = 0.51f;         // Slow LEFT motor to minimum for sharp right turn  
    float right_speed = 0.58f;        // Keep RIGHT motor at good speed
    
    direct_motor_control(left_speed, right_speed);
    
    // Debug output every 200ms (more frequent during correction)
    static uint32_t last_debug = 0;
    if (sensors.timestamp_ms - last_debug > 200) {
        printf("[CORRECTION_RIGHT] Error: %.3f, Motors: L=%.3f R=%.3f\n", 
               sensors.line_error, left_speed, right_speed);
        last_debug = sensors.timestamp_ms;
    }
}

static void state_handle_turning_left(void) {
    // Handle sharp LEFT turns using IMU heading feedback and line distance
    static float initial_heading = 0.0f;
    static uint32_t turn_start_time = 0;
    
    // Initialize turn parameters on entry
    if (get_state_duration_ms() < 50) { // First 50ms in turning state
        initial_heading = sensors.imu_heading;
        turn_start_time = sensors.timestamp_ms;
        printf("[TURNING_LEFT] Starting LEFT turn from heading %.1f°\n", initial_heading);
    }
    
    // Calculate how much we've turned so far
    float heading_change = sensors.imu_heading - initial_heading;
    // Normalize heading change to [-180, 180] range
    while (heading_change > 180.0f) heading_change -= 360.0f;
    while (heading_change < -180.0f) heading_change += 360.0f;
    
    // FLEXIBLE TURN COMPLETION: Multiple exit strategies
    float target_turn_angle = -90.0f; // Left turn is negative
    float turn_tolerance = 25.0f; // More tolerant ±25° tolerance
    bool angle_adequate = (heading_change <= (target_turn_angle + turn_tolerance)) && 
                         (heading_change >= (target_turn_angle - turn_tolerance));
    
    // Check if line is reacquired with reasonable error
    float detection_threshold = 0.4f; // Much more tolerant for turns
    bool line_reacquired = sensors.line_detected && (fabs(sensors.line_error) < detection_threshold);
    
    // Time-based completion: after 800ms, just check if we have reasonable line detection
    uint32_t turn_duration = sensors.timestamp_ms - turn_start_time;
    bool minimum_turn_time = turn_duration > 800;
    bool emergency_line_found = sensors.line_detected && (fabs(sensors.line_error) < 0.6f);
    
    // Multiple completion conditions (OR logic for more flexibility)
    if ((angle_adequate && line_reacquired) ||                    // Ideal: good angle + good line
        (minimum_turn_time && emergency_line_found) ||            // Time-based: minimum time + any line
        (turn_duration > 1500 && sensors.line_detected)) {       // Emergency: long time + any line detection
        
        printf("[TURNING_LEFT] Turn complete! Duration: %dms, Angle: %.1f° (target: %.1f°), Error: %.3f, Line: %s\n", 
               turn_duration, heading_change, target_turn_angle, sensors.line_error, 
               sensors.line_detected ? "FOUND" : "LOST");
        fsm_transition_to(STATE_LINE_FOLLOWING);
        return;
    }
    
    // Safety timeout after 3 seconds
    if ((sensors.timestamp_ms - turn_start_time) > 3000) {
        printf("[TURNING_LEFT] Turn timeout after %.1f° rotation - stopping\n", heading_change);
        fsm_transition_to(STATE_STOPPED);
        return;
    }
    
    // Execute LEFT turning maneuver - ULTRA aggressive differential steering
    float left_speed = 0.25f;   // Even slower left motor for sharper left turn
    float right_speed = 0.85f;  // Even faster right motor to pivot left more aggressively
    direct_motor_control(left_speed, right_speed);
    
    // Enhanced debug output showing completion status
    static uint32_t last_debug = 0;
    if (sensors.timestamp_ms - last_debug > 300) {
        uint32_t turn_duration = sensors.timestamp_ms - turn_start_time;
        printf("[TURNING_LEFT] %dms: Heading: %.1f° (Δ%.1f°/%.1f°), Error: %.3f, Line: %s, Angle: %s, Time: %s\n", 
               turn_duration, sensors.imu_heading, heading_change, target_turn_angle, sensors.line_error,
               sensors.line_detected ? "YES" : "NO",
               angle_adequate ? "OK" : "NO",
               minimum_turn_time ? "OK" : "NO");
        last_debug = sensors.timestamp_ms;
    }
}

static void state_handle_turning_right(void) {
    // Handle sharp RIGHT turns using IMU heading feedback and line distance
    static float initial_heading = 0.0f;
    static uint32_t turn_start_time = 0;
    
    // Initialize turn parameters on entry
    if (get_state_duration_ms() < 50) { // First 50ms in turning state
        initial_heading = sensors.imu_heading;
        turn_start_time = sensors.timestamp_ms;
        printf("[TURNING_RIGHT] Starting RIGHT turn from heading %.1f°\n", initial_heading);
    }
    
    // Calculate how much we've turned so far
    float heading_change = sensors.imu_heading - initial_heading;
    // Normalize heading change to [-180, 180] range
    while (heading_change > 180.0f) heading_change -= 360.0f;
    while (heading_change < -180.0f) heading_change += 360.0f;
    
    // FLEXIBLE TURN COMPLETION: Multiple exit strategies
    float target_turn_angle = 90.0f; // Right turn is positive
    float turn_tolerance = 25.0f; // More tolerant ±25° tolerance
    bool angle_adequate = (heading_change >= (target_turn_angle - turn_tolerance)) && 
                         (heading_change <= (target_turn_angle + turn_tolerance));
    
    // Check if line is reacquired with reasonable error
    float detection_threshold = 0.4f; // Much more tolerant for turns
    bool line_reacquired = sensors.line_detected && (fabs(sensors.line_error) < detection_threshold);
    
    // Time-based completion: after 800ms, just check if we have reasonable line detection
    uint32_t turn_duration = sensors.timestamp_ms - turn_start_time;
    bool minimum_turn_time = turn_duration > 800;
    bool emergency_line_found = sensors.line_detected && (fabs(sensors.line_error) < 0.6f);
    
    // Multiple completion conditions (OR logic for more flexibility)
    if ((angle_adequate && line_reacquired) ||                    // Ideal: good angle + good line
        (minimum_turn_time && emergency_line_found) ||            // Time-based: minimum time + any line
        (turn_duration > 1500 && sensors.line_detected)) {       // Emergency: long time + any line detection
        
        printf("[TURNING_RIGHT] Turn complete! Duration: %dms, Angle: %.1f° (target: %.1f°), Error: %.3f, Line: %s\n", 
               turn_duration, heading_change, target_turn_angle, sensors.line_error, 
               sensors.line_detected ? "FOUND" : "LOST");
        fsm_transition_to(STATE_LINE_FOLLOWING);
        return;
    }
    
    // Safety timeout after 3 seconds
    if ((sensors.timestamp_ms - turn_start_time) > 3000) {
        printf("[TURNING_RIGHT] Turn timeout after %.1f° rotation - stopping\n", heading_change);
        fsm_transition_to(STATE_STOPPED);
        return;
    }
    
    // Execute RIGHT turning maneuver - ULTRA aggressive differential steering
    float left_speed = 0.85f;   // Even faster left motor to pivot right more aggressively
    float right_speed = 0.25f;  // Even slower right motor for sharper right turn
    direct_motor_control(left_speed, right_speed);
    
    // Enhanced debug output showing completion status
    static uint32_t last_debug = 0;
    if (sensors.timestamp_ms - last_debug > 300) {
        uint32_t turn_duration = sensors.timestamp_ms - turn_start_time;
        printf("[TURNING_RIGHT] %dms: Heading: %.1f° (Δ%.1f°/%.1f°), Error: %.3f, Line: %s, Angle: %s, Time: %s\n", 
               turn_duration, sensors.imu_heading, heading_change, target_turn_angle, sensors.line_error,
               sensors.line_detected ? "YES" : "NO",
               angle_adequate ? "OK" : "NO",
               minimum_turn_time ? "OK" : "NO");
        last_debug = sensors.timestamp_ms;
    }
}

static void state_handle_stopped(void) {
    direct_motor_control(0.0f, 0.0f);
    printf("[STOPPED] Test complete.\n");
}

// ========== BARCODE STATE HANDLERS ==========

static void state_handle_barcode_detected(void) {
    // Step 1: Robot slows down to accommodate for decoding latency
    printf("[BARCODE_DETECTED] Slowing down for barcode decoding\n");
    
    // Slow forward movement at reduced speed
    direct_motor_control(BARCODE_DECODE_SPEED, BARCODE_DECODE_SPEED);
    
    // Transition to decoding state after brief slowdown
    if (get_state_duration_ms() > 100) {
        fsm_transition_to(STATE_BARCODE_DECODING);
    }
}

static void state_handle_barcode_decoding(void) {
    // Step 2 & 3: IR Barcode Driver decodes barcode and converts to command
    
    // Continue slow movement while decoding
    direct_motor_control(BARCODE_DECODE_SPEED, BARCODE_DECODE_SPEED);
    
    // Simulate barcode decoding - in real implementation, this would read from barcode sensor
    // For now, simulate with timing-based character injection
    static bool simulation_done = false;
    if (get_state_duration_ms() > 500 && !simulation_done) {
        // Simulate receiving barcode data: *L* for left turn
        barcode_decode_character('*');  // Start delimiter
        barcode_decode_character('L');  // Command character (Left turn)
        barcode_decode_character('*');  // End delimiter
        simulation_done = true;
        printf("[BARCODE_DECODING] Simulated barcode: *L* (Left turn)\n");
    }
    
    // Check if barcode decoding is complete
    if (sensors.barcode_complete) {
        printf("[BARCODE_DECODING] ✅ Barcode decoded: '%c'\n", sensors.barcode_command);
        
        // Convert character to heading change
        float heading_change = char_to_command(sensors.barcode_command);
        sensors.target_heading = sensors.imu_heading + heading_change;
        
        // Normalize target heading to [0, 360)
        while (sensors.target_heading >= 360.0f) sensors.target_heading -= 360.0f;
        while (sensors.target_heading < 0.0f) sensors.target_heading += 360.0f;
        
        printf("[BARCODE_DECODING] Target heading: %.1f° (current: %.1f°, change: %.1f°)\n", 
               sensors.target_heading, sensors.imu_heading, heading_change);
        
        fsm_transition_to(STATE_BARCODE_EXECUTING);
        return;
    }
    
    // Timeout protection
    if (get_state_duration_ms() > BARCODE_TIMEOUT_MS) {
        printf("[BARCODE_DECODING] ⚠️ Decoding timeout - returning to line following\n");
        barcode_reset_decoder();
        fsm_transition_to(STATE_LINE_FOLLOWING);
    }
}

static void state_handle_barcode_executing(void) {
    // Step 4 & 5: Calculate heading change and execute motor movements
    
    float current_heading = sensors.imu_heading;
    float heading_error = sensors.target_heading - current_heading;
    
    // Normalize heading error to [-180, 180]
    while (heading_error > 180.0f) heading_error -= 360.0f;
    while (heading_error < -180.0f) heading_error += 360.0f;
    
    // Execute turn based on heading error
    float turn_tolerance = 10.0f;  // ±10° tolerance
    
    if (fabs(heading_error) < turn_tolerance) {
        // Target heading reached
        printf("[BARCODE_EXECUTING] ✅ Target heading reached! Error: %.1f°\n", heading_error);
        barcode_reset_decoder();
        fsm_transition_to(STATE_LINE_FOLLOWING);
        return;
    }
    
    // Execute differential steering to reach target heading
    float base_speed = 0.5f;
    float turn_speed = 0.3f;
    
    if (heading_error > 0) {
        // Turn left (positive error = need to turn left)
        direct_motor_control(base_speed - turn_speed, base_speed + turn_speed);
        printf("[BARCODE_EXECUTING] Turning LEFT - Error: %.1f°\n", heading_error);
    } else {
        // Turn right (negative error = need to turn right)
        direct_motor_control(base_speed + turn_speed, base_speed - turn_speed);
        printf("[BARCODE_EXECUTING] Turning RIGHT - Error: %.1f°\n", heading_error);
    }
    
    // Timeout protection
    if (get_state_duration_ms() > 5000) {
        printf("[BARCODE_EXECUTING] ⚠️ Execution timeout - returning to line following\n");
        barcode_reset_decoder();
        fsm_transition_to(STATE_LINE_FOLLOWING);
    }
}

static void state_process_state(void) {
    switch (current_state) {
        case STATE_INIT:
            state_handle_init();
            break;
            
        case STATE_LINE_FOLLOWING:
            state_handle_line_following();
            break;
            
        case STATE_CORRECTION_LEFT:
            state_handle_correction_left();
            break;
            
        case STATE_CORRECTION_RIGHT:
            state_handle_correction_right();
            break;
            
        case STATE_PRE_TURN:
            state_handle_pre_turn();
            break;
            
        case STATE_TURNING_LEFT:
            state_handle_turning_left();
            break;
            
        case STATE_TURNING_RIGHT:
            state_handle_turning_right();
            break;
            
        case STATE_BARCODE_DETECTED:
            state_handle_barcode_detected();
            break;
            
        case STATE_BARCODE_DECODING:
            state_handle_barcode_decoding();
            break;
            
        case STATE_BARCODE_EXECUTING:
            state_handle_barcode_executing();
            break;
            
        case STATE_STOPPED:
            state_handle_stopped();
            break;
    }
    
    // Auto-stop after 45 seconds for safety
    if (get_state_duration_ms() > 45000) {
        if (current_state != STATE_STOPPED) {
            printf("[FSM] Auto-stopping after 45 seconds\n");
            fsm_transition_to(STATE_STOPPED);
        }
    }
}

// ========== MAIN PROGRAM ==========

int main() {
    stdio_init_all();
    
    printf("=== roboDriverV4 Line Following + Correction FSM Test ===\n");
    printf("States: INIT → LINE_FOLLOWING ⇄ CORRECTION → STOPPED\n");
    printf("- Normal following: gentle corrections (±0.02 PWM)\n");
    printf("- Correction mode: aggressive corrections (±0.05 PWM)\n"); 
    printf("- Direct PWM control (no smoothing interference)\n");
    printf("- PID execution frequency: %.1f Hz (%.1f ms period)\n", 
           50.0f, 1000.0f/50.0f);
    
    // Initialize hardware
    ir_track_init();  // Initialize IR sensor
    
    // Initialize PWM for motors
    setup_pwm_pair(8, 9);   // Right motor (M1A, M1B)
    setup_pwm_pair(10, 11); // Left motor (M2A, M2B)
    
    // Main FSM loop at 50Hz (20ms period)
    uint32_t last_update = 0;
    const uint32_t update_period_ms = 20;
    
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        if ((now - last_update) >= update_period_ms) {
            state_update_sensors();
            state_process_state();
            last_update = now;
        }
        
        sleep_ms(1); // Small sleep to prevent CPU spinning
    }
    
    return 0;
}