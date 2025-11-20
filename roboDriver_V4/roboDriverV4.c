#include "roboDriver_V4.h"  // Includes everything
#include "IMUDriver.h"  // Direct IMU access
#include "WithPID.h"  // Direct PID access
#include "IR_track.h" //IR Line Tracing funcs
#include "decode_barcode.h" //Barcode decoding
#include <string.h> // For strcmp, memset

// ========== MOTOR CALIBRATION CONSTANTS ==========
// These compensate for hardware differences between left and right motors
// Adjust these values to make robot go straight when both motors get same PWM
#define MOTOR_CALIBRATION_LEFT  1.0f    // Left motor multiplier (baseline)
#define MOTOR_CALIBRATION_RIGHT 0.97f   // Right motor multiplier (slightly reduced for balance)

// Recovery system removed for clean straight-line performance

// ========== GLOBAL VARIABLES FOR NEW SYSTEM LOGIC ==========
bool line_trace_flag = false;  // Core line detection flag (as specified: is_line_traced = TRUE)

// Function declarations from WithPID.c
float get_last_speed_L_mm_per_s(void);
float get_last_speed_R_mm_per_s(void);
void set_target_speed(float speed_mm_per_s);
void robot_movement(float left_motor_output, float right_motor_output);
void update_wheel_speeds(void);

// Additional function declarations for roboDriverV3
float calculate_heading_difference(float current, float target);
bool read_robot_state(robot_state_t *state);
void set_motor_speed(int left_speed, int right_speed);
void stop_robot(void);
bool execute_precise_turn(const char* direction, float current_heading);
bool execute_precise_turn_angle(float current_heading, float target_angle);

// ========== NEW TURNING FUNCTION (as requested) ==========
/**
 * NEW Turning Function as specified:
 * params: current_heading, target_heading  
 * var calculated in function: target_angle = target - current
 * differentiates between left and right based on shortest path
 * return values: motor output in PWM levels for Left and Right wheels
 * return values will be put into update_wheel_speeds() function to execute turns
 */
void calculate_turn_motor_outputs(float current_heading, float target_heading, 
                                  float* left_motor_pwm, float* right_motor_pwm) {
    // Calculate target_angle = target - current (as specified)
    float target_angle = target_heading - current_heading;
    
    // Normalize to -180 to +180 degrees for shortest path (differentiates left/right)
    while (target_angle > 180.0f) target_angle -= 360.0f;
    while (target_angle < -180.0f) target_angle += 360.0f;
    
    // 15-degree increment system from PDF (6 steps = 90 degrees)
    const float STEP_ANGLE = 15.0f;  // Each "pie slice" from your PDF
    const float PRECISION_TOLERANCE = 3.0f;  // Tighter tolerance for accuracy
    
    // Calculate which 15-degree "notch" we're aiming for
    int steps_needed = (int)round(fabs(target_angle) / STEP_ANGLE);
    float target_step_angle = steps_needed * STEP_ANGLE;
    if (target_angle < 0) target_step_angle = -target_step_angle;
    
    printf("[TURN_CALC] Current: %.1f°, Target: %.1f°, Angle: %.1f°\n", 
           current_heading, target_heading, target_angle);
    printf("[TURN_CALC] Step system: %.1f° → %d steps of 15° = %.1f°\n", 
           fabs(target_angle), steps_needed, target_step_angle);
    
    // Stepped turning power - gentler for precision
    float base_turn_power = 0.45f;  // Reduced for 15-degree precision
    
    if (fabs(target_angle) > PRECISION_TOLERANCE) {
        if (target_angle > 0) {
            // Turn RIGHT - left motor forward, right motor reverse
            *left_motor_pwm = base_turn_power;
            *right_motor_pwm = -base_turn_power * 0.8f;
            printf("[TURN_CALC] RIGHT turn (%.1f°): L=%.3f, R=%.3f\n", 
                   target_angle, *left_motor_pwm, *right_motor_pwm);
        } else {
            // Turn LEFT - right motor forward, left motor reverse
            *left_motor_pwm = -base_turn_power * 0.8f;
            *right_motor_pwm = base_turn_power;
            printf("[TURN_CALC] LEFT turn (%.1f°): L=%.3f, R=%.3f\n", 
                   target_angle, *left_motor_pwm, *right_motor_pwm);
        }
    } else {
        // Within tolerance - no turn needed
        *left_motor_pwm = 0.0f;
        *right_motor_pwm = 0.0f;
        printf("[TURN_CALC] Within %.1f° tolerance - turn complete\n", PRECISION_TOLERANCE);
    }
}

// Execute turn using 15-degree stepped approach from PDF
void execute_stepped_turn(float target_heading) {
    const float STEP_ANGLE = 15.0f;  // Each "pie slice" from your PDF
    const float STEP_DURATION_MS = 150;  // Time for each 15-degree step
    const float PRECISION_TOLERANCE = 3.0f;
    
    printf("[STEPPED_TURN] Starting stepped turn to %.1f°\n", target_heading);
    
    float current_heading = get_current_heading();
    float total_angle_diff = target_heading - current_heading;
    
    // Normalize angle difference
    while (total_angle_diff > 180.0f) total_angle_diff -= 360.0f;
    while (total_angle_diff < -180.0f) total_angle_diff += 360.0f;
    
    // Calculate number of 15-degree steps needed (6 steps = 90°)
    int total_steps = (int)round(fabs(total_angle_diff) / STEP_ANGLE);
    printf("[STEPPED_TURN] Need %d steps of 15° for %.1f° total turn\n", 
           total_steps, total_angle_diff);
    
    // Execute each 15-degree step
    for (int step = 1; step <= total_steps; step++) {
        float step_target = current_heading + (step * STEP_ANGLE * (total_angle_diff > 0 ? 1 : -1));
        
        printf("[STEPPED_TURN] Step %d/%d: targeting %.1f°\n", step, total_steps, step_target);
        
        // Apply turn for this step
        float left_pwm, right_pwm;
        calculate_turn_motor_outputs(get_current_heading(), step_target, &left_pwm, &right_pwm);
        robot_movement(left_pwm, right_pwm);
        
        // Wait for step duration
        sleep_ms((uint32_t)STEP_DURATION_MS);
        
        // Check if we've reached the target
        float current = get_current_heading();
        float remaining = fabs(target_heading - current);
        if (remaining <= PRECISION_TOLERANCE) {
            printf("[STEPPED_TURN] Target reached early at step %d\n", step);
            break;
        }
    }
    
    // Stop motors
    robot_movement(0.0f, 0.0f);
    
    printf("[STEPPED_TURN] Turn complete. Final heading: %.1f°\n", get_current_heading());
}

// ========== HIGH-LEVEL SYSTEM FUNCTIONS ==========

bool robot_full_system_init(void) {
    printf("Initializing robot system...\n");
    
    // Initialize IMU system
    printf("Attempting IMU initialization on Grove 4 (GP16=SDA, GP17=SCL)...\n");
    i2c_initialize();
    printf("I2C bus initialized, attempting LSM303DLHC detection...\n");
    if (!lsm303_init()) {
        printf("ERROR: IMU initialization failed! (Check Grove 4 connection)\n");
        return false;
    }
    printf("✓ IMU system initialized successfully\n");
    
    // Initialize motor and encoder system  
    if (!motor_system_init()) {
        printf("ERROR: Motor/encoder initialization failed!\n");
        return false;
    }
    printf("✓ Motor and encoder system initialized\n");
    
    // 1. IR line tracer init (as specified)
    ir_track_init();
    printf("✓ IR Line Tracer initialized on GP28\n");
    
    // 1. IR barcode decoder init (as specified)  
    init_pin_and_button();
    printf("✓ IR Barcode Decoder initialized (init_pin_and_button called)\n");
    
    printf("✓ Robot system fully initialized with IR sensors!\n");
    printf("\n=== AVAILABLE FUNCTIONS (as specified) ===\n");
    printf("1. init_pin_and_button() - ✓ Called\n");
    printf("2. get_decoded_barcode_char() - ✓ Available (wraps compareTwoArray)\n");
    printf("3. char_to_command() - ✓ Available for direction conversion\n");
    printf("4. is_barcode_ready_to_decode() - ✓ Available for detection\n");
    printf("NEW: calculate_turn_motor_outputs() - ✓ Added for motor control\n");
    printf("==========================================\n\n");
    return true;
}

bool is_robot_level(float max_tilt_degrees) {
    // This function should check if robot is level based on accelerometer
    // For now, return true as a placeholder
    return true;
}

bool update_robot_state(robot_state_t *state) {
    // Read IMU data
    state->imu_valid = imu_read_all_data(state->accel_g, &state->headings, &state->compass_quadrant);
    
    // Get tilt angles
    if (state->imu_valid) {
        imu_get_tilt_angles(&state->tilt_roll, &state->tilt_pitch);
    }
    
    // Update wheel speeds
    update_wheel_speeds();
    state->left_speed = get_left_wheel_speed();
    state->right_speed = get_right_wheel_speed();
    state->encoders_valid = true; // Assume encoders always work
    
    // Calculate distance traveled (simple integration)
    float current_distance = (get_left_wheel_distance() + get_right_wheel_distance()) / 2.0f;
    state->distance_traveled = current_distance;
    
    // Update current heading and calculate heading error
    if (state->imu_valid) {
        state->heading_current = state->headings.xy_heading;
        
        // Calculate heading error from some target (you'll set this elsewhere)
        static float last_heading = -999; // Invalid initial value
        if (last_heading != -999) {
            state->heading_error = calculate_turn_angle(last_heading, state->heading_current);
        } else {
            state->heading_error = 0;
        }
        last_heading = state->heading_current;
    }
    
    return state->imu_valid && state->encoders_valid;
}

float get_current_heading(void) {
    // Get magnetometer data for heading calculation
    int16_t mag_data[3];
    if (read_magnetometer(mag_data)) {
        float mag_x = (float)mag_data[0];
        float mag_y = (float)mag_data[1];
        return calculate_heading(mag_x, mag_y);
    }
    return 0.0f; // Return 0 if IMU read fails
}

float calculate_turn_angle(float current_heading, float target_heading) {
    float angle_diff = target_heading - current_heading;
    
    // Normalize to -180 to +180 range
    while (angle_diff > 180.0f) angle_diff -= 360.0f;
    while (angle_diff < -180.0f) angle_diff += 360.0f;
    
    return angle_diff;
}

void emergency_stop(void) {
    stop_robot();
    printf("EMERGENCY STOP ACTIVATED!\n");
}

void print_robot_status(const robot_state_t *state) {
    printf("=== ROBOT STATUS ===\n");
    printf("IMU: %s | Encoders: %s\n", 
           state->imu_valid ? "OK" : "FAIL",
           state->encoders_valid ? "OK" : "FAIL");
           
    if (state->imu_valid) {
        printf("Heading: %.1f° (Q%d) | Tilt: R=%.1f° P=%.1f°\n",
               state->headings.xy_heading, state->compass_quadrant,
               state->tilt_roll, state->tilt_pitch);
        printf("Accel: X=%.2fg Y=%.2fg Z=%.2fg\n",
               state->accel_g[0], state->accel_g[1], state->accel_g[2]);
    }
    
    if (state->encoders_valid) {
        float avg_speed = (state->left_speed + state->right_speed) / 2.0f;
        printf("Speed: L=%.0fmm/s R=%.0fmm/s | Avg=%.0fmm/s\n",
               state->left_speed, state->right_speed, avg_speed);
        printf("Distance: %.0fmm | Heading Error: %.1f°\n",
               state->distance_traveled, state->heading_error);
    }
    printf("====================\n\n");
}

// ========== TURNING CONTROL FUNCTIONS ==========

// Calculate motor outputs for turning from current heading to target heading
// Returns motor speeds for left and right wheels (-255 to 255)
typedef struct {
    int left_motor;   // Left motor speed (-255 to 255)
    int right_motor;  // Right motor speed (-255 to 255)
} turn_output_t;

turn_output_t calculate_turn_motors(float current_heading, float target_heading) {
    turn_output_t output = {0, 0};
    
    // Calculate shortest angle difference
    float angle_diff = calculate_heading_difference(current_heading, target_heading);
    
    // Determine turn direction and intensity
    float turn_power = 120;  // Base turning power
    
    if (fabsf(angle_diff) < 5.0f) {
        // Close to target - gentle correction
        turn_power = 60;
    } else if (fabsf(angle_diff) > 45.0f) {
        // Large turn needed - more power
        turn_power = 180;
    }
    
    if (angle_diff > 0) {
        // Turn right (clockwise)
        output.left_motor = (int)turn_power;   // Left wheel forward
        output.right_motor = -(int)turn_power; // Right wheel backward
    } else {
        // Turn left (counter-clockwise)
        output.left_motor = -(int)turn_power;  // Left wheel backward
        output.right_motor = (int)turn_power;  // Right wheel forward
    }
    
    printf("Turn: %.1f° → %.1f° (diff=%.1f°) | Motors: L=%d R=%d\n",
           current_heading, target_heading, angle_diff, 
           output.left_motor, output.right_motor);
    
    return output;
}

// Execute precise turn using direction commands (LEFT/RIGHT for 90° turns, or custom directions)
bool execute_precise_turn(const char* direction, float current_heading) {
    if (!direction) return false;
    
    float target_heading = current_heading;
    
    if (strcmp(direction, "LEFT") == 0) {
        target_heading = current_heading - 90.0f;  // Standard 90° left turn
        if (target_heading < 0) target_heading += 360.0f;
        printf("Executing LEFT turn (90°): %.1f° → %.1f°\n", current_heading, target_heading);
    } else if (strcmp(direction, "RIGHT") == 0) {
        target_heading = current_heading + 90.0f;  // Standard 90° right turn
        if (target_heading >= 360.0f) target_heading -= 360.0f;
        printf("Executing RIGHT turn (90°): %.1f° → %.1f°\n", current_heading, target_heading);
    } else if (strcmp(direction, "AROUND") == 0 || strcmp(direction, "REVERSE") == 0) {
        target_heading = current_heading + 180.0f;  // 180° turn around
        if (target_heading >= 360.0f) target_heading -= 360.0f;
        printf("Executing turn around (180°): %.1f° → %.1f°\n", current_heading, target_heading);
    } else {
        printf("Unknown turn direction: %s (supported: LEFT, RIGHT, AROUND)\n", direction);
        return false;
    }
    
    // Execute the turn with multiple correction steps
    uint32_t turn_start = to_ms_since_boot(get_absolute_time());
    const uint32_t turn_timeout = 3000;  // 3 second timeout
    
    while ((to_ms_since_boot(get_absolute_time()) - turn_start) < turn_timeout) {
        // Update IMU readings
        robot_state_t state;
        if (read_robot_state(&state) && state.imu_valid) {
            float heading_error = calculate_heading_difference(state.headings.xy_heading, target_heading);
            
            // Check if turn is complete
            if (fabsf(heading_error) < 5.0f) {
                stop_robot();
                printf("Turn complete! Final heading: %.1f° (error: %.1f°)\n", 
                       state.headings.xy_heading, heading_error);
                return true;
            }
            
            // Apply turn correction
            turn_output_t turn = calculate_turn_motors(state.headings.xy_heading, target_heading);
            set_motor_speed(turn.left_motor, turn.right_motor);
        }
        
        sleep_ms(20);  // Update at 50Hz for faster heading response
    }
    
    // Timeout - stop and report
    stop_robot();
    printf("Turn timeout! Check IMU or mechanical issues.\n");
    return false;
}

// ========== HELPER FUNCTION IMPLEMENTATIONS ==========

// Calculate the shortest angular difference between two headings
float calculate_heading_difference(float current, float target) {
    float diff = target - current;
    
    // Normalize to [-180, 180] range
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    
    return diff;
}

// Read current robot state (simplified implementation)
bool read_robot_state(robot_state_t *state) {
    if (!state) return false;
    
    // Initialize state
    memset(state, 0, sizeof(robot_state_t));
    
    // Try to read IMU data using existing functions
    int16_t raw_accel[3], raw_mag[3];
    bool accel_ok = read_accelerometer(raw_accel);
    bool mag_ok = read_magnetometer(raw_mag);
    
    if (accel_ok && mag_ok) {
        // Convert accelerometer data
        float accel_float[3];
        for (int i = 0; i < 3; i++) {
            accel_float[i] = (float)raw_accel[i];
            state->accel_g[i] = accel_float[i] / 16384.0f;  // Simple conversion to g
        }
        
        // Process magnetometer data
        float mag_float[3], mag_lpf[3], mag_kalman[3];
        for (int i = 0; i < 3; i++) {
            mag_float[i] = (float)raw_mag[i];
        }
        
        apply_mag_low_pass_filter(mag_float, mag_lpf);
        apply_mag_kalman_filter(mag_lpf, mag_kalman);
        state->headings = calculate_headings_3d(mag_kalman[0], mag_kalman[1], mag_kalman[2]);
        
        // Simple heading error (no baseline for now)
        state->heading_error = 0.0f;
        state->imu_valid = true;
    } else {
        state->imu_valid = false;
    }
    
    // Read encoder data
    state->left_speed = get_last_speed_L_mm_per_s();
    state->right_speed = get_last_speed_R_mm_per_s();
    state->encoders_valid = true;  // Assume encoders always work
    
    return true;
}

// Set motor speeds directly
void set_motor_speed(int left_speed, int right_speed) {
    // Convert int speeds to float and use existing motor control
    float left_f = (float)left_speed;
    float right_f = (float)right_speed;
    
    // Use existing robot_movement function
    robot_movement(left_f, right_f);
}

// Stop the robot
void stop_robot(void) {
    robot_movement(0.0f, 0.0f);
}

// ========== ROBOT CONTROL ALGORITHMS ==========

// Global variables for system state
static float base_g_data[3];           // Initial accelerometer baseline
static heading_3d_t base_headings;     // Initial heading baseline
static bool system_initialized = false;
static uint32_t last_imu_update_ms;
static uint32_t last_pid_update_ms;
static uint32_t last_speed_update_ms;

// Motor smoothing state variables
static float smooth_motor_L = 0.0f;   // Smoothed left motor output (will initialize to target)
static float smooth_motor_R = 0.0f;   // Smoothed right motor output (will initialize to target)
static bool motor_smoothing_initialized = false;



// Control parameters
#define MAX_TILT_ANGLE 15.0f          // Maximum tilt angle to consider "level" (degrees)
// Heading correction parameters  
#define HEADING_CORRECTION_GAIN 0.0008f  // Much smaller gain to prevent hitting maximum
#define HEADING_DEADBAND_DEGREES 10.0f   // Even larger deadband for stability
#define MAX_HEADING_CORRECTION 0.03f     // Reduced max correction to 3%
#define MIN_MOTOR_OUTPUT 0.0f             // Allow zero output (stiction handling done separately)
#define IMU_UPDATE_INTERVAL_MS 200    // Slower IMU updates (200ms) for more stability
#define PID_UPDATE_INTERVAL_MS 100    // Run PID every 100ms
#define MOTOR_SMOOTHING_FACTOR 0.9f  // Much faster response for precision control

/**
 * @brief Check if robot is level and stable based on XZ tilt angle
 * @param current_g_data Current accelerometer data [x,y,z] in g-force
 * @return true if robot is level within acceptable limits
 */
bool is_robot_level_and_stable(float current_g_data[3]) {
    // Calculate tilt angle from accelerometer (XZ plane - side tilt)
    float tilt_xz = atan2f(current_g_data[0], current_g_data[2]) * 180.0f / 3.14159265f;
    float abs_tilt = fabs(tilt_xz);
    
    // Check if within acceptable tilt limits
    bool is_level = abs_tilt < MAX_TILT_ANGLE;
    
    if (!is_level) {
        printf("WARNING: Robot not level! Tilt: %.1f° (max: %.1f°)\n", abs_tilt, MAX_TILT_ANGLE);
    }
    
    return is_level;
}

/**
 * @brief Calculate heading error between current and base headings
 * @param current_headings Current compass readings
 * @param base_headings Baseline compass readings
 * @return Heading error in degrees (-180 to +180)
 */
float calculate_heading_error(heading_3d_t current_headings, heading_3d_t base_headings) {
    // Use XY heading (horizontal compass) for main heading control
    return calculate_turn_angle(base_headings.xy_heading, current_headings.xy_heading);
}

/**
 * @brief Calculate motor output corrections for heading control
 * @param error_headings Heading error in degrees
 * @param base_motor_output_L Base left motor output from PID
 * @param base_motor_output_R Base right motor output from PID  
 * @param corrected_output_L Output: corrected left motor output
 * @param corrected_output_R Output: corrected right motor output
 */
void calculate_angle_correction(float error_headings, 
                               float base_motor_output_L, float base_motor_output_R,
                               float *corrected_output_L, float *corrected_output_R) {
    
    // Apply deadband - ignore small errors to prevent oscillation
    if (fabsf(error_headings) < HEADING_DEADBAND_DEGREES) {
        *corrected_output_L = base_motor_output_L;
        *corrected_output_R = base_motor_output_R;
        return;
    }
    
    // Calculate correction with proportional control
    float correction = error_headings * HEADING_CORRECTION_GAIN;
    
    // Limit maximum correction to prevent instability
    correction = fmaxf(-MAX_HEADING_CORRECTION, fminf(MAX_HEADING_CORRECTION, correction));
    
    // Calculate target motor outputs
    float target_L = base_motor_output_L - correction;  // Subtract correction from left
    float target_R = base_motor_output_R + correction;  // Add correction to right
    
    // Clamp target outputs to valid range
    target_L = fmaxf(MIN_MOTOR_OUTPUT, fminf(1.0f, target_L));
    target_R = fmaxf(MIN_MOTOR_OUTPUT, fminf(1.0f, target_R));
    
    // Initialize smoothing on first run
    if (!motor_smoothing_initialized) {
        smooth_motor_L = target_L;
        smooth_motor_R = target_R;
        motor_smoothing_initialized = true;
        printf("Motor smoothing initialized: L=%.3f, R=%.3f\n", smooth_motor_L, smooth_motor_R);
    }
    
    // Apply exponential smoothing to reduce jitter
    smooth_motor_L += (target_L - smooth_motor_L) * MOTOR_SMOOTHING_FACTOR;
    smooth_motor_R += (target_R - smooth_motor_R) * MOTOR_SMOOTHING_FACTOR;
    
    // Use smoothed outputs
    *corrected_output_L = smooth_motor_L;
    *corrected_output_R = smooth_motor_R;
    
    // Debug output  
    static uint32_t debug_counter = 0;
    if (debug_counter++ % 5 == 0) { // Print every 5 cycles to see smoothing better
        printf("Heading Error: %+4.1f° | Correction: %+5.3f | Target: L=%.2f R=%.2f | Smooth: L=%.2f R=%.2f\n", 
               error_headings, correction, target_L, target_R, *corrected_output_L, *corrected_output_R);
    }
}


/**
 * @brief Enhanced robot movement function (forward only, no reverse)
 * @param motor_output_L Left motor PWM (0.0 to 1.0)
 * @param motor_output_R Right motor PWM (0.0 to 1.0)
 */
void enhanced_robot_movement(float motor_output_L, float motor_output_R) {
    // Use the existing robot_movement function from WithPID.c
    robot_movement(motor_output_L, motor_output_R);
}

/**
 * @brief Initialize baseline readings when robot is level and stable
 * @return true if baseline successfully established
 */
bool establish_baseline_readings(void) {
    printf("Establishing baseline readings...\n");
    
    // Try multiple times to get stable readings
    for (int attempts = 0; attempts < 5; attempts++) {
        bool accel_ok = read_accelerometer((int16_t*)base_g_data); // Cast needed for raw reading
        bool mag_ok = read_magnetometer((int16_t*)&base_headings);  // Cast needed for raw reading
        
        if (accel_ok && mag_ok) {
            // Convert raw accelerometer to g-force  
            int16_t raw_accel[3];
            if (read_accelerometer(raw_accel)) {
                float lpf_data[3], kalman_data[3];
                apply_low_pass_filter(raw_accel, lpf_data);
                apply_kalman_filter(lpf_data, kalman_data);
                convert_to_g(kalman_data, base_g_data);
            }
            
            // Get processed magnetometer data
            int16_t raw_mag[3];
            if (read_magnetometer(raw_mag)) {
                float mag_float[3], mag_lpf[3], mag_kalman[3];
                for (int i = 0; i < 3; i++) {
                    mag_float[i] = (float)raw_mag[i];
                }
                apply_mag_low_pass_filter(mag_float, mag_lpf);
                apply_mag_kalman_filter(mag_lpf, mag_kalman);
                base_headings = calculate_headings_3d(mag_kalman[0], mag_kalman[1], mag_kalman[2]);
            }
            
            // Check if robot is level
            if (is_robot_level_and_stable(base_g_data)) {
                printf("✓ Baseline established - Robot is level and stable\n");
                printf("  Base Heading XY: %.1f°\n", base_headings.xy_heading);
                printf("  Base Heading XZ: %.1f°\n", base_headings.xz_heading); 
                printf("  Base Heading YZ: %.1f°\n", base_headings.yz_heading);
                printf("  Base Accel: X=%.2fg Y=%.2fg Z=%.2fg\n", 
                       base_g_data[0], base_g_data[1], base_g_data[2]);
                return true;
            } else {
                printf("Robot not level, attempt %d/5...\n", attempts + 1);
                sleep_ms(500);
            }
        } else {
            printf("Failed to read sensors, attempt %d/5...\n", attempts + 1);
            sleep_ms(500);
        }
    }
    
    printf("ERROR: Could not establish stable baseline readings!\n");
    return false;
}

// Execute precise turn to specific angle (in degrees)
bool execute_precise_turn_angle(float current_heading, float target_angle) {
    printf("Executing precise turn: %.1f° → %.1f°\n", current_heading, target_angle);
    
    // Execute the turn with multiple correction steps
    uint32_t turn_start = to_ms_since_boot(get_absolute_time());
    const uint32_t turn_timeout = 3000;  // 3 second timeout
    
    while ((to_ms_since_boot(get_absolute_time()) - turn_start) < turn_timeout) {
        // Update IMU readings
        robot_state_t state;
        if (read_robot_state(&state) && state.imu_valid) {
            float heading_error = calculate_heading_difference(state.headings.xy_heading, target_angle);
            
            // Check if turn is complete
            if (fabsf(heading_error) < 5.0f) {
                stop_robot();
                printf("Precision turn complete! Final heading: %.1f° (error: %.1f°)\n", 
                       state.headings.xy_heading, heading_error);
                return true;
            }
            
            // Apply turn correction
            turn_output_t turn = calculate_turn_motors(state.headings.xy_heading, target_angle);
            set_motor_speed(turn.left_motor, turn.right_motor);
        }
        
        sleep_ms(20);  // Update at 50Hz for faster heading response
    }
    
    // Timeout - stop and report
    stop_robot();
    printf("Precision turn timeout! Check IMU or mechanical issues.\n");
    return false;
}

// ========== SIMPLE AUTONOMOUS NAVIGATION ==========

// Legacy function for barcode direction-based turns (kept for compatibility)
/*
void OLD_calculate_turn_motor_outputs_DISABLED(const char* direction, float current_heading, 
                                 float* motor_L, float* motor_R) {
    // Simple turning logic - adjust motor outputs for 90-degree turns
    if (strcmp(direction, "LEFT") == 0) {
        *motor_L = -0.4f;  // Left motor backward
        *motor_R = 0.4f;   // Right motor forward
        printf("🔄 Turning LEFT: Motor L=%.2f, R=%.2f\n", *motor_L, *motor_R);
    }
    else if (strcmp(direction, "RIGHT") == 0) {
        *motor_L = 0.4f;   // Left motor forward
        *motor_R = -0.4f;  // Right motor backward
        printf("� Turning RIGHT: Motor L=%.2f, R=%.2f\n", *motor_L, *motor_R);
    }
    else if (strcmp(direction, "AROUND") == 0) {
        *motor_L = -0.4f;  // Left motor backward
        *motor_R = 0.4f;   // Right motor forward (180-degree turn)
        printf("� Turning AROUND: Motor L=%.2f, R=%.2f\n", *motor_L, *motor_R);
    }
    else {
        // Unknown direction, don't turn
        *motor_L = 0.0f;
        *motor_R = 0.0f;
        printf("Unknown direction: %s\n", direction);
    }
}
*/

// Generalized motor output calculation for any angle turn (Future use)
void calculate_general_turn_motor_outputs(float turn_angle_degrees, float turn_power, 
                                         float* motor_L, float* motor_R) {
    // Normalize angle to -180 to +180 range
    while (turn_angle_degrees > 180.0f) turn_angle_degrees -= 360.0f;
    while (turn_angle_degrees < -180.0f) turn_angle_degrees += 360.0f;
    
    // Clamp turn power to safe range
    turn_power = fmaxf(0.1f, fminf(0.6f, turn_power));
    
    if (turn_angle_degrees > 0) {
        // Positive angle = turn right
        *motor_L = turn_power;   // Left motor forward
        *motor_R = -turn_power;  // Right motor backward
        printf("Turning RIGHT %.1f°: Motor L=%.2f, R=%.2f\n", 
               turn_angle_degrees, *motor_L, *motor_R);
    } else if (turn_angle_degrees < 0) {
        // Negative angle = turn left
        *motor_L = -turn_power;  // Left motor backward
        *motor_R = turn_power;   // Right motor forward
        printf("Turning LEFT %.1f°: Motor L=%.2f, R=%.2f\n", 
               fabsf(turn_angle_degrees), *motor_L, *motor_R);
    } else {
        // No turn needed
        *motor_L = 0.0f;
        *motor_R = 0.0f;
        printf("No turn needed (0°)\n");
    }
}

// Convert barcode direction to 90-degree angle for barcode-specific turns
float barcode_direction_to_angle(const char* direction) {
    if (strcmp(direction, "LEFT") == 0) {
        return -90.0f;  // Left turn = -90 degrees
    }
    else if (strcmp(direction, "RIGHT") == 0) {
        return 90.0f;   // Right turn = +90 degrees
    }
    else if (strcmp(direction, "AROUND") == 0) {
        return 180.0f;  // Around turn = 180 degrees
    }
    else {
        printf("Unknown barcode direction: %s\n", direction);
        return 0.0f;    // No turn for unknown direction
    }
}

// ========== MAIN PROGRAM ==========

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB connection
    
    printf("\n=== ROBO DRIVER V2 SYSTEM ===\n");
    printf("Demo 1: Stable Straight Movement without IR Sensor\n");
    printf("================================\n\n");
    
    // 1. Initialize motor system (critical - must work)
    printf("Initializing motor and encoder system...\n");
    if (!motor_system_init()) {
        printf("CRITICAL ERROR: Motor system failed to initialize!\n");
        printf("Cannot continue without motors - System halted.\n");
        return -1;
    }
    printf("✓ Motor and encoder system initialized\n\n");
    
    // 2. Try to initialize IMU system (optional - robot can work without it)
    bool imu_available = false;
    printf("Attempting IMU initialization on Grove 4 (GP16=SDA, GP17=SCL)...\n");
    i2c_initialize();
    printf("I2C bus initialized, attempting LSM303DLHC detection...\n");
    if (lsm303_init()) {
        printf("✓ IMU system initialized - heading correction available\n");
        
        // Try to establish baseline readings
        printf("Attempting IMU baseline calibration...\n");
        if (establish_baseline_readings()) {
            printf("✓ IMU calibration successful - full navigation available\n");
            imu_available = true;
        } else {
            printf("⚠ IMU calibration failed - will use IMU without baseline\n");
            // Set default baseline values
            base_g_data[0] = 0.0f; base_g_data[1] = 0.0f; base_g_data[2] = 1.0f;
            base_headings.xy_heading = 0.0f;
            base_headings.xz_heading = 0.0f;  
            base_headings.yz_heading = 0.0f;
            imu_available = true;  // Still use IMU, just without perfect baseline
        }
    } else {
        printf("⚠ IMU initialization failed - robot will move without heading correction\n");
        printf("  This is OK - robot can still move forward, just without compass\n");
        imu_available = false;
    }
    
    system_initialized = true;
    
    // 3. Initialize IR sensors for autonomous navigation
    printf("\nInitializing IR sensors for line tracking and barcode reading...\n");
    ir_track_init();                    // Initialize IR line tracer
    init_pin_and_button();             // Initialize barcode decoder pins
    enable_barcode_scanning();         // Enable barcode scanning
    printf("✓ IR line tracer initialized\n");
    printf("✓ IR barcode decoder initialized\n\n");
    
    printf("\n=== ROBO DRIVER V3 SIMPLE AUTONOMOUS SYSTEM ===\n");
    printf("Using existing IR functions for autonomous navigation:\n");
    printf("✓ IR Line Tracer - is_line_traced() for line detection\n");
    printf("✓ IR Barcode Decoder - char_to_command() for direction conversion\n");  
    printf("✓ IMU + PID Control - Precise movement with heading correction\n");
    printf("✓ Simple Motor Control - Direct motor output calculation for turns\n\n");
    printf("Simple Logic:\n");
    printf("1. If line detected: Continue following with PID+IMU\n");
    printf("2. If barcode ready: Decode → Calculate turn → Execute turn\n");
    printf("3. If no line: Stop motors\n");
    printf("System uses existing IR functions - no complex state machines!\n\n");
    
    // Initialize timing
    last_imu_update_ms = to_ms_since_boot(get_absolute_time());
    last_pid_update_ms = to_ms_since_boot(get_absolute_time());
    
    // Main control variables
    float current_g_data[3];           // Current accelerometer readings
    heading_3d_t current_headings;     // Current compass readings  
    float error_headings = 0.0f;       // Heading error for control
    bool accel_ok = false, mag_ok = false;
    
    // Motor control variables
    float motor_output_L = 0.0f, motor_output_R = 0.0f;
    float corrected_motor_L = 0.0f, corrected_motor_R = 0.0f;
    
    // Set initial target speed
    float target_speed_mm_per_s = 100.0f;  // Start with 200mm/s (moderate speed)
    set_target_speed(target_speed_mm_per_s);
    
    uint32_t loop_counter = 0;
    uint32_t last_status_print_ms = to_ms_since_boot(get_absolute_time());
    
    // =========== MAIN CONTROL LOOP ===========
    while (true) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        
        // Read IMU every 50ms (only if available)
        if (imu_available && (now_ms - last_imu_update_ms) >= IMU_UPDATE_INTERVAL_MS) {
            
            // Read accelerometer and magnetometer 
            int16_t raw_accel[3], raw_mag[3];
            accel_ok = read_accelerometer(raw_accel);
            mag_ok = read_magnetometer(raw_mag);
            
            if (accel_ok && mag_ok) {
                // Process accelerometer data
                float lpf_accel[3], kalman_accel[3];
                apply_low_pass_filter(raw_accel, lpf_accel);
                apply_kalman_filter(lpf_accel, kalman_accel);
                convert_to_g(kalman_accel, current_g_data);
                
                // Process magnetometer data  
                float mag_float[3], mag_lpf[3], mag_kalman[3];
                for (int i = 0; i < 3; i++) {
                    mag_float[i] = (float)raw_mag[i];
                }
                apply_mag_low_pass_filter(mag_float, mag_lpf);
                apply_mag_kalman_filter(mag_lpf, mag_kalman);
                current_headings = calculate_headings_3d(mag_kalman[0], mag_kalman[1], mag_kalman[2]);
                
                // Calculate heading error
                error_headings = calculate_heading_error(current_headings, base_headings);
            } else {
                // IMU read failed - disable IMU for this session
                printf("IMU read failed - disabling heading correction\n");
                imu_available = false;
                accel_ok = false;
                mag_ok = false;
            }
            
            last_imu_update_ms = now_ms;

        } else if (!imu_available) {
            // No IMU - set default values
            accel_ok = false;
            mag_ok = false;
            error_headings = 0.0f;  // No heading correction
        }
        
        // Update wheel speed measurements (only every 300ms to allow even more pulse accumulation)
        if (now_ms - last_speed_update_ms >= 300) {
            update_wheel_speeds();
            last_speed_update_ms = now_ms;
        }
        
        // ========== SMOOTH LINE-FOLLOWING SYSTEM ==========
        
        // Line following state variables (persistent across loop iterations)
        static float target_motor_speed = 0.0f;     // Ramped target speed
        static float current_motor_speed = 0.0f;    // Current ramped speed

        static bool was_on_line = false;            // Previous line state
        
        // Recovery variables removed for clean implementation
        
        // Simple line following system
        static float last_good_heading = 0.0f;      // IMU heading when last on line
        static uint32_t off_line_start = 0;         // When line was lost (for IMU return)
        static float last_on_line_heading = 0.0f;   // Heading when we were last on line  
        static float overshoot_correction = 0.0f;   // How much to correct back
        
        // 2. Robot movement based on IR Line Tracer value (as specified)
        line_trace_flag = is_line_traced();  // Update global flag: is_line_traced = TRUE (as specified)
        bool line_detected = line_trace_flag;  // If line is detected, robot will continue to follow line
        uint16_t raw_adc = get_line_adc_value();
        

        
        // STABILITY: Add hysteresis to prevent rapid on/off switching
        static bool was_on_line_stable = false;
        static uint32_t last_state_change = 0;
        
        // Only change state if we're confident (prevent rapid oscillation)
        if (line_detected != was_on_line_stable) {
            if ((now_ms - last_state_change) > 50) {  // 50ms minimum between state changes
                was_on_line_stable = line_detected;
                last_state_change = now_ms;
            } else {
                line_detected = was_on_line_stable;  // Keep previous stable state
            }
        }
        
        // Debug output for IR line state (every 500ms to avoid spam)
        static uint32_t last_ir_debug = 0;
        if ((now_ms - last_ir_debug) > 500) {
            printf("[IR DEBUG] ADC: %d, State: %s, Speed: %.2f->%.2f\n", 
                   raw_adc, line_detected ? "BLACK" : "WHITE", 
                   current_motor_speed, target_motor_speed);
            last_ir_debug = now_ms;
        }
        
        // LINE FOLLOWING STATE MACHINE
        if (line_detected) {
            // Line detected - ramp up speed smoothly
            if (!was_on_line) {
                printf("Line acquired - starting smooth acceleration\n");
                // Reset off-line tracking - return to normal correction mode
                off_line_start = 0;
            }
            
            // IMU PRECISION PLOTTING (as specified: "IMU must plot precision of movement as necessary")
            if (imu_available && accel_ok && mag_ok) {
                last_good_heading = current_headings.xy_heading;
                // IMU plots precision of movement for accurate line following
                static uint32_t last_heading_debug = 0;
                if ((now_ms - last_heading_debug) > 1000) {
                    printf("[IMU-PRECISION] Plotting movement precision: %.1f° for guidance\n", last_good_heading);
                    last_heading_debug = now_ms;
                }
            }
            
            // Set target speed and ramp up gradually (reduced for gentle cruising)
            target_motor_speed = 0.35f;  // Target 35% power for controlled line following (PWM boost handles stiction)
            
            was_on_line = true;
            
            // ========== NEW BARCODE LOGIC (as specified) ==========
            // 3. IR barcode decoder: if the start of a valid barcode is detected
            if (is_barcode_ready_to_decode()) {
                printf("[BARCODE] Start of valid barcode detected - implementing specified sequence\n");
                
                // --> 1. Robot slows down to accommodate for decoding latency rate
                printf("[BARCODE] Step 1: Slowing down for decoding latency\n");
                target_motor_speed = 0.15f;  // Slow down to 15% speed
                current_motor_speed = 0.15f;
                robot_movement(current_motor_speed, current_motor_speed);
                sleep_ms(200);  // Allow time to slow down
                
                // --> 2. IR Barcode Driver decodes barcode and gets a character, surrounded by * delimiters
                printf("[BARCODE] Step 2: Decoding barcode character (surrounded by * delimiters)\n");
                char decoded_char = get_decoded_barcode_char();  // Use public function (internally calls compareTwoArray)
                
                if (decoded_char != 0 && decoded_char != '*') {  // Valid character (not delimiter)
                    printf("[BARCODE] Step 2 Complete: Decoded character '%c'\n", decoded_char);
                    
                    // --> 3. IR Barcode Driver converts character to a direction [char_to_command function]
                    printf("[BARCODE] Step 3: Converting character to direction using char_to_command()\n");
                    const char* direction = char_to_command(decoded_char);  // Use specified function
                    
                    if (direction != NULL) {
                        printf("[BARCODE] Step 3 Complete: Direction = '%s'\n", direction);
                        
                        // --> 4. Main sys calculates heading and plots the change in direction
                        printf("[BARCODE] Step 4: Calculating heading and plotting direction change\n");
                        float current_heading = current_headings.xy_heading;  // Get current IMU heading
                        float target_heading = current_heading;
                        
                        // Calculate target heading based on direction (90-degree turns)
                        if (strcmp(direction, "RIGHT") == 0) {
                            target_heading = current_heading + 90.0f;
                            printf("[BARCODE] Step 4: Plotting RIGHT turn (+90°)\n");
                        } else if (strcmp(direction, "LEFT") == 0) {
                            target_heading = current_heading - 90.0f;
                            printf("[BARCODE] Step 4: Plotting LEFT turn (-90°)\n");
                        }
                        
                        // Normalize target heading
                        while (target_heading >= 360.0f) target_heading -= 360.0f;
                        while (target_heading < 0.0f) target_heading += 360.0f;
                        
                        printf("[BARCODE] Step 4 Complete: Heading change from %.1f° to %.1f°\n", 
                               current_heading, target_heading);
                        
                        // --> 5. Motors move based on calculated motor outputs to execute turn
                        printf("[BARCODE] Step 5: Calculating motor outputs and executing turn\n");
                        
                        // Stop briefly before turn
                        robot_movement(0.0f, 0.0f);
                        sleep_ms(300);
                        
                        // Execute 15-degree stepped turn using PDF method
                        printf("[BARCODE] Step 5: Executing stepped turn to %.1f°\n", target_heading);
                        execute_stepped_turn(target_heading);
                        
                        printf("[BARCODE] Step 5 Complete: 15-degree stepped turn executed\n");
                        
                    } else {
                        printf("[BARCODE] ERROR: char_to_command() returned invalid direction\n");
                    }
                } else {
                    printf("[BARCODE] No valid character decoded from compareTwoArray()\n");
                }
                
                // Reset to normal line following after barcode processing
                target_motor_speed = 0.0f;
                current_motor_speed = 0.0f;
            }
            
        }
        
        // ========== SMOOTH SPEED RAMPING (TIME-BASED) ==========
        // Much more gradual ramping based on time, not loop iterations
        static uint32_t last_ramp_time = 0;
        uint32_t ramp_interval_ms = 10;  // Update speed every 10ms for much faster response
        
        if ((now_ms - last_ramp_time) >= ramp_interval_ms) {
            float ramp_rate = 0.015f;  // Gentle ramping for smoother acceleration
            
            if (current_motor_speed < target_motor_speed) {
                current_motor_speed += ramp_rate;
                if (current_motor_speed > target_motor_speed) {
                    current_motor_speed = target_motor_speed;
                }
            } else if (current_motor_speed > target_motor_speed) {
                current_motor_speed -= ramp_rate;
                if (current_motor_speed < target_motor_speed) {
                    current_motor_speed = target_motor_speed;
                }
            }
            
            last_ramp_time = now_ms;
        }
        
        // Ensure minimum speed threshold (lowered threshold)
        if (current_motor_speed < 0.01f && target_motor_speed > 0.0f) {
            // Don't reset to 0 if we're trying to accelerate
            current_motor_speed = 0.01f;  // Keep minimum speed for ramping
        } else if (target_motor_speed == 0.0f && current_motor_speed < 0.01f) {
            current_motor_speed = 0.0f;  // Full stop when target is 0
        }
        
        // Debug ramping process
        static uint32_t last_ramp_debug = 0;
        if ((now_ms - last_ramp_debug) > 500) {
            printf("[RAMP DEBUG] Target: %.2f, Current: %.2f, Line: %s\n", 
                   target_motor_speed, current_motor_speed, line_detected ? "YES" : "NO");
            last_ramp_debug = now_ms;
        }
        
        // ========== SIMPLIFIED LINE CORRECTION SYSTEM ==========
        float line_correction_L = 0.0f, line_correction_R = 0.0f;
        
        // Simple line correction - update every 5ms
        static uint32_t last_line_update = 0;
        uint32_t line_update_interval_ms = 5;
        
        if (current_motor_speed > 0.0f && (now_ms - last_line_update) >= line_update_interval_ms) {
            
            float correction = 0.0f;  // Motor correction to apply
            
            // ENHANCED LINE CORRECTION LOGIC with predictive corrections
            if (line_detected) {
                // ON LINE: Enhanced correction with predictive elements
                float optimal_adc = 3200.0f;  // Center target
                float adc_error = (float)raw_adc - optimal_adc;
                
                // DISABLED PREDICTIVE CORRECTION - Was interfering with precision control
                // The predictive logic was artificially boosting errors by 100x,
                // causing massive corrections that bypassed all precision zones
                // static float last_stored_heading = 0.0f;  // UNUSED - removed to fix warning
                
                // GENTLE correction strength based on error magnitude (back to basics)
                float error_magnitude = fabs(adc_error);
                float scale_factor;
                float max_correction;
                
                // FAST-ACTING PRECISE LINE TRACKING - Quick, calculated responses
                float precision_factor = 1.0f;
                
                if (error_magnitude < 50.0f) {
                    // PRECISION ZONE: Ultra-fine, immediate micro-corrections
                    scale_factor = 12000.0f;   // Very high sensitivity for instant response
                    max_correction = 0.025f;   // 2.5% - tiny but immediate
                    precision_factor = 1.0f;   // Pure precision, no amplification
                } else if (error_magnitude < 120.0f) {
                    // REACTIVE ZONE: Fast, measured corrections
                    scale_factor = 8000.0f;    // High sensitivity for quick response
                    max_correction = 0.050f;   // 5% - small but responsive
                    precision_factor = 1.0f;   // Calculated, not aggressive
                } else if (error_magnitude < 250.0f) {
                    // CORRECTION ZONE: Balanced, swift corrections
                    scale_factor = 4000.0f;    // Medium sensitivity
                    max_correction = 0.090f;   // 9% - measured response
                    precision_factor = 1.0f;   // No over-amplification
                } else {
                    // RECOVERY ZONE: Firm but controlled return
                    scale_factor = 2500.0f;    // Lower sensitivity to prevent overshoot
                    max_correction = 0.150f;   // 15% - firm but not violent
                    precision_factor = 1.0f;   // Controlled response
                }
                
                correction = adc_error / scale_factor;  
                correction *= precision_factor;  // Apply calculated precision response
                correction = fmaxf(-max_correction, fminf(max_correction, correction));
                
                // Store heading when well-centered (using last_good_heading instead)
                if (fabs(adc_error) < 200.0f && imu_available && accel_ok && mag_ok) {
                    robot_state_t temp_state;
                    if (read_robot_state(&temp_state) && temp_state.imu_valid) {
                        last_good_heading = temp_state.headings.xy_heading;  // Use existing variable
                    }
                }
                
                printf("[LINE-CORRECT] ON-LINE ADC:%d, Error:%.0f, Scale:%.0f, Precision:%.1fx, Correction:%.3f (max:%.3f)\n", 
                       raw_adc, adc_error, scale_factor, precision_factor, correction, max_correction);
                
            } else {
                // OFF LINE: IMU-guided return-to-line correction
                was_on_line = false;  // Ensure state is properly tracked
                
                // Set base search speed so robot can move during search
                target_motor_speed = 0.15f;  // Moderate speed for line searching
                
                if (off_line_start == 0) {
                    off_line_start = now_ms;
                    
                    // Store the heading when we lost the line (if IMU available)
                    if (imu_available && accel_ok && mag_ok) {
                        last_on_line_heading = current_headings.xy_heading;
                        printf("[IMU-RETURN] Lost line! Stored heading: %.1f°, ADC:%d\n", 
                               last_on_line_heading, raw_adc);
                    } else {
                        last_on_line_heading = -999.0f;  // Invalid heading
                        printf("[LINE-CORRECT] OFF-LINE! No IMU, ADC:%d\n", raw_adc);
                    }
                    
                    // ASYMMETRIC CORRECTION: Stronger correction for right overshoot (harder to recover)
                    if (raw_adc > 3200) {
                        // Right overshoot is harder to recover - use stronger correction
                        overshoot_correction = -0.12f;  // Robot RIGHT of line → STRONGER LEFT turn
                        printf("[IMU-RETURN] Overshot RIGHT (ADC:%d > 3200), STRONG LEFT correction\n", raw_adc);
                    } else {
                        // Left overshoot recovers easier - keep gentle
                        overshoot_correction = +0.08f;  // Robot LEFT of line → Gentle RIGHT turn
                        printf("[IMU-RETURN] Overshot LEFT (ADC:%d < 3200), GENTLE RIGHT correction\n", raw_adc);
                    }
                }
                
                uint32_t off_time = now_ms - off_line_start;
                
                // IMU-GUIDED RETURN: Use heading information if available
                if (last_on_line_heading != -999.0f && imu_available && accel_ok && mag_ok) {
                    // Calculate how much we've deviated from the stored heading
                    float current_heading = current_headings.xy_heading;
                    float heading_error = calculate_heading_difference(current_heading, last_on_line_heading);
                    
                    if (off_time < 300) {
                        // PHASE 1: GENTLE magnetic heading correction - smooth and controlled
                        float heading_based_correction = -heading_error * 0.008f;  // Very gentle response
                        heading_based_correction = fmaxf(-0.06f, fminf(0.06f, heading_based_correction));  // Conservative limits
                        
                        // Combine corrections for smooth magnetic pull without violence
                        correction = (overshoot_correction * 0.7f) + (heading_based_correction * 0.4f);
                        
                        printf("[IMU-RETURN] FIXED correction: heading_error=%.1f°, overshoot=%.3f, heading_based=%.3f, final=%.3f\n",
                               heading_error, overshoot_correction, heading_based_correction, correction);
                        
                    } else if (off_time < 800) {
                        // PHASE 2: GENTLE magnetic heading return - smooth and controlled
                        correction = -heading_error * 0.012f;  // Gentle magnetic pull
                        correction = fmaxf(-0.10f, fminf(0.10f, correction));  // Conservative power limits
                        
                        printf("[IMU-RETURN] FIXED HEADING return: error=%.1f°, correction=%.3f (time=%dms)\n",
                               heading_error, correction, off_time);
                        
                    } else {
                        // PHASE 3: GENTLE SEARCH - controlled movement
                        if (off_time < 1200) {
                            correction = overshoot_correction * 1.5f;  // Controlled same direction
                        } else if (off_time < 1600) {
                            correction = -overshoot_correction * 1.5f;  // Controlled opposite direction
                        } else {
                            off_line_start = 0;  // Reset and try again
                            correction = 0.0f;
                        }
                        
                        printf("[IMU-RETURN] FALLBACK search: correction=%.3f (time=%dms)\n",
                               correction, off_time);
                    }
                    
                } else {
                    // NO IMU: GENTLE search pattern - smooth and controlled
                    if (off_time < 500) {
                        correction = overshoot_correction * 1.2f;  // Gentle initial search
                    } else if (off_time < 1000) {
                        correction = -overshoot_correction * 1.2f; // Gentle opposite search
                    } else {
                        off_line_start = 0;
                        correction = 0.0f;
                    }
                    
                    printf("[LINE-CORRECT] NO-IMU Search: correction=%.3f (time=%dms)\n",
                           correction, off_time);
                }
                
                // TIMEOUT CHECK: Stop robot if line search exceeds maximum time
                const uint32_t max_search_time = 3000;  // 3 seconds maximum search time
                if (off_time > max_search_time) {
                    printf("[LINE-CORRECTION-TIMEOUT] Failed to find line after %dms - STOPPING ROBOT!\n", off_time);
                    target_motor_speed = 0.0f;     // Stop the robot
                    current_motor_speed = 0.0f;    // Immediate stop
                    correction = 0.0f;             // No corrections
                    off_line_start = 0;            // Reset timer
                }
            }
            
            // ALWAYS APPLY CORRECTIONS - No blocking conditions
            line_correction_L = current_motor_speed + correction;  // Left motor
            line_correction_R = current_motor_speed - correction;  // Right motor
            
            // Ensure motors stay within reasonable bounds
            line_correction_L = fmaxf(0.0f, fminf(0.6f, line_correction_L));
            line_correction_R = fmaxf(0.0f, fminf(0.6f, line_correction_R));
            
            printf("[LINE-CORRECT] Motors: Base=%.3f, L=%.3f, R=%.3f (correction=%.3f)\n", 
                   current_motor_speed, line_correction_L, line_correction_R, correction);
            
            last_line_update = now_ms;
            
        } else {
            // Not time for update or stopped - use base speed
            line_correction_L = current_motor_speed;
            line_correction_R = current_motor_speed;
        }
        
        // ========== COOPERATIVE IMU + LINE CORRECTION ==========
        
        // Start with line-corrected motor outputs
        motor_output_L = line_correction_L;
        motor_output_R = line_correction_R;
        
        // CRITICAL: Check if robot should actually stop (override all other systems)
        bool should_stop = (target_motor_speed <= 0.0f && current_motor_speed <= 0.03f);
        
        // Apply motor control based on smooth ramping and line correction
        // Allow movement if: motor speed > 0 OR we're off-line (IMU corrections) OR corrections are significant
        bool allow_movement = (current_motor_speed > 0.0f) || (!line_detected) || (fabs(motor_output_L - motor_output_R) > 0.02f);
        
        if (allow_movement && !should_stop) {
            // Normal line-following mode - clamp outputs to safe range
            motor_output_L = fmaxf(0.0f, fminf(0.5f, motor_output_L));
            motor_output_R = fmaxf(0.0f, fminf(0.5f, motor_output_R));
            
            // Normal line-following motor outputs
            corrected_motor_L = motor_output_L;
            corrected_motor_R = motor_output_R;
            
            // Apply motor calibration to compensate for hardware differences
            float calibrated_motor_L = corrected_motor_L * MOTOR_CALIBRATION_LEFT;
            float calibrated_motor_R = corrected_motor_R * MOTOR_CALIBRATION_RIGHT;
            
            // SMART STICTION HANDLING: Preserve corrections while ensuring movement
            const float MIN_STICTION_THRESHOLD = 0.45f;
            
            // Calculate the difference (correction amount) before stiction
            float motor_diff = calibrated_motor_L - calibrated_motor_R;
            float avg_motor = (calibrated_motor_L + calibrated_motor_R) / 2.0f;
            
            // Only apply stiction if BOTH motors are below threshold (preventing movement)
            if (fabs(calibrated_motor_L) < MIN_STICTION_THRESHOLD && fabs(calibrated_motor_R) < MIN_STICTION_THRESHOLD) {
                // Both motors below threshold - boost while preserving difference
                if (avg_motor >= 0.0f) {
                    // Forward motion - boost to threshold while keeping difference
                    calibrated_motor_L = MIN_STICTION_THRESHOLD + (motor_diff / 2.0f);
                    calibrated_motor_R = MIN_STICTION_THRESHOLD - (motor_diff / 2.0f);
                } else {
                    // Reverse motion - boost to negative threshold while keeping difference  
                    calibrated_motor_L = -MIN_STICTION_THRESHOLD + (motor_diff / 2.0f);
                    calibrated_motor_R = -MIN_STICTION_THRESHOLD - (motor_diff / 2.0f);
                }
            }
            // If at least one motor is above threshold, corrections will work - don't interfere
            
            // DIAGNOSTIC: Check if motors are actually being commanded
            static uint32_t last_motor_debug = 0;
            if ((now_ms - last_motor_debug) > 200) {  // Every 200ms for faster debug
                printf("[MOTOR DEBUG] Sending PWM: L=%.3f R=%.3f | Expected movement: %s\n", 
                       calibrated_motor_L, calibrated_motor_R,
                       (calibrated_motor_L > 0.05f || calibrated_motor_R > 0.05f) ? "YES" : "NO");
                last_motor_debug = now_ms;
            }
            
            // Apply PWM to robot movement function
            robot_movement(calibrated_motor_L, calibrated_motor_R);
            
        } else {
            // Speed is 0 OR robot should stop - stop motors completely
            robot_movement(0.0f, 0.0f);
            corrected_motor_L = 0.0f;
            corrected_motor_R = 0.0f;
            
            // Debug when stopping
            static uint32_t last_stop_debug = 0;
            if ((now_ms - last_stop_debug) > 1000) {
                if (should_stop) {
                    printf("[MOTOR DEBUG] STOPPED - target=%.2f, current=%.2f\n", 
                           target_motor_speed, current_motor_speed);
                } else {
                    printf("[MOTOR DEBUG] STOPPED - current_motor_speed=%.2f\n", current_motor_speed);
                }
                last_stop_debug = now_ms;
            }
        }
        
        // Enhanced status reporting every 2 seconds (reduced frequency)
        if ((now_ms - last_status_print_ms) >= 2000) {
            // Show both corrected and calibrated motor values for tuning
            float cal_L = (current_motor_speed > 0.0f) ? corrected_motor_L * MOTOR_CALIBRATION_LEFT : 0.0f;
            float cal_R = (current_motor_speed > 0.0f) ? corrected_motor_R * MOTOR_CALIBRATION_RIGHT : 0.0f;
            
            printf("[%lu] Line:%s | Target:%.2f Current:%.2f | Raw: L=%.2f R=%.2f | Cal: L=%.2f R=%.2f | Speeds: L=%4.0f R=%4.0f mm/s\n",
                   loop_counter / 100,  // Approximate seconds
                   line_detected ? "ON" : "OFF",
                   target_motor_speed, current_motor_speed,
                   corrected_motor_L, corrected_motor_R,  // Raw values
                   cal_L, cal_R,  // Calibrated values
                   get_last_speed_L_mm_per_s(), get_last_speed_R_mm_per_s());
            
            if (imu_available && (accel_ok && mag_ok)) {
                printf("     IMU: Heading=%.1f° Error=%+.1f° | ADC=%d\n",
                       current_headings.xy_heading, error_headings, raw_adc);
            }
            
            last_status_print_ms = now_ms;
        }

        
        loop_counter++;
        
        // Balanced control loop - fast but stable
        sleep_ms(5);   // 200Hz control loop for smooth, responsive line following
    }
    
    return 0;
}

