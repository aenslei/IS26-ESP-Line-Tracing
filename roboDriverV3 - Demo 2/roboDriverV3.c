#include "roboDriver_V3.h"  // Includes everything
#include "IMUDriver.h"  // Direct IMU access
#include "WithPID.h"  // Direct PID access
#include "IR_track.h" //IR Line Tracing funcs
#include "decode_barcode.h" //Barcode decoding
#include <string.h> // For strcmp, memset

// ========== MOTOR CALIBRATION CONSTANTS ==========
// These compensate for hardware differences between left and right motors
// Adjust these values to make robot go straight when both motors get same PWM
#define MOTOR_CALIBRATION_LEFT  1.0f    // Left motor multiplier (baseline)
#define MOTOR_CALIBRATION_RIGHT 1.02f   // Right motor multiplier (REDUCED - was overwhelming line corrections)

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
    
    // Initialize IR Line Tracer
    ir_track_init();
    printf("✓ IR Line Tracer initialized on GP28\n");
    
    // Initialize IR Barcode Decoder  
    init_pin_and_button();
    printf("✓ IR Barcode Decoder initialized\n");
    
    printf("✓ Robot system fully initialized with IR sensors!\n\n");
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
    return imu_get_compass_heading();
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
        
        sleep_ms(50);  // Update at 20Hz
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
static float smooth_motor_L = 0.35f;   // Smoothed left motor output
static float smooth_motor_R = 0.35f;   // Smoothed right motor output
static bool motor_smoothing_initialized = false;



// Control parameters
#define MAX_TILT_ANGLE 15.0f          // Maximum tilt angle to consider "level" (degrees)
// Heading correction parameters  
#define HEADING_CORRECTION_GAIN 0.0008f  // Much smaller gain to prevent hitting maximum
#define HEADING_DEADBAND_DEGREES 10.0f   // Even larger deadband for stability
#define MAX_HEADING_CORRECTION 0.03f     // Reduced max correction to 3%
#define MIN_MOTOR_OUTPUT 0.1f             // Minimum motor output to maintain control
#define IMU_UPDATE_INTERVAL_MS 200    // Slower IMU updates (200ms) for more stability
#define PID_UPDATE_INTERVAL_MS 100    // Run PID every 100ms
#define MOTOR_SMOOTHING_FACTOR 0.08f  // Even stronger smoothing (very slow changes)

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
        
        sleep_ms(50);  // Update at 20Hz
    }
    
    // Timeout - stop and report
    stop_robot();
    printf("Precision turn timeout! Check IMU or mechanical issues.\n");
    return false;
}

// ========== SIMPLE AUTONOMOUS NAVIGATION ==========

// Legacy function for barcode direction-based turns (kept for compatibility)
void calculate_turn_motor_outputs(const char* direction, float current_heading, 
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
        printf("⚠️ Unknown direction: %s\n", direction);
    }
}

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
        static uint32_t line_lost_time = 0;         // When line was lost
        static bool was_on_line = false;            // Previous line state
        static uint32_t last_line_correction = 0;   // Last line correction attempt
        
        // IMU-GUIDED RETURN-TO-LINE SYSTEM
        static float last_good_heading = 0.0f;      // IMU heading when last on line
        static bool heading_stored = false;          // Whether we have a valid stored heading
        static uint32_t heading_store_time = 0;     // When heading was stored
        
        bool line_detected = is_line_traced();
        uint16_t raw_adc = get_line_adc_value();
        
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
                line_lost_time = 0;  // Reset lost timer
                heading_stored = false;  // Reset heading system when regaining line
            }
            
            // STORE CURRENT HEADING when on line for return-to-line guidance
            if (imu_available && accel_ok && mag_ok) {
                last_good_heading = current_headings.xy_heading;
                heading_stored = true;
                heading_store_time = now_ms;
                // Debug: Only print occasionally to avoid spam
                static uint32_t last_heading_debug = 0;
                if ((now_ms - last_heading_debug) > 1000) {
                    printf("[IMU-STORE] Stored heading: %.1f° for return guidance\n", last_good_heading);
                    last_heading_debug = now_ms;
                }
            }
            
            // Set target speed and ramp up gradually (reduced for smoother operation)
            target_motor_speed = 0.3f;  // Target 20% power for very stable line following
            was_on_line = true;
            
            // Check for barcode processing
            if (is_barcode_ready_to_decode()) {
                // Barcode detected - handle barcode turn
                printf("Barcode detected, processing turn...\n");
                
                // Slow down for decoding
                target_motor_speed = 0.1f;
                current_motor_speed = 0.1f;
                robot_movement(current_motor_speed, current_motor_speed);
                sleep_ms(100);
                
                char decoded_char = get_decoded_barcode_char();
                if (decoded_char != 0) {
                    printf("Decoded character: %c\n", decoded_char);
                    
                    const char* direction = char_to_command(decoded_char);
                    if (direction) {
                        printf("Direction command: %s\n", direction);
                        
                        // Execute smooth turn sequence
                        robot_movement(0.0f, 0.0f);  // Stop
                        current_motor_speed = 0.0f;
                        target_motor_speed = 0.0f;
                        sleep_ms(300);
                        
                        float turn_angle = barcode_direction_to_angle(direction);
                        float turn_power = 0.35f;  // Slightly reduced turn power
                        
                        float turn_motor_L, turn_motor_R;
                        calculate_general_turn_motor_outputs(turn_angle, turn_power, 
                                                           &turn_motor_L, &turn_motor_R);
                        
                        robot_movement(turn_motor_L, turn_motor_R);
                        sleep_ms(1200);  // Slightly reduced turn time
                        
                        robot_movement(0.0f, 0.0f);  // Stop after turn
                        current_motor_speed = 0.0f;
                        target_motor_speed = 0.0f;
                        sleep_ms(200);
                        printf("Turn completed\n");
                    }
                }
                // Reset to normal line following after barcode
                target_motor_speed = 0.0f;
                current_motor_speed = 0.0f;
            }
            
        } else {
            // No line detected
            if (was_on_line) {
                // Just lost the line - start timer
                line_lost_time = now_ms;
                printf("Line lost - beginning smooth deceleration\n");
            }
            
            was_on_line = false;
            
            // Gradual deceleration when line is lost
            if (line_lost_time == 0) line_lost_time = now_ms;  // Initialize if not set
            
            uint32_t time_without_line = now_ms - line_lost_time;
            
            if (time_without_line < 1500) {
                // First 1500ms - MAINTAIN GOOD SPEED for return-to-line corrections
                target_motor_speed = 0.22f;  // Keep decent speed for corrections to work
                printf("[RETURN-TO-LINE] Active search: %.2f (t=%dms)\n", target_motor_speed, time_without_line);
            } else if (time_without_line < 3000) {
                // 1.5-3 seconds - continue search with reduced speed
                target_motor_speed = 0.15f;  // Still enough for corrections
                printf("[RETURN-TO-LINE] Extended search: %.2f (t=%dms)\n", target_motor_speed, time_without_line);
            } else if (time_without_line < 4000) {
                // 3-4 seconds - final attempt with minimal speed
                target_motor_speed = 0.10f;  // Last chance
                printf("[RETURN-TO-LINE] Final search: %.2f (t=%dms)\n", target_motor_speed, time_without_line);
            } else {
                // After 4 seconds - stop completely (give up)
                target_motor_speed = 0.0f;
                static uint32_t last_stop_msg = 0;
                if ((now_ms - last_stop_msg) > 3000) {
                    printf("Line lost for >4000ms - motors stopped\n");
                    last_stop_msg = now_ms;
                }
            }
        }
        
        // ========== SMOOTH SPEED RAMPING (TIME-BASED) ==========
        // Much more gradual ramping based on time, not loop iterations
        static uint32_t last_ramp_time = 0;
        uint32_t ramp_interval_ms = 50;  // Update speed every 50ms for smoother ramping
        
        if ((now_ms - last_ramp_time) >= ramp_interval_ms) {
            float ramp_rate = 0.01f;  // Much smaller increment for very gradual ramping
            
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
        
        // ========== IMPROVED LINE CORRECTION SYSTEM ==========
        float line_correction_L = 0.0f, line_correction_R = 0.0f;
        
        // (Removed unused PID variables)
        
        // Faster line correction updates (separate from speed ramping)
        static uint32_t last_line_update = 0;
        uint32_t line_update_interval_ms = 20;  // Update line correction every 20ms (faster response)
        
        // ALWAYS run correction logic when motors are moving (whether line detected or not)
        if (current_motor_speed > 0.0f) {
            // Updated for new threshold (1500 from IR_track.c)
            
            // REFINED LINE POSITION DETECTION for 1cm narrow line
            // For a 1cm line, we need very subtle position detection
            
            float normalized_error = 0.0f;
            
            // Shared variable to remember last position for return-to-line logic
            static float last_on_line_position = 0.0f;
            
            // ========== LINE CORRECTION: BOTH ON-LINE AND OFF-LINE ==========
            
            if (line_detected) {
                // ON LINE: Use proportional control based on distance from optimal center
                
                // Define optimal ADC center point (where robot should stay)
                float optimal_adc = 3200.0f;  // Target ADC reading for perfect centering
                
                // Calculate how far we are from optimal center
                float adc_deviation = (float)raw_adc - optimal_adc;
                
                // Remember this position for return-to-line logic
                last_on_line_position = adc_deviation;
                
                // Use IMU heading information to inform correction strength and direction
                float correction_scaling = 4500.0f;  // FASTER response scaling (was 6000)
                float max_correction = 0.045f;       // STRONGER base correction (was 0.035)
                
                // EDGE DETECTION: More aggressive for faster response
                float adc_distance_from_edge = fabs(adc_deviation);
                bool near_edge = (adc_distance_from_edge > 150.0f);  // MORE SENSITIVE threshold (was 200)
                
                if (near_edge) {
                    // PROGRESSIVE corrections - FASTER and STRONGER
                    float edge_factor = (adc_distance_from_edge - 150.0f) / 250.0f;  // Smaller range for faster scaling
                    edge_factor = fminf(edge_factor, 1.0f);  // Cap at 1.0
                    
                    // Interpolate between normal and edge correction strength - MORE AGGRESSIVE
                    correction_scaling = 4500.0f - (edge_factor * 2500.0f);  // 4500 -> 2000 (much faster response)
                    max_correction = 0.045f + (edge_factor * 0.035f);        // 0.045 -> 0.080 (stronger corrections)
                }
                // If IMU is available, use heading error to further adjust correction behavior
                else if (imu_available && accel_ok && mag_ok) {
                    // Use IMU heading to determine if we need stronger or gentler corrections
                    float heading_magnitude = fabs(error_headings);
                    
                    if (heading_magnitude > 5.0f) {
                        // Large heading error - be more responsive but not excessive
                        correction_scaling = 4000.0f;   // Responsive correction (was 2000)
                        max_correction = 0.050f;        // Reasonable corrections (was 0.080)
                    } else if (heading_magnitude < 2.0f) {
                        // Small heading error - gentle to prevent overcorrection
                        correction_scaling = 8000.0f;   // Gentler (was 5000)
                        max_correction = 0.025f;        // Smaller corrections (was 0.040)
                    }
                    // Medium heading error (2-5°) uses base values (6000, 0.035)
                }
                
                // Apply the dynamically adjusted correction
                normalized_error = adc_deviation / correction_scaling;
                normalized_error = fmaxf(-max_correction, fminf(max_correction, normalized_error));
                
            } else if (!line_detected) {
                // OFF LINE (!line_detected): IMU-GUIDED RETURN-TO-LINE SYSTEM
                static uint32_t off_line_start = 0;
                
                if (off_line_start == 0) {
                    off_line_start = now_ms;
                    printf("[IMU-RETURN] WHITE DETECTED! ADC:%d Last_pos:%.1f - STARTING IMU-GUIDED RETURN\n", 
                           raw_adc, last_on_line_position);
                }
                
                uint32_t off_time = now_ms - off_line_start;
                
                // SMART IMU-GUIDED CORRECTIONS: Only use IMU if it helps return to line
                if (heading_stored && imu_available && accel_ok && mag_ok && (now_ms - heading_store_time < 5000)) {
                    // We have a valid stored heading from when we were on the line
                    float current_heading = current_headings.xy_heading;
                    float heading_error = calculate_heading_difference(current_heading, last_good_heading);
                    
                    printf("[IMU-RETURN] Current: %.1f°, Target: %.1f°, Error: %.1f°, OffTime: %dms\n", 
                           current_heading, last_good_heading, heading_error, off_time);
                    
                    // SMART CORRECTION: Use different strategies based on how long we've been off-line
                    if (off_time < 300) {
                        // EARLY OFF-LINE: Use position-based correction (more reliable)
                        if (last_on_line_position > 0) {
                            normalized_error = -0.10f;  // Strong left turn to recover from right drift
                        } else {
                            normalized_error = +0.10f;  // Strong right turn to recover from left drift
                        }
                        printf("[IMU-RETURN] EARLY: Position-based correction: %.3f (ignoring IMU initially)\n", normalized_error);
                        
                    } else if (fabs(heading_error) > 15.0f) {
                        // MAJOR DEVIATION: Robot has turned way off course, use strong position correction
                        if (last_on_line_position > 0) {
                            normalized_error = -0.12f;  // Strong left turn
                        } else {
                            normalized_error = +0.12f;  // Strong right turn
                        }
                        printf("[IMU-RETURN] MAJOR DEVIATION (%.1f°): Using strong position correction: %.3f\n", 
                               heading_error, normalized_error);
                        
                    } else if (fabs(heading_error) > 5.0f) {
                        // MODERATE DEVIATION: Use gentle IMU guidance
                        normalized_error = heading_error * 0.008f;  // Gentler scaling for off-line
                        normalized_error = fmaxf(-0.08f, fminf(0.08f, normalized_error));  // Smaller cap
                        
                        printf("[IMU-RETURN] MODERATE: Gentle IMU correction: %.3f (from heading error: %.1f°)\n", 
                               normalized_error, heading_error);
                    } else {
                        // SMALL DEVIATION: Use position-based correction as it's more reliable
                        if (last_on_line_position > 0) {
                            normalized_error = -0.06f;  // Gentle left turn
                        } else {
                            normalized_error = +0.06f;  // Gentle right turn
                        }
                        printf("[IMU-RETURN] SMALL ERROR: Position guidance preferred: %.3f\n", normalized_error);
                    }
                    
                } else {
                    // No IMU guidance available - fall back to position-based corrections
                    printf("[IMU-RETURN] No IMU guidance (stored:%s, imu:%s, age:%dms), using fallback\n",
                           heading_stored ? "yes" : "no", 
                           (imu_available && accel_ok && mag_ok) ? "yes" : "no",
                           heading_stored ? (now_ms - heading_store_time) : 0);
                    
                    if (off_time < 400) {
                        // Initial correction based on last known position
                        if (last_on_line_position > 0) {
                            normalized_error = -0.08f;  // Strong left turn (reduced from 0.10)
                        } else {
                            normalized_error = +0.08f;  // Strong right turn (reduced from 0.10)
                        }
                        printf("[RETURN-FALLBACK] Position-based correction: %.3f\n", normalized_error);
                    } else if (off_time < 800) {
                        // Try opposite direction but gentler
                        if (last_on_line_position > 0) {
                            normalized_error = +0.06f;  // Try right (gentler)
                        } else {
                            normalized_error = -0.06f;  // Try left (gentler)
                        }
                        printf("[RETURN-FALLBACK] Opposite direction (gentle): %.3f\n", normalized_error);
                    } else {
                        // After 800ms, reset and try again
                        if (off_time > 1000) {
                            off_line_start = 0;  // Reset sooner
                        }
                        normalized_error = 0.0f;
                    }
                }
            } else {
                // line_detected is TRUE but raw_adc < 1500 (edge of line)
                // Use gentle edge corrections
                float adc_deviation = (float)raw_adc - 3200.0f;
                last_on_line_position = adc_deviation;
                normalized_error = adc_deviation / 8000.0f;  // Gentle edge correction
                normalized_error = fmaxf(-0.03f, fminf(0.03f, normalized_error));
                printf("[EDGE-CORRECT] ADC:%d, Edge correction: %.3f\n", raw_adc, normalized_error);
            }
            
            // DIRECT BANG-BANG MOTOR CONTROL - Simple and Responsive
            if ((now_ms - last_line_update) >= line_update_interval_ms) {
                
                // *** RAMPING PROTECTION *** 
                // Prevent line corrections during initial acceleration to ensure balanced motors
                static uint32_t acceleration_start_time = 0;
                bool is_accelerating = (current_motor_speed < target_motor_speed);
                
                if (is_accelerating && acceleration_start_time == 0) {
                    acceleration_start_time = now_ms;  // Mark start of acceleration
                }
                
                if (!is_accelerating) {
                    acceleration_start_time = 0;  // Reset when not accelerating
                }
                
                // SIMPLIFIED: Always allow corrections during first 4 seconds (critical period) OR after stabilization
                uint32_t time_since_acceleration = (acceleration_start_time > 0) ? (now_ms - acceleration_start_time) : 0;
                bool in_critical_period = (time_since_acceleration < 4000);  // First 4 seconds are critical
                bool stable_period = (time_since_acceleration > 2000);       // After 2 seconds is stable
                
                bool allow_corrections = (!is_accelerating) ||  // Always when not accelerating
                                        in_critical_period ||    // Always during first 4 seconds
                                        stable_period;           // Always after stabilization
                
                // EMERGENCY OVERRIDE: Force corrections for significant line deviation
                float current_adc_deviation = (float)raw_adc - 3200.0f;
                bool needs_emergency_correction = (fabs(current_adc_deviation) > 300.0f) || (raw_adc < 1500);
                
                if (needs_emergency_correction) {
                    allow_corrections = true;  // Override everything for line emergencies
                }
                
                // DEBUG: Show correction blocking status when off-line
                if (raw_adc < 1500) {
                    printf("[CORRECTION-DEBUG] allow=%s | accel=%s | critical=%s(%.1fs) | stable=%s | emergency=%s\n",
                           allow_corrections ? "YES" : "NO",
                           is_accelerating ? "YES" : "NO",
                           in_critical_period ? "YES" : "NO", time_since_acceleration/1000.0f,
                           stable_period ? "YES" : "NO",
                           needs_emergency_correction ? "YES" : "NO");
                }
                
                // RE-ENABLE LINE CORRECTIONS with smoothed critical period boost
                static float previous_motor_correction = 0.0f;
                float motor_correction = 0.0f;
                
                if (allow_corrections) {
                    motor_correction = normalized_error;  // Apply line corrections
                    
                    // DEBUG: Show if corrections are being calculated
                    if (raw_adc < 1500) {
                        printf("[RETURN-DEBUG] Correction allowed: %.3f (from normalized_error: %.3f)\n", 
                               motor_correction, normalized_error);
                    }
                    
                    // MODERATE CRITICAL PERIOD BOOST: Extra corrections in first 2 seconds but not excessive
                    if (in_critical_period && time_since_acceleration < 2000) {
                        float old_correction = motor_correction;
                        motor_correction *= 1.3f;  // Moderate boost (was 2.0f)
                        motor_correction = fmaxf(-0.08f, fminf(0.08f, motor_correction));  // Cap at ±0.08 (was 0.15)
                        if (raw_adc < 1500) {
                            printf("[RETURN-DEBUG] Critical period boost: %.3f -> %.3f\n", old_correction, motor_correction);
                        }
                    }
                    
                    // FAST RESPONSE SMOOTHING: Minimal smoothing for quick corrections
                    float smoothing_factor;
                    if (raw_adc < 1500) {
                        // OFF-LINE: Minimal smoothing for fast recovery while preventing violent overcorrection
                        smoothing_factor = 0.8f;  // 80% new, 20% previous - much faster response
                        motor_correction = fmaxf(-0.08f, fminf(0.08f, motor_correction));  // Increased cap for stronger corrections
                        printf("[RETURN-DEBUG] Fast recovery mode - minimal smoothing\n");
                    } else {
                        // ON-LINE: Very minimal smoothing for maximum responsiveness
                        smoothing_factor = 0.9f;  // 90% new, 10% previous - nearly instant response
                    }
                    
                    float pre_smooth = motor_correction;
                    motor_correction = (smoothing_factor * motor_correction) + ((1.0f - smoothing_factor) * previous_motor_correction);
                    
                    if (raw_adc < 1500) {
                        printf("[RETURN-DEBUG] Fast smoothing: %.3f -> %.3f\n", pre_smooth, motor_correction);
                    }
                } else {
                    // DEBUG: Show when corrections are blocked
                    if (raw_adc < 1500) {
                        printf("[RETURN-DEBUG] CORRECTIONS BLOCKED! allow_corrections=false\n");
                    }
                }
                
                previous_motor_correction = motor_correction;  // Remember for next iteration
                
                // Apply differential steering to current motor speed
                line_correction_L = current_motor_speed - motor_correction; // Left motor
                line_correction_R = current_motor_speed + motor_correction; // Right motor
                
                // MOTOR BALANCE CORRECTION: Left motor is stronger, reduce its power
                line_correction_L = line_correction_L * 0.90f;  // Reduce left motor by only 10% (was 15%)
                
                // DEBUG: Show final motor values when off-line
                if (raw_adc < 1500) {
                    printf("[RETURN-DEBUG] Final motors: L=%.3f R=%.3f (base_speed=%.3f correction=%.3f)\n",
                           line_correction_L, line_correction_R, current_motor_speed, motor_correction);
                }
                
                // Debug output showing bang-bang zones and motor response
                if ((now_ms - last_line_correction) > 400) {
                    const char* zone;
                    if (raw_adc >= 3200) zone = "CENTERED";
                    else if (raw_adc >= 2500) zone = "GOOD    ";
                    else if (raw_adc >= 2000) zone = "EDGE    ";
                    else if (raw_adc >= 1500) zone = "NEAR-OFF";
                    else zone = "RETURN-LINE";  // Changed to show active return mode
                    
                    // Enhanced status showing correction timing and overrides
                    const char* status;
                    if (allow_corrections) {
                        if (needs_emergency_correction) {
                            status = "EMERGENCY";
                        } else if (in_critical_period) {
                            status = "CRITICAL";
                        } else {
                            status = "ACTIVE";
                        }
                    } else {
                        status = "BLOCKED";  // Should rarely happen with new logic
                    }
                    
                    // Check if edge detection was triggered (use updated 200 threshold)
                    float debug_adc_deviation = (float)raw_adc - 3200.0f;  // optimal_adc value
                    float debug_distance = fabs(debug_adc_deviation);
                    bool debug_near_edge = (debug_distance > 200.0f);  // Updated threshold
                    
                    const char* edge_status;
                    if (debug_near_edge) {
                        float debug_edge_factor = (debug_distance - 200.0f) / 300.0f;  // Updated range
                        debug_edge_factor = fminf(debug_edge_factor, 1.0f);
                        if (debug_edge_factor < 0.3f) edge_status = "*EDGE-LOW*";
                        else if (debug_edge_factor < 0.7f) edge_status = "*EDGE-MID*"; 
                        else edge_status = "*EDGE-HIGH*";
                    } else {
                        edge_status = zone;
                    }
                    
                    printf("[MOTOR-CTRL] %s %s T:%dms | ADC:%d Err:%.3f -> L=%.3f R=%.3f\n",
                           status, edge_status, time_since_acceleration, raw_adc, normalized_error, line_correction_L, line_correction_R);
                    last_line_correction = now_ms;
                }
                
                last_line_update = now_ms;
            } else {
                // Not time for line update yet, use current motor speed
                line_correction_L = current_motor_speed;
                line_correction_R = current_motor_speed;
            }
        } else {
            // No line or stopped - no correction
            line_correction_L = current_motor_speed;
            line_correction_R = current_motor_speed;
        }
        
        // ========== COOPERATIVE IMU + LINE CORRECTION ==========
        
        // Start with line-corrected motor outputs
        motor_output_L = line_correction_L;
        motor_output_R = line_correction_R;
        
        // Apply motor control based on smooth ramping and line correction
        if (current_motor_speed > 0.0f) {
            // Clamp outputs to safe range (allowing lower minimums for smooth ramping)
            motor_output_L = fmaxf(0.0f, fminf(0.5f, motor_output_L));
            motor_output_R = fmaxf(0.0f, fminf(0.5f, motor_output_R));
            
            // IMU PROVIDES HEADING INFORMATION FOR MAIN CORRECTION SYSTEM
            // The main system decides corrections, IMU just provides heading context
            corrected_motor_L = motor_output_L;
            corrected_motor_R = motor_output_R;
            
            // Apply motor calibration to compensate for hardware differences
            float calibrated_motor_L = corrected_motor_L * MOTOR_CALIBRATION_LEFT;
            float calibrated_motor_R = corrected_motor_R * MOTOR_CALIBRATION_RIGHT;
            
            // DIAGNOSTIC: Check if motors are actually being commanded
            static uint32_t last_motor_debug = 0;
            if ((now_ms - last_motor_debug) > 1000) {  // Every 1 second
                printf("[MOTOR DEBUG] Sending PWM: L=%.3f R=%.3f | Expected movement: %s\n", 
                       calibrated_motor_L, calibrated_motor_R,
                       (calibrated_motor_L > 0.05f || calibrated_motor_R > 0.05f) ? "YES" : "NO");
                last_motor_debug = now_ms;
            }
            
            // Apply PWM to robot movement function
            robot_movement(calibrated_motor_L, calibrated_motor_R);
            
        } else {
            // Speed is 0 - stop motors smoothly
            robot_movement(0.0f, 0.0f);
            corrected_motor_L = 0.0f;
            corrected_motor_R = 0.0f;
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
        
        // Small delay to prevent overwhelming the system
        sleep_ms(10);
    }
    
    return 0;
}

