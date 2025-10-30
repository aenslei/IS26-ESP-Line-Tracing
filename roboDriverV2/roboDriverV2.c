#include "roboDriver_V2.h"  // Includes everything
#include "../IMU-Driver-V1/IMUDriver.h"  // Direct IMU access
#include "WithPID.h"  // Direct PID access

// Function declarations from WithPID.c
float get_last_speed_L_mm_per_s(void);
float get_last_speed_R_mm_per_s(void);
void set_target_speed(float speed_mm_per_s);
void robot_movement(float left_motor_output, float right_motor_output);
void update_wheel_speeds(void);

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
    
    printf("✓ Robot system fully initialized!\n\n");
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
    printf("\nStarting main control loop...\n");
    printf("Goal: Stable, straight movement without jerkiness\n\n");
    
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
        
        // Simple motor control - use constant base speed with light speed feedback
        motor_output_L = 0.35f;  // Start with 35% base power
        motor_output_R = 0.35f;  // Start with 35% base power
        
        // Update wheel speed measurements (only every 300ms to allow even more pulse accumulation)
        if (now_ms - last_speed_update_ms >= 300) {
            update_wheel_speeds();
            last_speed_update_ms = now_ms;
        }
        
        // Light speed adjustment (much simpler than complex PID)
        float current_speed_L = get_last_speed_L_mm_per_s();
        float current_speed_R = get_last_speed_R_mm_per_s();
        float target_speed_mm_per_s = 150.0f;  // Target speed for simple control
        
        // Simple proportional adjustment (very light)
        if (current_speed_L < target_speed_mm_per_s * 0.8f && current_speed_R < target_speed_mm_per_s * 0.8f) {
            motor_output_L += 0.05f;  // Slight increase if too slow
            motor_output_R += 0.05f;  // Slight increase if too slow
        }
        // if (current_speed_R < target_speed_mm_per_s * 0.8f) {
        //     motor_output_R += 0.05f;  // Slight increase if too slow
        // }
        
        // Clamp outputs
        motor_output_L = fmaxf(0.2f, fminf(0.6f, motor_output_L));
        motor_output_R = fmaxf(0.2f, fminf(0.6f, motor_output_R));
        
        // Apply angle correction algorithm (only if IMU is working)
        if (imu_available && accel_ok && mag_ok) {
            calculate_angle_correction(error_headings, motor_output_L, motor_output_R,
                                     &corrected_motor_L, &corrected_motor_R);
        } else {
            // No IMU correction - use base motor outputs directly
            corrected_motor_L = motor_output_L;
            corrected_motor_R = motor_output_R;
        }
        
        // Apply PWM to robot movement function
        robot_movement(corrected_motor_L, corrected_motor_R);
        
        // Status reporting every 1 second
        if ((now_ms - last_status_print_ms) >= 1000) {
            if (imu_available) {
                printf("[%lu] IMU:%s Speeds: L=%4.0f R=%4.0f mm/s | Motors: L=%.2f R=%.2f | Heading: %.1f° (Error: %+.1f°)\n",
                       loop_counter / 20, // Approximate seconds
                       (accel_ok && mag_ok) ? "OK" : "ERR",
                       get_last_speed_L_mm_per_s(), get_last_speed_R_mm_per_s(),
                       corrected_motor_L, corrected_motor_R,
                       current_headings.xy_heading, error_headings);
            } else {
                printf("[%lu] IMU:DISABLED Speeds: L=%4.0f R=%4.0f mm/s | Motors: L=%.2f R=%.2f | Mode: Forward-only\n",
                       loop_counter / 20, // Approximate seconds
                    get_last_speed_L_mm_per_s(), get_last_speed_R_mm_per_s(),
                    corrected_motor_L, corrected_motor_R);
            }
            last_status_print_ms = now_ms;
        }
        
        loop_counter++;
        
        // Small delay to prevent overwhelming the system
        sleep_ms(10);
    }
    
    return 0;
}

