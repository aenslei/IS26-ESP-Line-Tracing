#ifndef ROBO_DRIVER_V2_H
#define ROBO_DRIVER_V2_H

// Include both subsystem headers
#include "../IMU-Driver-V1/IMUDriver.h"
#include "WithPID.h"

// === Combined Robot System Configuration ===

// // Robot physical parameters
// #define ROBOT_WHEELBASE_MM 150.0f    // Distance between wheels (mm)
// #define ROBOT_MAX_SPEED_MM_S 500.0f  // Maximum safe speed (mm/s)
// #define ROBOT_MAX_TURN_RATE 180.0f   // Maximum turn rate (degrees/s)

// Control loop timing
#define CONTROL_LOOP_MS 50          // Main control loop period (ms)
#define IMU_UPDATE_MS 50           // IMU reading period (ms)  
#define PID_UPDATE_MS 100            // PID controller period (ms)

// === Combined Data Structures ===

typedef struct {
    // IMU data
    float accel_g[3];           // Accelerometer [x,y,z] in g-force
    heading_3d_t headings;      // All 3D compass headings
    int compass_quadrant;       // Current compass quadrant (1-4)
    float tilt_roll;           // Roll angle (degrees) -  XY axis
    float tilt_pitch;          // Pitch angle (degrees) - XZ axis
    
    // Motion data  
    float left_speed;          // Left wheel speed (mm/s): motor_output_L
    float right_speed;         // Right wheel speed (mm/s) motor_output_R
    // float robot_speed;         // Average robot speed (mm/s)
    
    // Calculated navigation
    float heading_current;     // Current heading reading, every 50ms 
    float heading_error;       // Change in compass heading since last reading
    float distance_traveled;   // Total distance traveled (mm)
    
    // Status flags
    bool imu_valid;           // True if IMU data is valid
    bool encoders_valid;      // True if encoder data is valid
} robot_state_t;

// Navigation control structure
typedef struct {
    float target_heading;      // Target compass heading (degrees): SET AS BASE HEADING
    float target_speed;        // Target forward speed (mm/s)s: FROM PID
    float heading_tolerance;   // Acceptable heading error (degrees)
    float speed_tolerance;     // Acceptable speed error (mm/s)

    //IMU G Values for angle control
    // float heading_current;    
    // float heading_error;     // Change in compass heading since last readin
    
    // Navigation PID for heading and angle control: Need?
    float heading_kp;         // Proportional gain for heading
    float heading_ki;         // Integral gain for heading
    float heading_kd;         // Derivative gain for heading
} navigation_params_t;

// === Combined System Functions ===

/**
 * @brief Initialize complete robot system (IMU + motors + encoders)
 * @return true if all subsystems initialized successfully
 */
bool robot_full_system_init(void);

/**
 * @brief Check if robot is level (not tilted)
 * @param max_tilt_degrees Maximum acceptable tilt angle
 * @return true if robot is within tilt limits
 */
bool is_robot_level(float max_tilt_degrees);


/**
 * @brief Update all robot sensors and state
 * @param state Pointer to robot state structure to update
 * @return true if update successful
 */
bool update_robot_state(robot_state_t *state);

/**
 * @brief Navigate robot to target heading at target speed
 * @param params Navigation parameters
 * @param current_state Current robot state
 * @return true if navigation command executed successfully
 */
bool navigate_to_heading(const navigation_params_t *params, const robot_state_t *current_state);

/**
 * @brief Execute a turn by specified angle
 * @param turn_angle_degrees Angle to turn (positive = right, negative = left)
 * @param turn_speed_mm_s Speed during turn
 * @return true when turn is complete
 */
bool execute_turn(float turn_angle_degrees, float turn_speed_mm_s);

/**
 * @brief Move straight for specified distance
 * @param distance_mm Distance to travel (positive = forward, negative = backward)
 * @param speed_mm_s Speed during movement
 * @return true when movement is complete
 */
bool move_distance(float distance_mm, float speed_mm_s);

/**
 * @brief Emergency stop - immediately halt all movement
 */
void emergency_stop(void);

/**
 * @brief Get current robot heading (compass)
 * @return Heading in degrees (0-360), or -1 if invalid
 */
float get_current_heading(void);

/**
 * @brief Calculate turn angle needed to reach target heading
 * @param current_heading Current compass heading (degrees)
 * @param target_heading Target compass heading (degrees)  
 * @return Turn angle needed (-180 to +180 degrees)
 */
float calculate_turn_angle(float current_heading, float target_heading);

/**
 * @brief Print comprehensive robot status for debugging
 * @param state Current robot state
 */
void print_robot_status(const robot_state_t *state);

// === Line Following Integration Functions ===

/**
 * @brief Enhanced line following with IMU feedback
 * @param line_sensor_error Error from line sensor (-1.0 to 1.0)
 * @param base_speed_mm_s Base forward speed
 * @param turn_sensitivity Sensitivity to line sensor error (0.1 to 1.0)
 * @return true if command executed successfully
 */
bool line_follow_with_imu(float line_sensor_error, float base_speed_mm_s, float turn_sensitivity);

/**
 * @brief Detect and measure turns during line following
 * @param heading_threshold Minimum heading change to detect turn (degrees)
 * @return Turn angle detected (degrees), or 0 if no turn
 */
float detect_line_turn(float heading_threshold);

// === Calibration and Setup Functions ===

/**
 * @brief Calibrate IMU compass (rotate robot 360 degrees)
 * @return true if calibration successful
 */
bool calibrate_compass(void);

/**
 * @brief Auto-tune PID parameters by test movements
 * @return true if tuning successful
 */
bool auto_tune_pid(void);

/**
 * @brief Save calibration data to flash memory
 * @return true if save successful
 */
bool save_calibration_data(void);

/**
 * @brief Load calibration data from flash memory  
 * @return true if load successful
 */
bool load_calibration_data(void);

#endif // ROBO_DRIVER_V2_H
