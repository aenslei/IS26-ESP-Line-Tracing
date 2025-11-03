#ifndef WITH_PID_H
#define WITH_PID_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include <math.h>

// === Wheel Encoder Configuration ===
#define GPIO_ENCODER_PIN_L 2
#define GPIO_ENCODER_PIN_R 4
#define WHEEL_CIRCUMFERENCE_MM 210.0f   // Wheel circumference (mm)
#define ENCODER_NOTCHES 26              // Number of notches (pulses) per revolution

// Distance per notch
#define DIST_PER_PULSE_MM (WHEEL_CIRCUMFERENCE_MM / ENCODER_NOTCHES)

// Debounce settings
#define DEBOUNCE_TIME_US 800

// === Motor Driver Configuration ===
#define PWN_FREQ 10000

#define PWN_M1A 8 // Right motor A
#define PWN_M1B 9 // Right motor B
#define PWN_M2A 10 // Left motor A
#define PWN_M2B 11 // Left motor B

// === PID Controller Configuration ===

// Target speeds
extern float target_speed_mm_per_s;  // Target speed in mm/s

// PID state variables
extern float integral_L, integral_R;
extern float last_error_L, last_error_R;
extern absolute_time_t pid_last_time_L, pid_last_time_R;
extern float last_speed_L, last_speed_R;
extern float base_speed; // PWM level base on target speed 

// PID constants for moving straight 
extern float Kp_L, Ki_L, Kd_L; // Left wheel PID constants
extern float Kp_R, Ki_R, Kd_R; // Right wheel PID constants

// Encoder data (read-only access via functions)
extern volatile uint32_t pulse_count_L;       // Total pulses detected (left)
extern volatile uint32_t pulse_count_R;       // Total pulses detected (right)
extern volatile float last_speed_L_mm_per_s;  // Last computed speed (left)
extern volatile float last_speed_R_mm_per_s;  // Last computed speed (right)

// Function Declarations

// === Motor Control Functions ===

/**
 * @brief Setup PWM for a motor pin pair
 * @param gpioA GPIO pin for motor direction A
 * @param gpioB GPIO pin for motor direction B
 */
void setup_pwm_pair(uint gpioA, uint gpioB);

/**
 * @brief Control robot motor movement
 * @param sL Left motor speed (-1.0 to 1.0, negative = reverse)
 * @param sR Right motor speed (-1.0 to 1.0, negative = reverse)
 */
void robot_movement(float sL, float sR);

// === Encoder Functions ===

/**
 * @brief GPIO interrupt callback for wheel encoders
 * @param gpio GPIO pin that triggered the interrupt
 * @param events Event flags (edge rise/fall)
 */
void gpio_callback(uint gpio, uint32_t events);

/**
 * @brief Update wheel speed calculations
 * Call this regularly to update speed measurements
 */
void update_wheel_speeds(void);

/**
 * @brief Get current left wheel speed
 * @return Speed in mm/s
 */
float get_left_wheel_speed(void);

/**
 * @brief Get current right wheel speed  
 * @return Speed in mm/s
 */
float get_right_wheel_speed(void);

/**
 * @brief Get total distance traveled by left wheel
 * @return Distance in mm
 */
float get_left_wheel_distance(void);

/**
 * @brief Get total distance traveled by right wheel
 * @return Distance in mm
 */
float get_right_wheel_distance(void);

/**
 * @brief Reset encoder pulse counts
 */
void reset_encoder_counts(void);

// === PID Controller Functions ===

/**
 * @brief PID controller implementation
 * @param target Target value
 * @param current Current measured value
 * @param integral Pointer to integral accumulator
 * @param last_error Pointer to last error value
 * @param last_time Pointer to last update time
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @return Control output (0.0 to 1.0)
 */
float pid_controller(float target, float current, float *integral, float *last_error,
                    absolute_time_t *last_time, float Kp, float Ki, float Kd);

// === High-Level Robot Control Functions ===

/**
 * @brief Initialize the complete motor system (motors, encoders, PID)
 * @return true if initialization successful, false otherwise
 */
bool motor_system_init(void);

/**
 * @brief Set target speed for both wheels
 * @param speed_mm_per_s Target speed in mm/s
 */
void set_target_speed(float speed_mm_per_s);

/**
 * @brief Set individual PID constants for left wheel
 * @param Kp Proportional gain
 * @param Ki Integral gain  
 * @param Kd Derivative gain
 */
void set_left_pid_constants(float Kp, float Ki, float Kd);

/**
 * @brief Set individual PID constants for right wheel
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 */
void set_right_pid_constants(float Kp, float Ki, float Kd);

/**
 * @brief Run PID control loop for both wheels
 * Call this regularly (e.g., every 50ms) to maintain target speeds
 */
void run_pid_control(void);

/**
 * @brief Move robot straight at target speed
 * @param speed_mm_per_s Target speed in mm/s
 */
void move_straight(float speed_mm_per_s);

/**
 * @brief Turn robot by differential wheel speeds
 * @param left_speed_mm_per_s Left wheel target speed
 * @param right_speed_mm_per_s Right wheel target speed
 */
void differential_turn(float left_speed_mm_per_s, float right_speed_mm_per_s);

/**
 * @brief Stop robot immediately
 */
void stop_robot(void);

/**
 * @brief Get robot's average speed
 * @return Average speed of both wheels in mm/s
 */
float get_robot_speed(void);

/**
 * @brief Get robot's turning rate (speed difference)
 * @return Right wheel speed - left wheel speed (mm/s)
 */
float get_robot_turn_rate(void);

void emergency_stop(void);


#endif // WITH_PID_H
