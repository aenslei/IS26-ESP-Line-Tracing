#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include <math.h>


//--------Wheel Encoder Variables--------
#define GPIO_ENCODER_PIN_L 2
#define GPIO_ENCODER_PIN_R 4
#define WHEEL_CIRCUMFERENCE_MM 210.0f   // Example wheel circumference (mm)
#define ENCODER_NOTCHES 26              // Number of notches (pulses) per revolution

#define GPIO_LED_PIN 3   // Onboard LED on Raspberry Pi Pico

// Distance per notch
#define DIST_PER_PULSE_MM (WHEEL_CIRCUMFERENCE_MM / ENCODER_NOTCHES)

// Debounce settings
#define DEBOUNCE_TIME_US 800

// Left Wheel Encoder data
static volatile uint32_t pulse_count_L = 0;       // total pulses detected (left)
static volatile absolute_time_t last_pulse_L;     // time of last pulse (left)
static volatile float last_speed_L_mm_per_s = 0;  // last computed speed (left)

// Right Wheel Encoder data  
static volatile uint32_t pulse_count_R = 0;       // total pulses detected (right)
static volatile absolute_time_t last_pulse_R;     // time of last pulse (right)
static volatile float last_speed_R_mm_per_s = 0;  // last computed speed (right)

static volatile absolute_time_t last_speed_update_time;
//--------Motor Driver--------

#define PWN_FREQ 10000

#define PWN_M1A 8 //R
#define PWN_M1B 9 //R
#define PWN_M2A 10 //L
#define PWN_M2B 11 //L


//--------PID controller--------

// PID variables
float target_speed_mm_per_s = 300.0f;  // Target speed 200mm/s
float Kp = 0.01f, Ki = 0.0005f, Kd = 0.0005f;
float integral_L = 0.0f, integral_R = 0.0f;
float last_error_L = 0.0f, last_error_R = 0.0f;

//---------------FUNCTIONS---------------

//--------Motor Driver Functions--------
// Setting up PWM Levels

void setup_pwm_pair(uint gpioA, uint gpioB) {
    gpio_set_function(gpioA, GPIO_FUNC_PWM);
    gpio_set_function(gpioB, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(gpioA);
    pwm_set_wrap(slice, 65535);
    pwm_set_clkdiv(slice, 125.0f);
    pwm_set_enabled(slice, true);
}

void setup_pwm_led() {
    // Set the LED GPIO to PWM function
    gpio_set_function(GPIO_LED_PIN, GPIO_FUNC_PWM);

    // Get the PWM slice number for the GPIO
    uint slice_num = pwm_gpio_to_slice_num(GPIO_LED_PIN);

    // Set the PWM wrap value (max count)
    pwm_set_wrap(slice_num, 65535);

    // Set the clock divisor (adjust as needed)
    pwm_set_clkdiv(slice_num, 125.0f);

    // Enable the PWM slice
    pwm_set_enabled(slice_num, true);
}

// Dictating robot motor movement using A and B pins
// Both Left and Right Motor movement
void robot_movement(float sL, float sR)
{
    uint16_t pwm_level_L = (uint16_t)(fabs(sL) * 65535);
    uint16_t pwm_level_R = (uint16_t)(fabs(sR) * 65535);
	
    if(sR >= 0) {
        // Right motor FORWARD (now swapped)
        pwm_set_gpio_level(PWN_M1A, 0);
        pwm_set_gpio_level(PWN_M1B, pwm_level_R);
    } else {
        // Right motor REVERSE
        pwm_set_gpio_level(PWN_M1A, pwm_level_R);
        pwm_set_gpio_level(PWN_M1B, 0);
    }

    if(sL >= 0){ //if over 1, movement forward. A Pin
        pwm_set_gpio_level(PWN_M2A, pwm_level_L); //1
        pwm_set_gpio_level(PWN_M2B, 0);          //0
    } else{ //if negative, movement backward. B Pin
        pwm_set_gpio_level(PWN_M2A, 0);
        pwm_set_gpio_level(PWN_M2B, pwm_level_L);
	}

    // Update PWM on GPIO pin 3 based on PID output (e.g., left motor PID output)
    uint16_t led_duty = (uint16_t)(fabs(sL) * 65535);  // Use absolute value of PID output
    pwm_set_gpio_level(GPIO_LED_PIN, led_duty);

}

//--------Wheel Encoder Driver Functions--------
// Callback for GPIO Interrupts 
void gpio_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        // Get current time
        absolute_time_t now = get_absolute_time();

        // GPIO Callback for Left Encoder
        if (gpio == GPIO_ENCODER_PIN_L) {
            // Debounce: ignore pulses too close together
            if (!is_nil_time(last_pulse_L)) {
                int64_t dt_us = absolute_time_diff_us(last_pulse_L, now);
                if (dt_us < DEBOUNCE_TIME_US) {
                    return; // Ignore this pulse - too soon after last one
                }
            }
            
            pulse_count_L++;  // Count this valid pulse
            last_pulse_L = now;
            
            //GPIO Callback for Right Encoder
        } else if (gpio == GPIO_ENCODER_PIN_R) {
            // Debounce: ignore pulses too close together
            if (!is_nil_time(last_pulse_R)) {
                int64_t dt_us = absolute_time_diff_us(last_pulse_R, now);
                if (dt_us < DEBOUNCE_TIME_US) {
                    return; // Ignore this pulse - too soon after last one
                }
            }
            
            pulse_count_R++;  // Count this valid pulse
            last_pulse_R = now;
        }
    
    }
}

/*
void wheelEncoder(){
    // Calculate distances for both wheels
        float distance_L_mm = pulse_count_L * DIST_PER_PULSE_MM;
        float distance_R_mm = pulse_count_R * DIST_PER_PULSE_MM;
        
        printf("Encoder values -> Left: %lu pulses, %.2f mm, %.2f mm/s | Right: %lu pulses, %.2f mm, %.2f mm/s\n",
            pulse_count_L, distance_L_mm, last_speed_L_mm_per_s,
            pulse_count_R, distance_R_mm, last_speed_R_mm_per_s);
}
*/

//--------PID function----------

// PID controller function
float pid_controller(float target, float current, float *integral, float *last_error, absolute_time_t *last_time) {
    absolute_time_t now = get_absolute_time();  // Current absolute time
    uint32_t dt_us = absolute_time_diff_us(*last_time, now);  // Time elapsed in microseconds
    float dt = dt_us / 1000000.0f;  // Convert to seconds

    float error = target - current;
    /*
    *integral += error * dt;

    if (*integral > 1000.0f) *integral = 1000.0f;
    if (*integral < -1000.0f) *integral = -1000.0f;
    */
       // --- Deadband ---
    if (fabs(error) < 2.0f) error = 0.0f;

    // --- Anti-windup ---
    if (fabs(error) > 120.0f)
        *integral = 0.0f;
    else
        *integral += error * dt;

    // Clamp integral
    if (*integral > 800.0f) *integral = 800.0f;
    if (*integral < -800.0f) *integral = -800.0f;

    float derivative = (error - *last_error) / dt;
    derivative = 0.7f * derivative + 0.3f * ((error - *last_error) / dt);

    *last_error = error;
    *last_time = now;


    float output = Kp * error + Ki * *integral + Kd * derivative;
    
    // Clamp to 0–1 for PWM
    if (output > 1.0f) output = 1.0f;
    if (output < 0.0f) output = 0.0f;

    printf("P: %.2f, I: %.2f, D: %.2f, Output: %.2f, Error: %.2f\n",
           Kp * error, Ki * *integral, Kd * derivative, output, error);
    return output;
}

// Function to calculate wheel speeds
void update_wheel_speeds() {
    absolute_time_t now = get_absolute_time();
    float dt = absolute_time_diff_us(last_speed_update_time, now) / 1e6f;
    if (dt <= 0.01f) return;

    float raw_L = (pulse_count_L * DIST_PER_PULSE_MM) / dt;
    float raw_R = (pulse_count_R * DIST_PER_PULSE_MM) / dt;

    // Low-pass filter
    last_speed_L_mm_per_s = 0.7f * last_speed_L_mm_per_s + 0.3f * raw_L;
    last_speed_R_mm_per_s = 0.7f * last_speed_R_mm_per_s + 0.3f * raw_R;

    /*
    uint32_t dt_us = absolute_time_diff_us(last_speed_update_time, now);
    float dt = dt_us / 1000000.0f; // Convert to seconds

    if (dt > 0) {
        last_speed_L_mm_per_s = (pulse_count_L * DIST_PER_PULSE_MM) / dt;
        last_speed_R_mm_per_s = (pulse_count_R * DIST_PER_PULSE_MM) / dt;
    } else {
        last_speed_L_mm_per_s = 0;
        last_speed_R_mm_per_s = 0;
    }
    */

    printf("Pulse Count L: %lu, R: %lu, Speed L: %.2f mm/s, Speed R: %.2f mm/s\n",
           pulse_count_L, pulse_count_R, last_speed_L_mm_per_s, last_speed_R_mm_per_s);
    
    //reset pulses 
    pulse_count_L = 0;
    pulse_count_R = 0;
    last_speed_update_time = now;
}


int main(){

    stdio_init_all();
    setup_pwm_pair(PWN_M1A, PWN_M1B);
    setup_pwm_pair(PWN_M2A, PWN_M2B);
    setup_pwm_led();

    //------------WHEEL ENCODER INIT------------
    // Init left encoder GPIO pin
    gpio_init(GPIO_ENCODER_PIN_L);
    gpio_set_dir(GPIO_ENCODER_PIN_L, false); // input
    gpio_pull_up(GPIO_ENCODER_PIN_L);        // pull-up if open collector encoder

    // Init right encoder GPIO pin
    gpio_init(GPIO_ENCODER_PIN_R);
    gpio_set_dir(GPIO_ENCODER_PIN_R, false); // input
    gpio_pull_up(GPIO_ENCODER_PIN_R);        // pull-up if open collector encoder

    // Enable IRQ on rising edge for both encoders
    gpio_set_irq_enabled_with_callback(GPIO_ENCODER_PIN_L, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(GPIO_ENCODER_PIN_R, GPIO_IRQ_EDGE_RISE, true, &gpio_callback); 

    // Initialize last pulse times
    last_pulse_L = nil_time;
    last_pulse_R = nil_time;

    // Variables for non-blocking movement test timing
    //absolute_time_t last_movement_time = get_absolute_time();
    absolute_time_t last_encoder_print_time = get_absolute_time();
    absolute_time_t last_speed_update = get_absolute_time();
    absolute_time_t pid_last_time_L = get_absolute_time();
    absolute_time_t pid_last_time_R = get_absolute_time();

    //------------MAIN------------
    printf("Robot Driver Started - Running movement test with wheel encoder monitoring\n");
    
/*
    int movement_stage = 0;  // Track which part of movement test we're in
    int forward_step = 0;    // Track forward speed steps (0-4)
    int backward_step = 0;   // Track backward speed steps (0-2)
    
    while(true) {
        absolute_time_t now = get_absolute_time();

        // Check if it's time for next movement command (non-blocking)
        if (absolute_time_diff_us(last_movement_time, now) >= 3000000) { // 3 seconds
            
            switch(movement_stage) {
                case 0: // Stop phase
                    robot_movement(0, 0);
                    printf("\nRobot stops\n");
                    movement_stage = 1;
                    last_movement_time = now;
                    break;
                    
                case 1: // Forward movement phase
                    if (forward_step < 5) {
                        float current_speed = 0.5f + (speed_fwd + forward_step * 0.1f);
                        robot_movement(current_speed, current_speed);
                        printf("\nMove forward at speed: %.1f\n", current_speed);
                        forward_step++;
                        last_movement_time = now;
                    } else {
                        movement_stage = 2;
                        forward_step = 0;  // Reset for next cycle
                    }
                    break;
                    
                case 2: // Backward movement phase
                    if (backward_step < 3) {
                        float current_speed = speed_bwd + backward_step * 0.1f;
                        robot_movement(-current_speed, -current_speed);
                        printf("\nMove backward at speed: %.1f\n", current_speed);
                        backward_step++;
                        last_movement_time = now;
                    } else {
                        movement_stage = 0;  // Reset to stop phase
                        backward_step = 0;   // Reset for next cycle
                    }
                    break;
            }
        }
*/
   
    robot_movement(0.0f, 0.0f); //initial pwm values to zero 
    sleep_ms(3000);
    while (true){

        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(last_speed_update, now) >= 100000) { // 100ms
            update_wheel_speeds();
            last_speed_update = now;
        }
        
        
        float pid_output_L = pid_controller(target_speed_mm_per_s, last_speed_L_mm_per_s, &integral_L, &last_error_L, &pid_last_time_L); 
        float pid_output_R = pid_controller(target_speed_mm_per_s, last_speed_R_mm_per_s, &integral_R, &last_error_R, &pid_last_time_R);

       //make sure pid value does not exceed 1.0 

        if (pid_output_R > 1.0f) pid_output_R = 1.0f;
        if (pid_output_R < 0.0f) pid_output_R = 0.0f;
        robot_movement(pid_output_L, pid_output_R); //apply the pwm

        printf("Target: %.1f, Left: %.1f, Right: %.1f, PID L: %.2f, PID R: %.2f\n",
            target_speed_mm_per_s, last_speed_L_mm_per_s, last_speed_R_mm_per_s, pid_output_L, pid_output_R);

        sleep_ms(100);  // PID update interval

    }


}


