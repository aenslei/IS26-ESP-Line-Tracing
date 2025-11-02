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
float target_speed_mm_per_s = 200.0f;  // Target speed 

// Global variables
float integral_L = 0.0f, integral_R = 0.0f;
float last_error_L = 0.0f, last_error_R = 0.0f;
absolute_time_t pid_last_time_L, pid_last_time_R;
float last_speed_L = 0.0f, last_speed_R = 0.0f;
float base_speed = 0.4f; //pwm level base on target speed 

//PID constant for moving straight 
float Kp_L = 0.005f, Ki_L = 0.004f, Kd_L = 0.0003f;
float Kp_R = 0.008f, Ki_R = 0.004f, Kd_R = 0.0003f;

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


// Dictating robot motor movement using A and B pins
// Both Left and Right Motor movement
void robot_movement(float sL, float sR)
{
    //clamp power to 0 to 1
    sL = fmaxf(0.0f, fminf(1.0f, sL));
    sR = fmaxf(0.0f, fminf(1.0f, sR));

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

float pid_controller(float target, float current, float *integral, float *last_error,
                    absolute_time_t *last_time, float Kp, float Ki, float Kd) {
    absolute_time_t now = get_absolute_time();
    uint32_t dt_us = absolute_time_diff_us(*last_time, now);
    float dt = dt_us / 1000000.0f;

    float error = target - current;

    // More aggressive deadband
    if (fabs(error) < 10.0f) {
        error = 0.0f;
        *integral = 0.0f;
    }

    // Better anti-windup with lower threshold
    if (fabs(error) > 50.0f) {
        *integral = 0.0f;
    } else {
        *integral += error * dt;
        if (*integral > 30.0f) *integral = 30.0f;
        if (*integral < -30.0f) *integral = -30.0f;
    }

    //Low-pass filtering 
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - *last_error) / dt;
        derivative = 0.9f * derivative + 0.1f * ((error - *last_error) / dt);
    }

    *last_error = error;
    *last_time = now;

    float output = Kp * error + Ki * *integral + Kd * derivative;

    printf("P: %.2f, I: %.2f, D: %.2f, Output: %.2f, Error: %.2f\n",
           Kp * error, Ki * *integral, Kd * derivative, output, error);

    if (output > 1.0f) output = 1.0f;
    if (output < 0.0f) output = 0.0f;

    return output;
}

void update_wheel_speeds() {
    absolute_time_t now = get_absolute_time();
    float dt = absolute_time_diff_us(last_speed_update_time, now) / 1e6f;
    if (dt <= 0.01f) return;

    // Only calculate speed if we have pulses
    float raw_L = (pulse_count_L > 0) ? (pulse_count_L * DIST_PER_PULSE_MM) / dt : last_speed_L_mm_per_s;
    float raw_R = (pulse_count_R > 0) ? (pulse_count_R * DIST_PER_PULSE_MM) / dt : last_speed_R_mm_per_s;

    // Strong low-pass filtering
    last_speed_L_mm_per_s = 0.7f * last_speed_L_mm_per_s + 0.3f * raw_L;
    last_speed_R_mm_per_s = 0.7f * last_speed_R_mm_per_s + 0.3f * raw_R;

    // Validate speeds
    if (fabs(raw_L - last_speed_L_mm_per_s) > 100.0f) last_speed_L_mm_per_s = last_speed_L_mm_per_s;
    if (fabs(raw_R - last_speed_R_mm_per_s) > 100.0f) last_speed_R_mm_per_s = last_speed_R_mm_per_s;

    printf("Pulse Count L: %lu, R: %lu, Speed L: %.2f mm/s, Speed R: %.2f mm/s\n",
           pulse_count_L, pulse_count_R, last_speed_L_mm_per_s, last_speed_R_mm_per_s);

    //Reset count
    pulse_count_L = 0;
    pulse_count_R = 0;
    last_speed_update_time = now;
}

int main(){
    stdio_init_all();
    setup_pwm_pair(PWN_M1A, PWN_M1B);
    setup_pwm_pair(PWN_M2A, PWN_M2B);

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
    absolute_time_t last_speed_update = get_absolute_time();

    robot_movement(0.0f, 0.0f); //initial pwm values to zero 
    
    while (true) {
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(last_speed_update, now) >= 50000) { // update every 50ms
            update_wheel_speeds();
            last_speed_update = now;
        }

        // Calculate PID outputs - use same gains for moving straight 
        float pid_output_L = pid_controller(target_speed_mm_per_s, last_speed_L_mm_per_s,
                                            &integral_L, &last_error_L, &pid_last_time_L,
                                            Kp_L, Ki_L, Kd_L);
        float pid_output_R = pid_controller(target_speed_mm_per_s, last_speed_R_mm_per_s,
                                            &integral_R, &last_error_R, &pid_last_time_R,
                                            Kp_R, Ki_R, Kd_R);

        
        //ensures that motor still moves after obtaining target speed 
        float motor_output_L = base_speed + pid_output_L;
        float motor_output_R = base_speed + pid_output_R;

        //clamp motor output to be with 0 to 1.0 - this still overshoots the value of 1 
        motor_output_L = fmaxf(0.0f, fminf(motor_output_L, 1.0f));
        motor_output_R = fmaxf(0.0f, fminf(motor_output_R, 1.0f));
    
        // Apply the PWM
        robot_movement(motor_output_L,  motor_output_R);

        sleep_ms(50);  // Faster loop for better control - apply pwm accordingly every 50ms
    }
}
