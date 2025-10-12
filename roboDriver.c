#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"


//--------Wheel Encoder Variables--------
#define GPIO_ENCODER_PIN_L 2
#define GPIO_ENCODER_PIN_R 4
#define WHEEL_CIRCUMFERENCE_MM 210.0f   // Example wheel circumference (mm)
#define ENCODER_NOTCHES 26              // Number of notches (pulses) per revolution

#define GPIO_LED_PIN 3   // Onboard LED on Raspberry Pi Pico

// Distance per notch
#define DIST_PER_PULSE_MM (WHEEL_CIRCUMFERENCE_MM / ENCODER_NOTCHES)

// Debounce settings
#define DEBOUNCE_TIME_US 1000  // 1ms minimum between pulses

// Left Wheel Encoder data
static volatile uint32_t pulse_count_L = 0;       // total pulses detected (left)
static volatile absolute_time_t last_pulse_L;     // time of last pulse (left)
static volatile float last_speed_L_mm_per_s = 0;  // last computed speed (left)

// Right Wheel Encoder data  
static volatile uint32_t pulse_count_R = 0;       // total pulses detected (right)
static volatile absolute_time_t last_pulse_R;     // time of last pulse (right)
static volatile float last_speed_R_mm_per_s = 0;  // last computed speed (right)

//--------Motor Driver--------

#define PWN_FREQ 10000

#define PWN_M1A 8 //R
#define PWN_M1B 9 //R
#define PWN_M2A 10 //L
#define PWN_M2B 11 //L

//---------------FUNCTIONS---------------

//--------Motor Driver Functions--------
// Setting up PWM Levels
void setup_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, 65535);
    pwm_set_clkdiv(slice_num, 125.0f); 
    pwm_set_enabled(slice_num, true);
}

// Dictating robot motor movement using A and B pins
// Both Left and Right Motor movement
void robot_movement(float sL, float sR)
{
    if(sL >= 0){ //if over 1, movement forward. A Pin
        pwm_set_gpio_level(PWN_M1A, sR * 65535); //1
        pwm_set_gpio_level(PWN_M1B, 0);          //0
   } else{ //if negative, movement backward. B Pin
        pwm_set_gpio_level(PWN_M1A, 0);
        pwm_set_gpio_level(PWN_M1B, -sR * 65535);
	}
	
    if(sR >= 0){
        pwm_set_gpio_level(PWN_M2A, sL * 65535);
        pwm_set_gpio_level(PWN_M2B, 0);
   } else{
        pwm_set_gpio_level(PWN_M2A, 0);
        pwm_set_gpio_level(PWN_M2B, -sL * 65535);
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
                // Calculate speed if valid pulse
                if (dt_us > 0) {
                    last_speed_L_mm_per_s = (DIST_PER_PULSE_MM / dt_us) * 1e6f;
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
                // Calculate speed if valid pulse
                if (dt_us > 0) {
                    last_speed_R_mm_per_s = (DIST_PER_PULSE_MM / dt_us) * 1e6f;
                }
            }
            
            pulse_count_R++;  // Count this valid pulse
            last_pulse_R = now;
        }

        // Toggle LED on any encoder pulse
        gpio_xor_mask(1u << GPIO_LED_PIN);
    }
}

void wheelEncoder(){
    // Calculate distances for both wheels
        float distance_L_mm = pulse_count_L * DIST_PER_PULSE_MM;
        float distance_R_mm = pulse_count_R * DIST_PER_PULSE_MM;
        
        // Calculate average distance (for robot forward movement)
        float avg_distance_mm = (distance_L_mm + distance_R_mm) / 2.0f;
        
        printf("Left: %lu pulses, %.2f mm, %.2f mm/s | Right: %lu pulses, %.2f mm, %.2f mm/s | Avg: %.2f mm\n",
            pulse_count_L, distance_L_mm, last_speed_L_mm_per_s,
            pulse_count_R, distance_R_mm, last_speed_R_mm_per_s,
            avg_distance_mm);
}


int main(){
    //------------MOTOR DRIVER INIT------------
    float speed_fwd = 0.0f;
    float speed_bwd = 5.0f;

    stdio_init_all();

    setup_pwm(PWN_M1A);
    setup_pwm(PWN_M1B);
    setup_pwm(PWN_M2A);
    setup_pwm(PWN_M2B);

    //------------WHEEL ENCODER INIT------------
    // Init left encoder GPIO pin
    gpio_init(GPIO_ENCODER_PIN_L);
    gpio_set_dir(GPIO_ENCODER_PIN_L, false); // input
    gpio_pull_up(GPIO_ENCODER_PIN_L);        // pull-up if open collector encoder

    // Init right encoder GPIO pin
    gpio_init(GPIO_ENCODER_PIN_R);
    gpio_set_dir(GPIO_ENCODER_PIN_R, false); // input
    gpio_pull_up(GPIO_ENCODER_PIN_R);        // pull-up if open collector encoder

    // Init LED output
    gpio_init(GPIO_LED_PIN);
    gpio_set_dir(GPIO_LED_PIN, true); // output
    gpio_put(GPIO_LED_PIN, 0);        // start OFF

    // Enable IRQ on rising edge for both encoders
    gpio_set_irq_enabled_with_callback(GPIO_ENCODER_PIN_L, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(GPIO_ENCODER_PIN_R, GPIO_IRQ_EDGE_RISE, true);

    // Initialize last pulse times
    last_pulse_L = nil_time;
    last_pulse_R = nil_time;

    //------------MAIN------------
    printf("Robot Driver Started - Running movement test with wheel encoder monitoring\n");
    
    // Variables for non-blocking movement test timing
    absolute_time_t last_movement_time = get_absolute_time();
    absolute_time_t last_encoder_print_time = get_absolute_time();
    
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
        
        // Print wheel encoder data every 200ms (non-blocking)
        if (absolute_time_diff_us(last_encoder_print_time, now) >= 200000) { // 200ms
            wheelEncoder();
            last_encoder_print_time = now;
        }
        
        // Small delay to prevent overwhelming the CPU
        sleep_ms(10);
    }

}

