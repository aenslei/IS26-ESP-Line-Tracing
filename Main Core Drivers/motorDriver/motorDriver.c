/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"


#define PWN_FREQ 10000

#define PWN_M1A 8 
#define PWN_M1B 9 
#define PWN_M2A 10 
#define PWN_M2B 11

void setup_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, 65535);
    pwm_set_clkdiv(slice_num, 125.0f); 
    pwm_set_enabled(slice_num, true);
}

//left and right pwm level 

//currently this is only for 2 motor 
void robot_movement(float sL, float sR)
{
    if(sL >= 0){
        pwm_set_gpio_level(PWN_M1A, sL * 65535); //1
        pwm_set_gpio_level(PWN_M1B, 0);          //0
   } else{
        pwm_set_gpio_level(PWN_M1A, 0);
        pwm_set_gpio_level(PWN_M1B, -sL * 65535);
	}
	
    if(sR >= 0){
        pwm_set_gpio_level(PWN_M2A, sR * 65535);
        pwm_set_gpio_level(PWN_M2B, 0);
   } else{
        pwm_set_gpio_level(PWN_M2A, 0);
        pwm_set_gpio_level(PWN_M2B, -sR * 65535);
	}

}

//if can add an button control 

int main(){
    float speed_fwd = 0.0f;
    float speed_bwd = 5.0f;
    int i = 0;
    int j = 0; 
    //int mode = 0;
    stdio_init_all();
    
    setup_pwm(PWN_M1A);
    setup_pwm(PWN_M1B);
    setup_pwm(PWN_M2A);
    setup_pwm(PWN_M2B);


    //
    while (true) {
        
        robot_movement(0,0);
        printf("\nRobot stops");
        sleep_ms(5000);


        speed_fwd = 0;
        for(i=0; i<5; i++) //increase speed 
        {
            printf("\nStart robot move forward");
            robot_movement(0.5+speed_fwd, 0.5+speed_fwd); 
            
            //seems like move forward +ve, backward is -ve
            speed_fwd+=0.1;
            sleep_ms(3000);

        }

        speed_bwd = 0.5;  // Start with a reasonable speed (0.0 to 1.0)
        for (j = 0; j < 3; j++) {
            printf("\nMove backward at speed: %.1f", speed_bwd);
            robot_movement(-speed_bwd, -speed_bwd);  // Negative for backward
            speed_bwd += 0.1;  // Increase speed    
            sleep_ms(5000);
        }

    }



}



