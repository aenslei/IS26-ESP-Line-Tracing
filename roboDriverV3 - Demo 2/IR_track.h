#ifndef IR_TRACK_H
#define IR_TRACK_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#define LINE_ADC_PIN 28


void ir_track_init(void);
bool is_line_traced(void); 
uint16_t get_line_adc_value(void);  // For debugging - get raw ADC value

#endif