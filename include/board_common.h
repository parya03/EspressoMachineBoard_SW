/**
 * Common stuff
 * 
 * Pin definitions, etc
 */

#ifndef COMMON_H
#define COMMON_H

#include "driver/gpio.h"

// LCD pins
// Picked to be able to bypass IO matrix
#define IO_LCD_DC GPIO_NUM_9
#define IO_LCD_CS0 GPIO_NUM_10
#define IO_LCD_MOSI GPIO_NUM_11
#define IO_LCD_CLK GPIO_NUM_12
#define IO_LCD_MISO GPIO_NUM_13
#define IO_LCD_RESET GPIO_NUM_14
#define IO_LCD_LED GPIO_NUM_21

// Rotary encoder
#define IO_ENC_A GPIO_NUM_43
#define IO_ENC_B GPIO_NUM_44
#define IO_ENC_BUTTON GPIO_NUM_17

// LEDs
#define IO_LED_RED GPIO_NUM_46
#define IO_LED_GREEN GPIO_NUM_47
#define IO_LED_BLUE GPIO_NUM_48

// Pump and boiler SSR
#define IO_PUMP GPIO_NUM_6
#define IO_BOILER GPIO_NUM_7

// Thermistor measurement
#define IO_THERMISTOR GPIO_NUM_2

#endif