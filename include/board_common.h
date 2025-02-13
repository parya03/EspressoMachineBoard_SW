/**
 * Common stuff
 * 
 * Pin definitions, etc
 */

#ifndef COMMON_H
#define COMMON_H

#include "driver/gpio.h"

// Picked to be able to bypass IO matrix
#define IO_LCD_DC GPIO_NUM_9
#define IO_LCD_CS0 GPIO_NUM_10
#define IO_LCD_MOSI GPIO_NUM_11
#define IO_LCD_CLK GPIO_NUM_12
#define IO_LCD_MISO GPIO_NUM_13
#define IO_LCD_RESET GPIO_NUM_14

#endif