/**
 * File for I/O with outside world
 * 
 * Encoder, etc
 */

#ifndef IO_H
#define IO_H

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#include "esp_attr.h"
#include "esp_log.h"

#include "board_common.h"

// Initialize rotary encoder
esp_err_t encoder_init();

// Get count since last call
int encoder_get_count();

// Get button status
// ISR sets variable state on any edge
bool encoder_get_button_state();

// State stays until read
// Useful for LVGL's intermittent refreshes
bool encoder_get_button_state_sticky();

#endif