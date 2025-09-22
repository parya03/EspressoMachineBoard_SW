/**
 * File for PID control of temp, brew control, etc
 * 
 * 
 */

#ifndef CONTROL_H
#define CONTROL_H

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "math.h"
#include "driver/ledc.h"

#include "board_common.h"

#include "Interface.h"

#define PID_TIME_MS 250.0f // Make sure this is float

extern float setpoint; // Degrees C
extern float curr_temp; // Degrees C

void control_init();
void temp_control_task(void *pvParameters);

#endif