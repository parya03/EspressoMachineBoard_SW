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
#include "Matrix.h"

#define PID_TIME_MS 500.0f // 5000.0f // 500.0f // Make sure this is float
#define SSR_TIME_MS 500.0f // Cycle time of SSR
#define MCP_N 50 // How many samples ahead the MCP loop should use - keep >20 (to compensate for boiler delay)

// MPC tunables
#define ENERGY_PER_HALF_PHASE 6.482f // 777.8W RMS = 777.8 J/S -> 777.8J / 6.481666667 J per half phase
#define MODEL_J_PER_C 312.652f // Found empirically by dividing energy put in my temperature rise

extern float setpoint; // Degrees C
extern float curr_temp; // Degrees C

void control_init();
void temp_control_task(void *pvParameters);

#endif