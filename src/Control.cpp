/**
 * File for PID control of temp, brew control, etc
 * 
 * 
 */

#include "Control.h"

adc_oneshot_unit_handle_t adc1_handle = NULL;
adc_cali_handle_t adc1_cali_handle = NULL;
// int u = 0;

// Boiler IO
gpio_config_t boiler_io_conf = {
    .pin_bit_mask = ((1ULL << IO_BOILER)),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = (gpio_pullup_t)GPIO_PULLUP_DISABLE,
    .pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};

float setpoint = 27.0f; // Degrees C
float curr_temp = 100.0f; // Start high by default for safety

void control_init() {
    esp_err_t ret;
    
    // PWM config for boiler
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 2,  // Set output frequency at 1Hz so we only control duty cycle
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = IO_BOILER,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    
    // ADC1 Init
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ADC1 Config
    adc_oneshot_chan_cfg_t adc1_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_1, &adc1_config));

    // ADC1 Calibration
    ESP_LOGI("", "Calibrating ADC1 Channel 1 (thermistor)...");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);
    // if (ret == ESP_OK) {
    //     calibrated = true;
    // }
}

void temp_control_task(void *pvParameters) {
    float therm_raw_avg = 0;
    int therm_mV = 0;
    float temp_prev = 0.0f;

    // PID stuff
    float int_e = 0; // Integral
    float prev_e = 0; // Used for derivative
    float d_e = 0; // Derivative

    // Measure thermistor and output on serial
    while(1) {
        for(int i = 0; i < 100; i++) {
            int therm_raw = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_1, &therm_raw));

            therm_raw_avg += therm_raw;
            // ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, therm_raw, &therm_mV));

            // therm_mV += 30; // Experimentally determined to generally be measured as ~30mV lower than actual (multimeter reference)
        }

        therm_raw_avg /= 100.0f;

        // Calculate temp
        // Find resistance based off of reference voltage output (measured as 2.601v)
        // float resistance = ((2.601f * 10000.0f) / (float)(therm_mV/1000.0f)) - 10000.0f;
        // Exponential regression (experimental) based off of raw ADC values seems to work well
        // float resistance_kohms = (184.9902f*exp(-0.00371704f*therm_raw_avg)+15.55841f);
        float resistance_kohms = 794.88817 + (-214.44469 * log10f(therm_raw_avg)) + (-219.13082 * expf(-0.000495547 * therm_raw_avg));

        // Playing with data for a 100k NTC thermistor in Desmos gives this weird regression as best (for above 40C):
        // 44.81207 + -0.000144689x + 110.23167*e^(-0.000103527 * x)

        // Experimental regression: Numbers work until ~65C then we do linefar from there
        // curr_temp = 154.1806f + (-69.60769f * log10(resistance_kohms)) + (0.0740364f * resistance_kohms);
        // if(resistance_kohms > 19)
        //     curr_temp = 161.14291 + (-75.90308 * log10f(resistance_kohms)) + (0.151561 * resistance_kohms);
        // else
        //     curr_temp = (-8.41398 * resistance_kohms) + 229.19328;

        // Steinhard-Hart might not work because of FP shenanegans
        // curr_temp = 1.0f / (0.002036510094 + (0.0003240228966 * logf(resistance_kohms)) + (-0.000001519537509 * pow(logf(resistance_kohms), 3)));
        // B Model - Convert to degrees K for this
        curr_temp = (1.0f / ((1.0f/298.15f) + ((1.0f/3710.83f)*logf(resistance_kohms/84.39f)))) - 273.15;

        // ESP_LOGI("ADC", "Thermistor measured at raw=%f = %d mV, R=%f kOhms, Temp=%f C, dTemp = %f", therm_raw_avg, therm_mV, resistance_kohms, temp, (temp - temp_prev));
        temp_prev = curr_temp;

        // PID skeleton
        // setpoint = 27.0f + encoder_get_count_total();

        // Given in Standard form to do lambda tuning
        // https://blog.opticontrols.com/archives/124
        // https://blog.opticontrols.com/archives/260
        // Now try treating it as an integrating process

        // The response time is massive because we are controlling the temp of a large chunk of metal.
        // We should use the derivative to see what the temp is going to look like in ~10 seconds,
        // and if it will overshoot, then reduce control output
        float e = setpoint - curr_temp;
        float p = (2 * e);
        float i = (0 * int_e);
        float d = (12 * d_e);
        // if(d > 0.0f) {
        //     d = 0.0f;
        // }
        float kc = 2; // Controller gain
        int u = (int)(kc * (e + i + d));

        // Given in Series/Interactive form for Ziegler-Nichols tuning
        // float e = setpoint - temp;
        // float p = (1 * e);
        // float i = (0.033333 * int_e); // 1/30 * int_e
        // float d = (7.5 * d_e);
        // float kc = 1.6371; // Controller gain
        // int u = (int)(kc * (e + i) * (1 + d)); // Must go 0 - 60 for phase

        int_e += (e * (PID_TIME_MS / 1000.0f));
        d_e = (e - prev_e) * (1000.0f / PID_TIME_MS); // Derivative in degrees C/s
        
        // Bounds check control values
        // int_e = (int_e > 200) ? 200 : (int_e < -200) ? -200 : int_e;

        prev_e = e;

        u = (u < 0) ? 0 : (u > 60) ? 60 : u; // Bounds checking
        // ESP_LOGI("Temp PID", "y(t)=%f, e(t)=%f, u(t)=%f, P=%f, I=%f, D=%f, int_e=%f, d_e=%f", temp, e, u, p, i, d, int_e, d_e);

        ESP_LOGI("Temp PID", "t=%lu, r(t)=%f, y(t)=%f, e(t)=%f, u(t)=%d phases, resistance=%f kohms, raw ADC=%f, P=%f, I=%f, D=%f, Kc=%f, int_e=%f, d_e=%f", (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount()), setpoint, curr_temp, e, u, resistance_kohms, therm_raw_avg, p, i, d, kc, int_e, d_e);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 273*u);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        vTaskDelay(PID_TIME_MS / portTICK_PERIOD_MS);
    }
}