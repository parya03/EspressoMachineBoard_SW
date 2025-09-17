#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include <math.h>

#include "lvgl.h"

#include "board_common.h"
#include "Display.h"
#include "Interface.h"

#include <cstring>
#include <cstdlib>

#ifdef __cplusplus
extern "C" {
#endif

// LED IO
gpio_config_t led_io_conf = {
    .pin_bit_mask = ((1ULL << IO_LED_RED) | (1ULL << IO_LED_BLUE)),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = (gpio_pullup_t)GPIO_PULLUP_DISABLE,
    .pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};

// Boiler IO
gpio_config_t boiler_io_conf = {
    .pin_bit_mask = ((1ULL << IO_BOILER)),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = (gpio_pullup_t)GPIO_PULLUP_DISABLE,
    .pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};

// Pump IO
gpio_config_t pump_io_conf = {
    .pin_bit_mask = ((1ULL << IO_PUMP)),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = (gpio_pullup_t)GPIO_PULLUP_DISABLE,
    .pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};

uint8_t led_state = 0;

static void btn_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        ESP_LOGI("Button", "Clicked");
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Button: %d", cnt);

        led_state = !led_state;

        gpio_set_level(IO_LED_RED, (led_state));
        // gpio_set_level(IO_LED_BLUE, (led_state));

        // TEST
        // TEST: Turn on and off boiler SSR with LED
        gpio_set_level(IO_BOILER, led_state);
        gpio_set_level(IO_PUMP, led_state);

    }
}

adc_oneshot_unit_handle_t adc1_handle = NULL;
adc_cali_handle_t adc1_cali_handle = NULL;

void temp_measure_task(void *pvParameters) {
    float therm_raw_avg = 0;
    int therm_mV = 0;
    float temp_prev = 0.0f;

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
        float resistance_kohms = (184.9902f*exp(-0.00371704f*therm_raw_avg)+15.55841f);

        // Playing with data for a 100k NTC thermistor in Desmos gives this weird regression as best (for above 40C):
        // 44.81207 + -0.000144689x + 110.23167*e^(-0.000103527 * x)
        float temp = 154.1806f + (-69.60769f * log10(resistance_kohms)) + (0.0740364f * resistance_kohms);

        ESP_LOGI("ADC", "Thermistor measured at raw=%f = %d mV, R=%f kOhms, Temp=%f C, dTemp = %f", therm_raw_avg, therm_mV, resistance_kohms, temp, (temp - temp_prev));
        temp_prev = temp;

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

extern lv_indev_t *lvgl_encoder_input;

void app_main() {

    esp_err_t ret;
    
    // Fill lcd_fb with blue
    // for(int i = 0; i < 20 * 480; i++) {
    //     lcd_fb[i] = 0x001F;
    // }

    gpio_config(&led_io_conf);
    gpio_config(&boiler_io_conf);
    gpio_config(&pump_io_conf);
    
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

    encoder_init();
    lcd_init();

    lv_obj_t * btn = lv_button_create(lv_screen_active());     /*Add a button the current screen*/
    lv_obj_set_pos(btn, 10, 10);                            /*Set its position*/
    lv_obj_set_size(btn, 120, 50);                          /*Set its size*/
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    lv_obj_t * label = lv_label_create(btn);          /*Add a label to the button*/
    lv_label_set_text(label, "Button");                     /*Set the labels text*/
    lv_obj_center(label);

    auto group = lv_group_create();
    lv_group_add_obj(group, btn);
    lv_indev_set_group(lvgl_encoder_input, group);

    // LVGL refresh pinned to second core
    xTaskCreate(temp_measure_task, "temp_measure_task", 10240, NULL, 10, NULL);
    xTaskCreatePinnedToCore(display_task, "display_task", 10240, NULL, 10, NULL, 1);

    // TEST: Turn on and off boiler SSR
    // while(1) {
    //     gpio_set_level(IO_BOILER, 0);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    //     gpio_set_level(IO_BOILER, 1);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    // Return from this thread
    vTaskDelete(NULL);

    // uint16_t color = 0;
    // while(1) {
    //     gpio_set_level(IO_LED_BLUE, 1);
    //     int time_till_next = lv_timer_handler();
    //     gpio_set_level(IO_LED_BLUE, 0);
    //     // ESP_LOGI("Main:", "Time till next: %d", time_till_next);
    //     vTaskDelay(time_till_next / portTICK_PERIOD_MS);

    //     // ESP_LOGI("Main:", "Encoder count: %d", encoder_get_count());

    //     // lcd_fill_color(color++);
    
    //     // gpio_set_level(GPIO_NUM_46, (led_state));
    //     // gpio_set_level(GPIO_NUM_48, (led_state));

    //     // led_state = !led_state;

    //     // gpio_dump_io_configuration(stdout, (1ULL << 46));
    // }
}

#ifdef __cplusplus
}
#endif