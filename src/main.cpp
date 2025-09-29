#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <math.h>

#include "lvgl.h"

#include "board_common.h"
#include "Display.h"
#include "Interface.h"
#include "Control.h"

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

// Pump IO
gpio_config_t pump_io_conf = {
    .pin_bit_mask = ((1ULL << IO_PUMP)),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = (gpio_pullup_t)GPIO_PULLUP_DISABLE,
    .pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};

uint8_t led_state = 0;
uint8_t brew_state = 0;
uint8_t pump_state = 0;

uint32_t tick_brew_started = 0; // Keep track of time

// Start brewing
void brew_task(void *pvParameters) {
    // Set boiler to 93 C (Brew temp)
    setpoint = 70; // Slightly higher so it gets there quicker
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // setpoint = 20.0f; // Slightly higher so it gets there quicker

    // // Wait for it to heat
    // while(curr_temp <= 93.0) {
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    // tick_brew_started = xTaskGetTickCount();

    // // When heated, do the preinfusion
    // gpio_set_level(IO_PUMP, 1);
    // pump_state = 1;
    // // vTaskDelay(2000 / portTICK_PERIOD_MS);
    // // gpio_set_level(IO_PUMP, 0);
    // // pump_state = 0;
    // // Wait a bit then start brewing
    // setpoint = 20.0f; // TESTING
    // vTaskDelay(500 / portTICK_PERIOD_MS);
    // gpio_set_level(IO_PUMP, 1);
    // pump_state = 1;
    // vTaskDelay(35000 / portTICK_PERIOD_MS); // 30 sec brew time to start lol
    // gpio_set_level(IO_PUMP, 0);
    // pump_state = 0;
    // setpoint = 20.0f;

    // Done!



    vTaskDelete(NULL);

}

TaskHandle_t brew_task_handle = 0;
void brew_click_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        ESP_LOGI("Brew Button", "Clicked");
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        // lv_label_set_text_fmt(label, "Button: %d", cnt);
        brew_state = !brew_state;
        lv_obj_set_style_bg_color(btn, brew_state ? lv_color_t(0, 0, 0xFF) : lv_color_t(0xFF, 0, 0), LV_STATE_DEFAULT);

        // gpio_set_level(IO_LED_RED, (brew_state));
        // gpio_set_level(IO_LED_BLUE, (led_state));

        // TEST
        // TEST: Turn on and off boiler SSR with LED
        // gpio_set_level(IO_BOILER, led_state);
        // gpio_set_level(IO_PUMP, brew_state);

        if(brew_state) {
            // xTaskCreate(brew_task, "brew_task", 10240, NULL, 10, &brew_task_handle);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 8192);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        }
        else {
            // vTaskDelete(brew_task_handle);
            // gpio_set_level(IO_PUMP, 0);
            // pump_state = 0;
            // setpoint = 0.0f;

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        }

    }
}

lv_obj_t* curr_temp_bar;
lv_obj_t* set_temp_bar;

static void bar_event_cb(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target_obj(e);

    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.font = LV_FONT_DEFAULT;

    char buf[8];
    int val = lv_bar_get_value(obj);
    // ESP_LOGI("Display", "Float: %.2f", *((float *)(&val)));
    lv_snprintf(buf, sizeof(buf), "%d", val);

    lv_point_t txt_size;
    lv_text_get_size(&txt_size, buf, label_dsc.font, label_dsc.letter_space, label_dsc.line_space, LV_COORD_MAX,
                     label_dsc.flag);

    lv_area_t txt_area;
    txt_area.x1 = 0;
    txt_area.x2 = txt_size.x - 1;
    txt_area.y1 = 0;
    txt_area.y2 = txt_size.y - 1;

    lv_area_t indic_area;
    lv_obj_get_coords(obj, &indic_area);
    lv_area_set_width(&indic_area, lv_area_get_width(&indic_area) * lv_bar_get_value(obj) / 100);

    /*If the indicator is long enough put the text inside on the right*/
    if(lv_area_get_width(&indic_area) > txt_size.x + 20) {
        lv_area_align(&indic_area, &txt_area, LV_ALIGN_RIGHT_MID, -10, 0);
        label_dsc.color = lv_color_white();
    }
    /*If the indicator is still short put the text out of it on the right*/
    else {
        lv_area_align(&indic_area, &txt_area, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
        label_dsc.color = lv_color_black();
    }
    label_dsc.text = buf;
    label_dsc.text_local = true;
    lv_layer_t * layer = lv_event_get_layer(e);
    lv_draw_label(layer, &label_dsc, &txt_area);
}

extern lv_indev_t *lvgl_encoder_input;

void app_main() {

    esp_err_t ret;
    
    // Fill lcd_fb with blue
    // for(int i = 0; i < 20 * 480; i++) {
    //     lcd_fb[i] = 0x001F;
    // }

    gpio_config(&led_io_conf);
    gpio_config(&pump_io_conf);

    control_init();
    encoder_init();
    lcd_init();

    // // Create button
    // lv_obj_t * btn = lv_button_create(lv_screen_active());     /*Add a button the current screen*/
    // lv_obj_set_pos(btn, 10, 10);                            /*Set its position*/
    // lv_obj_set_size(btn, 120, 50);                          /*Set its size*/
    // lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    // lv_obj_t * label = lv_label_create(btn);          /*Add a label to the button*/
    // lv_label_set_text(label, "Button");                     /*Set the labels text*/
    // lv_obj_center(label);

    auto group = lv_group_create();
    lv_group_add_obj(group, lv_obj_find_by_name(main_screen, "brew_button"));
    lv_indev_set_group(lvgl_encoder_input, group);

    // // Create bars for current temp and setpoint
    // curr_temp_bar = lv_bar_create(lv_screen_active());
    // lv_bar_set_range(curr_temp_bar, 0, 100);
    // lv_obj_set_size(curr_temp_bar, 200, 20);
    // lv_obj_align(curr_temp_bar, LV_ALIGN_DEFAULT, 0, 100);
    // lv_obj_add_event_cb(curr_temp_bar, bar_event_cb, LV_EVENT_DRAW_MAIN_END, NULL);
    // // lv_anim_t a;
    // // lv_anim_init(&a);
    // // lv_anim_set_var(&a, curr_temp_bar);
    // // lv_anim_set_values(&a, 0, 100);
    // // lv_anim_set_exec_cb(&a, bar_set_value);
    // // lv_anim_set_duration(&a, 4000);
    // // // lv_anim_set_reverse_duration(&a, 4000);
    // // lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    // // lv_anim_start(&a);

    // set_temp_bar = lv_bar_create(lv_screen_active());
    // lv_bar_set_range(set_temp_bar, 0, 100);
    // lv_obj_set_size(set_temp_bar, 200, 20);
    // lv_obj_align(set_temp_bar, LV_ALIGN_DEFAULT, 0, 150);
    // lv_obj_add_event_cb(set_temp_bar, bar_event_cb, LV_EVENT_DRAW_MAIN_END, NULL);
    // lv_anim_t a1;
    // lv_anim_init(&a1);
    // lv_anim_set_var(&a1, set_temp_bar);
    // lv_anim_set_values(&a1, 0, 100);
    // lv_anim_set_exec_cb(&a1, bar_set_value);
    // lv_anim_set_duration(&a1, 4000);
    // lv_anim_set_reverse_duration(&a, 4000);
    // lv_anim_set_repeat_count(&a1, LV_ANIM_REPEAT_INFINITE);
    // lv_anim_start(&a1);

    // LVGL refresh pinned to second core
    xTaskCreate(temp_control_task, "temp_control_task", 10240, NULL, 10, NULL);
    xTaskCreatePinnedToCore(display_task, "display_task", 20480, NULL, 10, NULL, 1);

    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // u = 4;

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