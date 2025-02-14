#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"

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
    xTaskCreatePinnedToCore(display_task, "display_task", 10240, NULL, 10, NULL, 1);

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