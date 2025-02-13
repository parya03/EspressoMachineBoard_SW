#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "board_common.h"
#include "Display.h"

#include <cstring>
#include <cstdlib>

#ifdef __cplusplus
extern "C" {
#endif

// LED IO
gpio_config_t led_io_conf = {
    .pin_bit_mask = ((1ULL << GPIO_NUM_46) | (1ULL << GPIO_NUM_48)),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = (gpio_pullup_t)GPIO_PULLUP_DISABLE,
    .pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};

void app_main() {
    

    esp_err_t ret;
    uint8_t led_state = 0;
    
    // Fill lcd_fb with blue
    for(int i = 0; i < 20 * 480; i++) {
        lcd_fb[i] = 0x001F;
    }

    lcd_init();

    uint16_t color = 0;

    while(1) {

        lcd_fill_color(color++);
    
        gpio_set_level(GPIO_NUM_46, (led_state));
        gpio_set_level(GPIO_NUM_48, (led_state));

        led_state = !led_state;

        // gpio_dump_io_configuration(stdout, (1ULL << 46));
    }
}

#ifdef __cplusplus
}
#endif