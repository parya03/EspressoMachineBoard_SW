#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "board_common.h"

#include <cstring>
#include <cstdlib>

#ifdef __cplusplus
extern "C" {
#endif

spi_device_handle_t spi;

// LCD framebuffer
DMA_ATTR uint8_t lcd_fb[20 * 480 * 2];
DMA_ATTR uint8_t command;
DMA_ATTR uint16_t data[2];

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    int data_len_bytes;
} lcd_write_t;

IRAM_ATTR void lcd_spi_pre_transfer_callback(spi_transaction_t *trans) {
    // Set D/C properly based off of user defined thing
    gpio_set_level(IO_LCD_DC, (int)(trans->user));
}

// Combine command with data
void lcd_write(lcd_write_t write_struct) {
    spi_transaction_t transaction = {0};

    transaction.tx_buffer = &command;
    transaction.length = 8;
    transaction.user = (void *)0; // D/C LOW
    esp_err_t ret = spi_device_polling_transmit(spi, &transaction);
    ESP_ERROR_CHECK(ret);

    // Only command
    if(write_struct.data_len_bytes == 0) {
        return;
    }

    transaction.tx_buffer = write_struct.data;
    transaction.length = write_struct.data_len_bytes * 8;
    transaction.user = (void *)1; // D/C HIGH
    ret = spi_device_polling_transmit(spi, &transaction);
    ESP_ERROR_CHECK(ret);

    return;
}

DRAM_ATTR lcd_write_t lcd_init_list[] = {
    {0xF0, {0xC3}, 1},
    {0xF0, {0x96}, 1},
    {0x36, {0x68}, 1}, // Prev 0x48
    {0x3A, {0x05}, 1},
    {0xB0, {0x80}, 1},
    {0xB6, {0x20, 0x02}, 2},
    {0xB5, {0x02, 0x03, 0x00, 0x04}, 4},
    {0xB1, {0x80, 0x10}, 2},
    {0xB4, {0x00}, 1},
    {0xB7, {0xC6}, 1},
    {0xC5, {0x1C}, 1},
    {0xE4, {0x31}, 1},
    {0xE8, {0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33}, 8},
    {0xC2, {0xA7}, 1},
    {0xE0, {0xF0, 0x09, 0x13, 0x12, 0x12, 0x2B, 0x3C, 0x44, 0x4B, 0x1B, 0x18, 0x17, 0x1D, 0x21}, 14},
    {0xE1, {0xF0, 0x09, 0x13, 0x0C, 0x0D, 0x27, 0x3B, 0x44, 0x4D, 0x0B, 0x17, 0x17, 0x1D, 0x21}, 14},
    {0xF0, {0x3C}, 1},
    {0xF0, {0x69}, 1},
    {0x13, {0}, 0},
    {0x11, {0}, 0},
};

// Init sequence from LCD Wiki demo
esp_err_t lcd_init() {
    // Write out init commands
    for(int i = 0; i < 20; i++) {
        ESP_LOGI("LCD", "Writing command 0x%2X", lcd_init_list[i].cmd);
        lcd_write(lcd_init_list[i]);
    }

    vTaskDelay(120 / portTICK_PERIOD_MS);
    // DISP_ON
    lcd_write({0x29, {}, 0});

    return 0;
}

void app_main() {
    gpio_config_t io_conf = {};

    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((1ULL << IO_LCD_DC) | (1ULL << IO_LCD_RESET));
    //disable pull-down mode
    io_conf.pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t)GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .mosi_io_num = IO_LCD_MOSI,
        .miso_io_num = IO_LCD_MISO,
        .sclk_io_num = IO_LCD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 20 * 480 * 2 // 320 x 480 px * 2 bytes/px + 1 byte command
    };
    spi_device_interface_config_t devcfg = {
        .mode = 1,                              // TODO: Check CPOL/CPHA
        .clock_speed_hz = 10 * 1000 * 1000,     // Clock out at 10 MHz, TODO: Check
        .spics_io_num = IO_LCD_CS0,             // CS pin
        .queue_size = 32,                        // We want to be able to queue 7 transactions at a time
        .pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
    };

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    // Init LCD
    // Reset LCD
    gpio_set_level(IO_LCD_RESET, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(IO_LCD_RESET, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Write out RED
    // Set FB to 16-bit color red
    for(int i = 0; i < (20 * 480); i++) {
        // if(i % 2 == 0) {
        //     lcd_fb[i] = 0xF8;
        // }
        ((uint16_t *)lcd_fb)[i] = 0xF800;
    }
    spi_transaction_t transaction = { 0 };

    lcd_init();

    uint8_t rx_data_buf[10] = { 0 };

    // Get LCD data
    command = 0x04;
    data[0] = 0;
    data[1] = (480U >> 8 | 480U << 8);
    transaction.tx_buffer = &command;
    transaction.rx_buffer = rx_data_buf;
    transaction.length = 8;
    transaction.user = (void *)0; // D/C LOW
    ret = spi_device_polling_transmit(spi, &transaction);


    while(1) {
        
        // ESP_LOGI("LCD", "Sending CASET");
        // Set FB write pointer to (0, 0) to (20, 480)
        // CASET
        command = 0x2A;
        data[0] = 0;
        data[1] = (480U >> 8 | 480U << 8);
        transaction.tx_buffer = &command;
        transaction.length = 8;
        transaction.user = (void *)0; // D/C LOW
        ret = spi_device_polling_transmit(spi, &transaction);
        ESP_ERROR_CHECK(ret);
        transaction.tx_buffer = &data;
        transaction.length = 32;
        transaction.user = (void *)1; // D/C HIGH
        ret = spi_device_polling_transmit(spi, &transaction);
        ESP_ERROR_CHECK(ret);

        // ESP_LOGI("LCD", "Sending RASET");
        // RASET
        command = 0x2B;
        data[0] = 0;
        data[1] = (20) & 0xFF;
        transaction.tx_buffer = &command;
        transaction.length = 8;
        transaction.user = (void *)0; // D/C LOW
        ret = spi_device_polling_transmit(spi, &transaction);
        ESP_ERROR_CHECK(ret);
        transaction.tx_buffer = &data;
        transaction.length = 32;
        transaction.user = (void *)1; // D/C HIGH
        ret = spi_device_polling_transmit(spi, &transaction);
        ESP_ERROR_CHECK(ret);

        // ESP_LOGI("LCD", "Sending RAMWR (framebuffer)");
        // RAMWR
        command = 0x2C;
        transaction.tx_buffer = &command;
        transaction.length = 8;
        transaction.user = (void *)0; // D/C LOW
        ret = spi_device_polling_transmit(spi, &transaction);
        ESP_ERROR_CHECK(ret);
        transaction.tx_buffer = &lcd_fb;
        transaction.length = 20 * 480 * 16;
        transaction.user = (void *)1; // D/C HIGH
        ret = spi_device_polling_transmit(spi, &transaction);
        ESP_ERROR_CHECK(ret);

    
        // gpio_set_level(GPIO_NUM_46, 1);
        // gpio_set_level(GPIO_NUM_48, 1);

        // for(volatile int i = 0; i < 1000000; i++);

        // gpio_set_level(GPIO_NUM_46, 0);
        // gpio_set_level(GPIO_NUM_48, 0);

        // for(volatile int i = 0; i < 1000000; i++);

        // gpio_dump_io_configuration(stdout, (1ULL << 46));
    }
}

#ifdef __cplusplus
}
#endif