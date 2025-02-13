/**
 * Source for display driver
 */

#include "Display.h"

const char *TAG = "Display";

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    int data_len_bytes;
} lcd_write_t;

// Non-SPI GPIO config struct
gpio_config_t lcd_io_conf = {
    // Bit mask of the pins
    .pin_bit_mask = ((1ULL << IO_LCD_LED) | (1ULL << IO_LCD_DC) | (1ULL << IO_LCD_RESET)),
    // Set as output mode
    .mode = GPIO_MODE_OUTPUT,
    // Disable pull-up mode
    .pull_up_en = (gpio_pullup_t)GPIO_PULLUP_DISABLE,
    // Disable pull-down mode
    .pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_DISABLE,
    // Disable interrupt
    .intr_type = GPIO_INTR_DISABLE,
};

spi_device_handle_t spi;

// LCD framebuffer
DMA_ATTR uint16_t lcd_fb[20 * 480] = { 0 };
DMA_ATTR uint8_t command;
DMA_ATTR uint16_t data[2];

spi_bus_config_t buscfg = { 0 };
spi_device_interface_config_t devcfg = { 0 };

IRAM_ATTR void lcd_spi_pre_transfer_callback(spi_transaction_t *trans) {
    // Set D/C properly based off of user defined thing
    gpio_set_level(IO_LCD_DC, (int)(trans->user));

    return;
}

// List of init commands and data - in DRAM for SPI DMA
// {command, {data (uint8_t [])}, data_len_bytes}
DRAM_ATTR lcd_write_t lcd_init_list[] = {
    {0x01, {0}, 0},
    {0xF0, {0xC3}, 1},
    {0xF0, {0x96}, 1},
    // {0x36, {0x68}, 1}, // Prev 0x48
    // {0x3A, {0x05}, 1},
    // {0xB0, {0x80}, 1},
    // {0xB6, {0x00, 0x02}, 2},
    // {0xB5, {0x02, 0x03, 0x00, 0x04}, 4},
    // {0xB1, {0x80, 0x10}, 2},
    // {0xB4, {0x00}, 1},
    // {0xB7, {0xC6}, 1},
    // {0xC5, {0x24}, 1},
    // {0xE4, {0x31}, 1},
    // {0xE8, {0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33}, 8},
    // {0xC2, {0xA7}, 1},
    // {0xE0, {0xF0, 0x09, 0x13, 0x12, 0x12, 0x2B, 0x3C, 0x44, 0x4B, 0x1B, 0x18, 0x17, 0x1D, 0x21}, 14},
    // {0xE1, {0xF0, 0x09, 0x13, 0x0C, 0x0D, 0x27, 0x3B, 0x44, 0x4D, 0x0B, 0x17, 0x17, 0x1D, 0x21}, 14},
    // {0x36, {0x48}, 1},
    {0xC5, {0x1C}, 1},
    {0x36, {0x74}, 1}, // MADCTL
    {0x3A, {0x55}, 1},
    {0xB0, {0x80}, 1},
    {0xB4, {0x01}, 1},
    {0xB6, {0x80, 0x02, 0x3B}, 3},
    {0xB7, {0xC6}, 1},
    {0xF0, {0x69}, 1},
    {0xF0, {0x3C}, 1},
    // {0x13, {0}, 0},
    {0x11, {0}, 0},
    {0x29, {0}, 0},
};

// Combine command with data
static void lcd_write(lcd_write_t write_struct) {
    spi_transaction_t transaction = {0};

    transaction.tx_buffer = &write_struct.cmd;
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

// Init sequence from LCD Wiki demo and https://github.com/prenticedavid/Adafruit_ST7796S_kbv/blob/master/Adafruit_ST7796S_kbv.cpp
esp_err_t lcd_init() {
    esp_err_t ret;

    // Initialize non-SPI GPIOs
    gpio_config(&lcd_io_conf);

    // Initialize SPI
    buscfg = {
        .mosi_io_num = IO_LCD_MOSI,
        .miso_io_num = IO_LCD_MISO,
        .sclk_io_num = IO_LCD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_FB_SIZE_BYTES,
    };

    devcfg = {
        .mode = 0,                              // CPOL,CPHA = 0,0
        .clock_speed_hz = 10 * 1000 * 1000,     // Clock out at 10 MHz, TODO: Check
        .spics_io_num = IO_LCD_CS0,             // CS pin
        .queue_size = 32,                        // Queue this many transactions at once
        .pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
    };

    // Do the thing
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    // Init LCD

    // HW reset LCD
    gpio_set_level(IO_LCD_RESET, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(IO_LCD_RESET, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    spi_transaction_t transaction = { 0 };

    uint8_t rx_data_buf[10] = { 0 };

    // Write out init commands with delay between them
    for(int i = 0; i < 14; i++) {
        ESP_LOGI("LCD", "Writing command 0x%2X", lcd_init_list[i].cmd);
        lcd_write(lcd_init_list[i]);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);
    // DISP_ON
    lcd_write({0x29, {}, 0});

    // Done
    return 0;
}

// LCD divided up into 320/20 = 16 sections to reduce FB size
// because ESPIDF complains if trying to DMA a too big buffer
// Section 0 is top, 15 is bottom
esp_err_t lcd_write_fb_ptr(int section, uint16_t *fb_ptr) {
    // ESP_LOGI(TAG, "Writing LCD section %d", section);

    // Find write y coordinates based on section (x is always 0->480)
    uint16_t y_start = section * 20;
    uint16_t y_end = y_start + 20;
    
    uint8_t command = 0x00;
    uint16_t data[2] = { 0 };
    spi_transaction_t transaction = { 0 };

    esp_err_t ret;

    // Set row address
    command = 0x2B; // RASET
    // Going y_start -> y_end
    data[0] = (y_start >> 8) | (y_start << 8); // Fix endianness
    data[1] = (y_end >> 8) | (y_end << 8); // Fix endianness
    // Send command
    transaction.tx_buffer = &command;
    transaction.rx_buffer = NULL;
    transaction.length = 8;
    transaction.user = (void *)0; // D/C LOW
    ret = spi_device_polling_transmit(spi, &transaction);
    // Send data
    ESP_ERROR_CHECK(ret);
    transaction.tx_buffer = &data;
    transaction.length = 32;
    transaction.user = (void *)1; // D/C HIGH
    ret = spi_device_polling_transmit(spi, &transaction);
    ESP_ERROR_CHECK(ret);

    // Set column address
    command = 0x2A; // CASET
    data[0] = (0 >> 8) | (0 << 8);
    data[1] = (500 >> 8) | (500 << 8); // Fix endianness
    // Send command
    transaction.tx_buffer = &command;
    transaction.rx_buffer = NULL;
    transaction.length = 8;
    transaction.user = (void *)0; // D/C LOW
    ret = spi_device_polling_transmit(spi, &transaction);
    // Send data
    ESP_ERROR_CHECK(ret);
    transaction.tx_buffer = &data;
    transaction.length = 32;
    transaction.user = (void *)1; // D/C HIGH
    ret = spi_device_polling_transmit(spi, &transaction);
    ESP_ERROR_CHECK(ret);

    // Send framebuffer array
    command = 0x2C; // RAMWR
    // Send command
    transaction.tx_buffer = &command;
    transaction.rx_buffer = NULL;
    transaction.length = 8;
    transaction.user = (void *)0; // D/C LOW
    ret = spi_device_polling_transmit(spi, &transaction);
    // Send data
    ESP_ERROR_CHECK(ret);
    transaction.tx_buffer = &lcd_fb;
    transaction.length = LCD_FB_SIZE_BYTES * 8; // Length in bits
    transaction.user = (void *)1; // D/C HIGH
    ret = spi_device_polling_transmit(spi, &transaction);
    ESP_ERROR_CHECK(ret);

    return 0;
}

// 16-bit color
// Overwrites FB with color
esp_err_t lcd_fill_color(uint16_t color) {
    ESP_LOGI(TAG, "Fill LCD with color 0x%4X", color);

    for(int i = 0; i < 20 * 480; i++) {
        lcd_fb[i] = color;
    }

    for(int i = 0; i < 16; i++) {
        lcd_write_fb_ptr(i, lcd_fb);
    }

    return 0;
}