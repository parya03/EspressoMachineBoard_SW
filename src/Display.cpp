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

static TaskHandle_t fb_flush_task_handle;
static SemaphoreHandle_t fb_flush_sem;

SemaphoreHandle_t lvgl_mutex;

lv_display_t *lvgl_display;
lv_indev_t *lvgl_encoder_input; // Input encoder

// LCD framebuffer
DMA_ATTR uint16_t lcd_fb[20 * 480] = { 0 };
DMA_ATTR uint8_t command;
DMA_ATTR uint16_t data[2];

spi_bus_config_t buscfg = { 0 };
spi_device_interface_config_t devcfg = { 0 };

IRAM_ATTR void lcd_spi_pre_transfer_callback(spi_transaction_t *trans) {
    // Set D/C properly based off of user defined thing
    gpio_set_level(IO_LCD_DC, (int)(trans->user));

    // Take TX mutex so thread can block until transfer is done
    xSemaphoreTakeFromISR(fb_flush_sem, NULL);

    return;
}

IRAM_ATTR void lcd_transfer_done_cb(spi_transaction_t *trans) {
    // Notify task that transfer is done
    xSemaphoreGiveFromISR(fb_flush_sem, NULL);

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
    {0x36, {0x34}, 1}, // MADCTL
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
    if(ret != ESP_OK) {
        ESP_LOGE("Display", "Error writing display framebuffer - Tx length (bits): %d, Max Tx length (bits): %d", transaction.length, LCD_FB_SIZE_BYTES * 8);
    }
    ESP_ERROR_CHECK(ret);

    return 0;
}

esp_err_t lcd_write_fb_xy(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint16_t *fb_ptr) {
    // ESP_LOGI(TAG, "Writing LCD section %d", section);
    uint8_t command = 0x00;
    uint16_t data[2] = { 0 };
    spi_transaction_t transaction = { 0 };
    // Calculate from x and y bounds
    uint32_t bytes_written = ((x2 - x1) + 1) * ((y2 - y1) + 1) * 2;

    esp_err_t ret;

    // Set row address
    command = 0x2B; // RASET
    // Going y_start -> y_end
    data[0] = (y1 >> 8) | (y1 << 8); // Fix endianness
    data[1] = (y2 >> 8) | (y2 << 8); // Fix endianness
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
    data[0] = (x1 >> 8) | (x1 << 8);
    data[1] = (x2 >> 8) | (x2 << 8); // Fix endianness
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
    transaction.length = bytes_written * 8; // Length in bits
    transaction.user = (void *)1; // D/C HIGH
    ret = spi_device_polling_transmit(spi, &transaction);
    ESP_ERROR_CHECK(ret);

    return 0;
}

// 16-bit color
// Overwrites FB with color
esp_err_t lcd_fill_color(uint16_t color) {
    ESP_LOGI(TAG, "Fill LCD with color 0x%4X", color);

    for(int i = 0; i < (LCD_FB_SIZE_BYTES / 2); i++) {
        lcd_fb[i] = color;
    }

    for(int i = 0; i < 16; i++) {
        lcd_write_fb_ptr(i, lcd_fb);
    }

    return 0;
}

struct Args {
    lv_area_t area;
    uint16_t *fb;
} args;

// FB flush FreeRTOS task
void lvgl_flush_fb_task(void *pvParameters) {
    struct Args *args = (struct Args *)pvParameters;
    const lv_area_t *area = &(args->area);
    uint16_t *lvgl_fb = args->fb;

    // ESP_LOGI(TAG, "Writing LCD section %d", section);
    uint8_t command = 0x00;
    uint16_t data[2] = { 0 };
    spi_transaction_t transaction = { 0 };
    // Calculate from x and y bounds
    uint32_t bytes_written = ((area->x2 - area->x1) + 1) * ((area->y2 - area->y1) + 1) * 2;

    esp_err_t ret;

    // Set row address
    command = 0x2B; // RASET
    // Going y_start -> y_end
    data[0] = (area->y1 >> 8) | (area->y1 << 8); // Fix endianness
    data[1] = (area->y2 >> 8) | (area->y2 << 8); // Fix endianness
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
    data[0] = (area->x1 >> 8) | (area->x1 << 8);
    data[1] = (area->x2 >> 8) | (area->x2 << 8); // Fix endianness
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
    transaction.length = bytes_written * 8; // Length in bits
    transaction.user = (void *)1; // D/C HIGH
    ret = spi_device_queue_trans(spi, &transaction, portMAX_DELAY);
    xSemaphoreTake(fb_flush_sem, portMAX_DELAY); // Block thread until transfer done
    if(ret != ESP_OK) {
        ESP_LOGE("Display", "Error writing display framebuffer - X1: %ld, X2: %ld, Y1: %ld, Y2: %ld, Tx length (bits): %d, Max Tx length (bits): %d", area->x1, area->x2, area->y1, area->y2, transaction.length, LCD_FB_SIZE_BYTES * 8);
    }
    ESP_ERROR_CHECK(ret);

    // Tell LVGL that buffer is done flushing
    lv_display_flush_ready(lvgl_display);

    // Done, end task
    vTaskDelete(NULL);
    return;
}

void lvgl_flush_fb_cb(lv_display_t *disp, const lv_area_t *area, unsigned char *px_map) {

    uint16_t *lvgl_fb = (uint16_t *)px_map;
    
    // ESP_LOGI("Display", "Writing display framebuffer - X1: %ld, X2: %ld, Y1: %ld, Y2: %ld", area->x1, area->x2, area->y1, area->y2);
    args.area = *area;
    args.fb = lvgl_fb;

    // Create async task to flush FB
    xTaskCreate(lvgl_flush_fb_task, "lvgl_flush_fb_task", 4096, &args, 1, &fb_flush_task_handle);

    return;
}

void encoder_read_lvgl_cb(lv_indev_t *drv, lv_indev_data_t *data) {
    // Read encoder
    data->enc_diff = encoder_get_count();

    data->state = (encoder_get_button_state_sticky() ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED);

    return;
}

extern lv_obj_t *main_screen;

// Init sequence from LCD Wiki demo and https://github.com/prenticedavid/Adafruit_ST7796S_kbv/blob/master/Adafruit_ST7796S_kbv.cpp
esp_err_t lcd_init() {
    esp_err_t ret;

    // Create SPI semaphore
    fb_flush_sem = xSemaphoreCreateBinary();
    if(fb_flush_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create SPI semaphore");
        return -1;
    }

    /*******************************************
    * Initialize LCD hardware
    ********************************************/

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
        .clock_speed_hz = 15 * 1000 * 1000,     // Max seems to be 15 MHz based on datasheet
        .spics_io_num = IO_LCD_CS0,             // CS pin
        .queue_size = 32,                        // Queue this many transactions at once
        .pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
        .post_cb = lcd_transfer_done_cb,
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

    lcd_fill_color(0x0000);

    /*******************************************
    * Initialize LVGL
    ********************************************/

    lv_init(); // LVGL init
    lv_tick_set_cb(xTaskGetTickCount); // Set tick count cb
    lvgl_display = lv_display_create(LCD_WIDTH, LCD_HEIGHT);
    lv_display_set_flush_cb(lvgl_display, lvgl_flush_fb_cb);
    // Set FB and render in chunks
    lv_display_set_buffers(lvgl_display, lcd_fb, NULL, LCD_FB_SIZE_BYTES, LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    // Input encoder
    lvgl_encoder_input = lv_indev_create();
    lv_indev_set_type(lvgl_encoder_input, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_read_cb(lvgl_encoder_input, encoder_read_lvgl_cb);
    
    // Create mutex for LVGL
    lvgl_mutex = xSemaphoreCreateMutex();
    
    // Init UI
    ui_init("");
    ESP_LOGI("Display", "Main Screen %X", (unsigned int)main_screen);
    lv_screen_load(main_screen);

    // Force refresh
    // lv_refr_now(lvgl_display);

    // Done
    return 0;
}

extern float curr_temp;
extern float setpoint;
// extern lv_obj_t* curr_temp_bar;
// extern lv_obj_t* set_temp_bar;

// static void bar_set_value(void * bar, int32_t v)
// {
//     lv_bar_set_value((lv_obj_t *)bar, v, LV_ANIM_OFF);
// }

// Display task
void display_task(void *pvParameters) {
    auto curr_temp_label = lv_obj_find_by_name(main_screen, "curr_water_temp");
    auto set_temp_label = lv_obj_find_by_name(main_screen, "set_temp");
    while(1) {
        xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
        gpio_set_level(IO_LED_BLUE, 1);

        // bar_set_value(curr_temp_bar, curr_temp);
        // bar_set_value(set_temp_bar, setpoint);
        lv_label_set_text_fmt(curr_temp_label, "Curr temp: %f C", curr_temp);
        lv_label_set_text_fmt(set_temp_label, "Set temp: %f C", setpoint);

        int time_till_next_ms = lv_task_handler();
        gpio_set_level(IO_LED_BLUE, 0);
        xSemaphoreGive(lvgl_mutex);
        vTaskDelay(time_till_next_ms / portTICK_PERIOD_MS);
    }
    return;
}