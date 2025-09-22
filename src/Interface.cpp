/**
 * File for I/O with outside world
 * 
 * Encoder, etc
 */

#include "Interface.h"

// Encoder configuration
// Button state
static bool encoder_button_state = false;
static bool encoder_button_state_sticky = false;

// Use ESP32 PCNT module (Pulse Counter)

// No interrupt needed
static pcnt_unit_config_t encoder_unit_struct = {
    .low_limit = -100,
    .high_limit = 100,
};

static pcnt_unit_handle_t encoder_handle;

static pcnt_channel_handle_t encoder_channel_a;
static pcnt_channel_handle_t encoder_channel_b;

// Debounce filter
static pcnt_glitch_filter_config_t glitch_filter = {
    .max_glitch_ns = 1000,
};

static pcnt_chan_config_t encoder_channel_config_a = {
    .edge_gpio_num = IO_ENC_A,
    .level_gpio_num = IO_ENC_B,
};

static pcnt_chan_config_t encoder_channel_config_b = {
    .edge_gpio_num = IO_ENC_B,
    .level_gpio_num = IO_ENC_A,
};

// Encoder button
static gpio_config_t encoder_button_conf = {
    .pin_bit_mask = (1ULL << IO_ENC_BUTTON),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = (gpio_pullup_t)GPIO_PULLUP_DISABLE, // External pullup
    .pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_ANYEDGE,
};

IRAM_ATTR void encoder_button_isr(void *arg) {
    // Set global button state
    encoder_button_state = gpio_get_level(IO_ENC_BUTTON);
    encoder_button_state_sticky = encoder_button_state || encoder_button_state_sticky;
    // Set LED
    // gpio_set_level(IO_LED_RED, encoder_button_state);
    return;
}

esp_err_t encoder_init() {
    esp_err_t ret;
    
    // Init encoder rotation counter
    ret = pcnt_new_unit(&encoder_unit_struct, &encoder_handle);
    ESP_ERROR_CHECK(ret);
    ret = pcnt_unit_set_glitch_filter(encoder_handle, &glitch_filter);
    ESP_ERROR_CHECK(ret);
    ret = pcnt_new_channel(encoder_handle, &encoder_channel_config_a, &encoder_channel_a);
    ESP_ERROR_CHECK(ret);
    // ret = pcnt_new_channel(encoder_handle, &encoder_channel_config_b, &encoder_channel_b);
    // ESP_ERROR_CHECK(ret);
    ret = pcnt_channel_set_edge_action(encoder_channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    ESP_ERROR_CHECK(ret);
    ret = pcnt_channel_set_level_action(encoder_channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    ESP_ERROR_CHECK(ret);
    // ret = pcnt_channel_set_edge_action(encoder_channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    // ESP_ERROR_CHECK(ret);
    // ret = pcnt_channel_set_level_action(encoder_channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    ESP_ERROR_CHECK(ret);
    ret = pcnt_unit_enable(encoder_handle);
    ESP_ERROR_CHECK(ret);
    ret = pcnt_unit_clear_count(encoder_handle);
    ESP_ERROR_CHECK(ret);
    ret = pcnt_unit_start(encoder_handle);
    ESP_ERROR_CHECK(ret);

    // Init encoder button GPIO
    ret = gpio_config(&encoder_button_conf);
    ESP_ERROR_CHECK(ret);
    ret = gpio_install_isr_service(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(ret);
    ret = gpio_isr_handler_add(IO_ENC_BUTTON, encoder_button_isr, NULL);
    ESP_ERROR_CHECK(ret);

    return ESP_OK;
}

// Return count since last check
static int64_t last_check_count = 0;
int encoder_get_count() {
    int64_t curr = encoder_get_count_total();
    int64_t ret = curr - last_check_count;

    last_check_count = curr;

    return (int)ret;
}

static int64_t global_encoder_count = 0;
// Return total turns done by encoder
int64_t encoder_get_count_total() {
    int ret = 0;
    int curr_count = 0;
    ret = pcnt_unit_get_count(encoder_handle, &curr_count);
    ESP_ERROR_CHECK(ret);
    ret = pcnt_unit_clear_count(encoder_handle);
    ESP_ERROR_CHECK(ret);

    global_encoder_count += (curr_count / 2);

    return global_encoder_count;
}

bool encoder_get_button_state() {
    return encoder_button_state;
}

// State stays until read
// Useful for LVGL's intermittent refreshes
bool encoder_get_button_state_sticky() {
    bool ret = encoder_button_state_sticky;
    encoder_button_state_sticky = false;
    return ret;
}