/**
 * ST7796S Display driver
 * 
 * Fairly simple - just needs to initialize and support dumping a framebuffer for LVGL
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "lvgl.h"
#include "ui.h"

#include "board_common.h"
#include "Interface.h"

#include <cstring>
#include <cstdlib>

#define LCD_WIDTH 480
#define LCD_HEIGHT 320

#define LCD_FB_SIZE_BYTES (20 * 480 * 2)

extern DMA_ATTR uint16_t lcd_fb[20 * 480];

// LVGL mutex
extern SemaphoreHandle_t lvgl_mutex;

esp_err_t lcd_init();
esp_err_t lcd_write_fb_ptr(int section, uint16_t *fb_ptr);
esp_err_t lcd_fill_color(uint16_t color);
void display_task(void *pvParameters);

#endif