/**
 * @file main_screen_gen.c
 * @description Template source file for LVGL objects
 */

/*********************
 *      INCLUDES
 *********************/
#include "main_screen_gen.h"
#include "ui.h"

/*********************
 *      DEFINES
 *********************/



/**********************
 *      TYPEDEFS
 **********************/

/***********************
 *  STATIC VARIABLES
 **********************/

/***********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_obj_t * main_screen_create(void)
{
    LV_TRACE_OBJ_CREATE("begin");

    static lv_style_t main;

    static bool style_inited = false;

    if (!style_inited) {
        lv_style_init(&main);
        lv_style_set_pad_all(&main, 0);
        lv_style_set_border_width(&main, 0);
        lv_style_set_radius(&main, 0);
        lv_style_set_layout(&main, LV_LAYOUT_FLEX);
        lv_style_set_flex_flow(&main, LV_FLEX_FLOW_COLUMN);
        lv_style_set_pad_row(&main, 0);

        style_inited = true;
    }

    lv_obj_t * lv_obj_0 = lv_obj_create(NULL);
    lv_obj_set_width(lv_obj_0, lv_pct(100));
    lv_obj_set_height(lv_obj_0, lv_pct(100));
    lv_obj_add_style(lv_obj_0, &main, 0);

    lv_obj_t * row = row_create(lv_obj_0);
    lv_obj_set_name(row, "row");

    lv_obj_t * col1 = column_create(row);
    lv_obj_set_name(col1, "col1");

    lv_obj_t * lv_bar_0 = lv_bar_create(col1);
    lv_obj_set_height(lv_bar_0, 25);


    lv_obj_t * curr_brew_time_elapsed = lv_label_create(col1);
    lv_obj_set_name(curr_brew_time_elapsed, "curr_brew_time_elapsed");


    lv_obj_t * curr_water_temp = lv_label_create(col1);
    lv_obj_set_name(curr_water_temp, "curr_water_temp");


    lv_obj_t * set_temp = lv_label_create(col1);
    lv_obj_set_name(set_temp, "set_temp");


    lv_obj_t * temperature_chart = lv_chart_create(col1);
    lv_obj_set_name(temperature_chart, "temperature_chart");
    lv_chart_set_point_count(temperature_chart, 50);
    lv_obj_set_width(temperature_chart, 300);
    lv_obj_set_height(temperature_chart, 200);

    lv_obj_t * brew_temp_chart_scale = lv_scale_create(temperature_chart);
    lv_obj_set_name(brew_temp_chart_scale, "brew_temp_chart_scale");
    lv_scale_set_mode(brew_temp_chart_scale, LV_SCALE_MODE_VERTICAL_LEFT);
    lv_obj_set_height(brew_temp_chart_scale, lv_pct(100));
    lv_obj_set_x(brew_temp_chart_scale, -110);




    lv_obj_t * col2 = column_create(row);
    lv_obj_set_name(col2, "col2");

    lv_obj_t * brew_button = lv_button_create(col2);
    lv_obj_set_name(brew_button, "brew_button");

    lv_obj_t * lv_label_0 = lv_label_create(brew_button);
    lv_label_set_text(lv_label_0, "Brew");

    lv_obj_add_event_cb(brew_button, brew_click_cb, LV_EVENT_CLICKED, NULL);





    LV_TRACE_OBJ_CREATE("finished");

    lv_obj_set_name(lv_obj_0, "main_screen");

    return lv_obj_0;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/