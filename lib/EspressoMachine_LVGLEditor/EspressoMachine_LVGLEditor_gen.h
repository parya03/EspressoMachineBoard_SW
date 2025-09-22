/**
 * @file EspressoMachine_LVGLEditor_gen.h
 */

#ifndef ESPRESSOMACHINE_LVGLEDITOR_GEN_H
#define ESPRESSOMACHINE_LVGLEDITOR_GEN_H

#ifndef UI_SUBJECT_STRING_LENGTH
#define UI_SUBJECT_STRING_LENGTH 256
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif

/*********************
 *      DEFINES
 *********************/



/**********************
 *      TYPEDEFS
 **********************/



/**********************
 * GLOBAL VARIABLES
 **********************/

/*-------------------
 * Permanent screens
 *------------------*/
extern lv_obj_t * main_screen;

/*----------------
 * Global styles
 *----------------*/


/*----------------
 * Fonts
 *----------------*/


/*----------------
 * Images
 *----------------*/

/*----------------
 * Subjects
 *----------------*/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/*----------------
 * Event Callbacks
 *----------------*/
void brew_click_cb(lv_event_t * e);

/**
 * Initialize the component library
 */

void EspressoMachine_LVGLEditor_init_gen(const char * asset_path);

/**********************
 *      MACROS
 **********************/

/**********************
 *   POST INCLUDES
 **********************/

/*Include all the widget and components of this library*/
#include "components/column/column_gen.h"
#include "components/row/row_gen.h"
#include "screens/main_screen/main_screen_gen.h"

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*ESPRESSOMACHINE_LVGLEDITOR_GEN_H*/