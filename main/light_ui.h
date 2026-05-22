/*
 * Minimal LVGL UI: full-screen image button that toggles the light.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t light_ui_start(void);
void light_ui_set(bool on);
void light_ui_show_main_screen(void);
void light_ui_show_status_screen(void);

#ifdef __cplusplus
}
#endif

