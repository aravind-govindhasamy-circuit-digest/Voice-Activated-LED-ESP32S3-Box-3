#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t fan_ui_start(void);
void fan_ui_update(bool on, uint8_t speed);
void fan_ui_show_fan_screen(void);
void fan_ui_show_sensor_screen(void);

#ifdef __cplusplus
}
#endif
