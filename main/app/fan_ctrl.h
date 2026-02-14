#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

esp_err_t fan_ctrl_init(void);
void fan_ctrl_set_power(bool on);
void fan_ctrl_set_speed(uint8_t speed); // 0-100
bool fan_ctrl_get_power(void);
uint8_t fan_ctrl_get_speed(void);

#ifdef __cplusplus
}
#endif
