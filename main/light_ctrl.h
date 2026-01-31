/*
 * Light control: binds persisted state to the board LED implementation.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t light_ctrl_init(void);
bool light_ctrl_get(void);
esp_err_t light_ctrl_set(bool on);
esp_err_t light_ctrl_toggle(void);

#ifdef __cplusplus
}
#endif

