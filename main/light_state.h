/*
 * Minimal light state (latched) stored in NVS.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t light_state_init(void);
bool light_state_get(void);
esp_err_t light_state_set(bool on);

#ifdef __cplusplus
}
#endif

