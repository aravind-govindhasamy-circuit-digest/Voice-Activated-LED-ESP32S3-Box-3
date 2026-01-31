/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Single-LED API: simple on/off on GPIO40. */
esp_err_t app_led_init(void);
esp_err_t app_led_deinit(void);
esp_err_t app_led_set_power(bool power);
bool app_led_get_state(void);

#ifdef __cplusplus
}
#endif
