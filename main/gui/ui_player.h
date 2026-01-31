/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the player page object (stub - returns NULL for minimal demo)
 *
 * @return lv_obj_t* Player page object, or NULL if not available
 */
lv_obj_t *get_player_page(void);

#ifdef __cplusplus
}
#endif
