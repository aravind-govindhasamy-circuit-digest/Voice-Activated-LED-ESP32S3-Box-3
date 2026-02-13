#pragma once

#include "esp_err.h"

/**
 * @brief Initialize WiFi station mode
 *
 * @return
 *    - ESP_OK: success
 *    - ESP_FAIL: failure
 */
esp_err_t app_wifi_init(void);
