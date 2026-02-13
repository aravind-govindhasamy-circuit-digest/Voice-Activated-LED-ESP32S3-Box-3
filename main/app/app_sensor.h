#pragma once

#include "esp_err.h"
#include <stdbool.h>


/**
 * @brief Initialize the AHT20 sensor (I2C)
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t app_sensor_init(void);

/**
 * @brief Get Temperature and Humidity from AHT20
 *
 * @param temp Pointer to float to store temperature (Celsius)
 * @param hum Pointer to float to store humidity (%)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t app_sensor_get_values(float *temp, float *hum);

/**
 * @brief Get illuminance from BH1750
 *
 * @return float lux value
 */
float app_sensor_get_lux(void);

/**
 * @brief Get presence status
 *
 * @return true if present, false otherwise
 */
bool app_sensor_get_presence(void);
