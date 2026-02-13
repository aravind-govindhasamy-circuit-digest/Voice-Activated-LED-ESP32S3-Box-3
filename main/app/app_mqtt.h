#pragma once

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Initialize MQTT client
 *
 * @return
 *    - ESP_OK: success
 *    - ESP_FAIL: failure
 */
esp_err_t app_mqtt_init(void);

/**
 * @brief Publish light state to MQTT
 *
 * @param state true for ON, false for OFF
 * @return esp_err_t
 */
esp_err_t app_mqtt_publish_state(bool state);

/**
 * @brief Publish sensor data to MQTT
 *
 * @param temp Temperature value
 * @param hum Humidity value
 * @param presence Presence status
 * @return esp_err_t
 */
esp_err_t app_mqtt_publish_sensor_data(float temp, float hum, bool presence);
