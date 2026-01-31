/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "app_led.h"
#include "driver/gpio.h"

static const char *TAG = "app_led";

/* Single LED output pin (common cathode LED with resistor to GND) */
#define SINGLE_LED_GPIO GPIO_NUM_40

static bool g_led_on = false;
static bool g_initialized = false;

esp_err_t app_led_init(void)
{
    ESP_LOGI(TAG, "Initializing single LED on GPIO%d", (int)SINGLE_LED_GPIO);

    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(SINGLE_LED_GPIO));
    esp_err_t ret = gpio_set_direction(SINGLE_LED_GPIO, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_set_direction failed: %d", (int)ret);
        return ret;
    }

    /* Default OFF */
    gpio_set_level(SINGLE_LED_GPIO, 0);
    g_led_on = false;
    g_initialized = true;

    return ESP_OK;
}

esp_err_t app_led_deinit(void)
{
    /* Optionally turn off LED and mark uninitialized */
    if (g_initialized) {
        gpio_set_level(SINGLE_LED_GPIO, 0);
        g_initialized = 0;
    }
    return ESP_OK;
}

static void update_pwm_led(uint8_t r, uint8_t g, uint8_t b)
{
    (void)r;
    (void)g;
    (void)b;
    if (!g_initialized) {
        return;
    }
    gpio_set_level(SINGLE_LED_GPIO, g_led_on ? 1 : 0);
}

esp_err_t app_led_set_power(bool power)
{
    if (!g_initialized) {
        ESP_LOGW(TAG, "app_led_set_power called before init");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "app_led_set_power(%s)", power ? "ON" : "OFF");
    g_led_on = power;
    update_pwm_led(0, 0, 0);
    return ESP_OK;
}

bool app_led_get_state(void)
{
    return g_led_on;
}
