/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp/esp-bsp.h"
#include "bsp_board.h"
#include "bsp_storage.h"

#include "app/fan_ctrl.h"
#include "app_mqtt.h"
#include "app_sensor.h"
#include "app_status.h"
#include "app_sr.h"
#include "app_wifi.h"
#include "fan_ui.h"
#include "gui/ui_boot_animate.h"
#include "gui/ui_sr.h"
#include "light_ctrl.h"
#include "light_ui.h"
#include "nvs_flash.h"
#include <math.h>

static const char *TAG = "main";

static void sensor_task(void *arg) {
  float temp = 0, hum = 0;
  bool presence = false;
  float last_temp = 0, last_hum = 0;
  bool last_presence = false;
  uint32_t last_publish_time = 0;

  while (1) {
    if (app_sensor_get_values(&temp, &hum) == ESP_OK) {
      presence = app_sensor_get_presence();
      app_status_update_sensor(temp, hum, presence);
      uint32_t now = xTaskGetTickCount();

      // Check for significant changes
      bool changed = false;
      if (presence != last_presence) {
        changed = true;
      } else if (fabs(temp - last_temp) > 0.5f) {
        changed = true;
      } else if (fabs(hum - last_hum) > 2.0f) {
        changed = true;
      }

      // Heartbeat: Publish at least every 60 seconds regardless of change
      bool heartbeat = (now - last_publish_time) > pdMS_TO_TICKS(60000);

      if (changed || heartbeat) {
        app_mqtt_publish_sensor_data(temp, hum, presence);
        last_temp = temp;
        last_hum = hum;
        last_presence = presence;
        last_publish_time = now;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Poll every second for responsiveness
  }
}

static void after_boot(void) {
  /* Minimal main screen */
  ESP_ERROR_CHECK(light_ui_start());
  ESP_ERROR_CHECK(fan_ui_start());
  fan_ui_update(fan_ctrl_get_power(), fan_ctrl_get_speed());
  ui_sr_anim_init();
}

void app_main(void) {
  ESP_LOGI(TAG, "Compile time: %s %s", __DATE__, __TIME__);

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  /* Needed for SR echo wav files */
  bsp_spiffs_mount();

  bsp_i2c_init();

  bsp_display_cfg_t cfg = {.lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
                           .buffer_size =
                               BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT,
                           .double_buffer = 0,
                           .flags = {
                               .buff_dma = true,
                           }};
  cfg.lvgl_port_cfg.task_affinity = 1;
  bsp_display_start_with_config(&cfg);
  bsp_board_init();

  ESP_LOGI(TAG, "start light demo UI");
  ESP_ERROR_CHECK(light_ctrl_init());
  ESP_ERROR_CHECK(fan_ctrl_init());

  // Sensor initialized later

  bsp_display_lock(0);
  boot_animate_start(after_boot);
  bsp_display_unlock();

  /* SR detect can saturate one core during DSP windows; monitor core0 idle and
   * rely on per-task WDT resets for SR tasks. */
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 20000,
      .idle_core_mask = (1U << 0),
      .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&wdt_config);

  vTaskDelay(pdMS_TO_TICKS(500));
  bsp_display_backlight_on();

  vTaskDelay(pdMS_TO_TICKS(1500));
  ESP_ERROR_CHECK(app_sr_start(false));

  // Initialize Sensor Dock after SR is running
  ESP_LOGI(TAG, "Initialising Sensor Dock");
  app_sensor_init();

  /* WiFi and MQTT */
  if (app_wifi_init() == ESP_OK) {
    app_mqtt_init();
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
  }
}
