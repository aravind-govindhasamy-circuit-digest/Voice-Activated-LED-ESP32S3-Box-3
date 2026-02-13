/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp/esp-bsp.h"
#include "bsp_board.h"
#include "bsp_storage.h"


#include "app_mqtt.h"
#include "app_sr.h"
#include "app_wifi.h"
#include "gui/ui_boot_animate.h"
#include "gui/ui_sr.h"
#include "light_ctrl.h"
#include "light_ui.h"
#include "nvs_flash.h"


static const char *TAG = "main";

static void after_boot(void) {
  /* Minimal main screen + SR overlay */
  ESP_ERROR_CHECK(light_ui_start());
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
  bsp_display_lock(0);
  boot_animate_start(after_boot);
  bsp_display_unlock();

  vTaskDelay(pdMS_TO_TICKS(500));
  bsp_display_backlight_on();

  ESP_LOGI(TAG, "speech recognition start (english, 2 cmds)");
  vTaskDelay(pdMS_TO_TICKS(1500));
  ESP_ERROR_CHECK(app_sr_start(false));

  /* WiFi and MQTT */
  if (app_wifi_init() == ESP_OK) {
    app_mqtt_init();
  }
}
