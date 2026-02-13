/*
 * Light control: binds persisted state to the board LED implementation.
 *
 * For ESP32-S3-BOX-3 in this repo, we drive a single GPIO LED (GPIO40) using
 * the simple `app_led_*` API from app_led.c.
 */

#include "light_ctrl.h"

#include "esp_check.h"
#include "esp_log.h"

#include "app_led.h"
#include "app_mqtt.h"
#include "bsp_board.h"
#include "light_state.h"
#include "light_ui.h"

static const char *TAG = "light_ctrl";
static bool s_inited = false;

static esp_err_t init_default_led_hw(void) {
  const board_res_desc_t *brd = bsp_board_get_description();
  ESP_RETURN_ON_FALSE(brd, ESP_ERR_INVALID_STATE, TAG, "board desc not ready");

  /* Single-LED mode; LED is hard-wired to GPIO40 in app_led.c */
  return app_led_init();
}

esp_err_t light_ctrl_init(void) {
  if (s_inited) {
    return ESP_OK;
  }

  ESP_RETURN_ON_ERROR(light_state_init(), TAG, "light_state_init failed");
  ESP_RETURN_ON_ERROR(init_default_led_hw(), TAG, "init_default_led_hw failed");

  bool on = light_state_get();
  ESP_LOGI(TAG, "restore light state: %s", on ? "ON" : "OFF");
  ESP_RETURN_ON_ERROR(app_led_set_power(on), TAG, "set power failed");
  ESP_LOGI(TAG, "LED initialized to: %s", app_led_get_state() ? "ON" : "OFF");

  s_inited = true;
  return ESP_OK;
}

bool light_ctrl_get(void) { return light_state_get(); }

esp_err_t light_ctrl_set(bool on) {
  ESP_LOGI(TAG, "light_ctrl_set(%s)", on ? "ON" : "OFF");
  light_ui_set(on); // Sync UI
  ESP_RETURN_ON_ERROR(app_led_set_power(on), TAG, "set power failed");
  ESP_RETURN_ON_ERROR(light_state_set(on), TAG, "persist failed");
  ESP_LOGI(TAG, "LED HW state now: %s", app_led_get_state() ? "ON" : "OFF");

  // Notify MQTT
  app_mqtt_publish_state(on);

  return ESP_OK;
}

esp_err_t light_ctrl_toggle(void) { return light_ctrl_set(!light_ctrl_get()); }
