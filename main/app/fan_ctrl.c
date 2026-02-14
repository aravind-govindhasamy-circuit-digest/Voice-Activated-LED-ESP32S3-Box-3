#include "fan_ctrl.h"
#include "app_mqtt.h"
#include "esp_log.h"
#include "fan_ui.h"


static const char *TAG = "fan_ctrl";
static bool s_fan_on = false;
static uint8_t s_fan_speed = 50;

static void fan_ctrl_sync(bool publish_mqtt) {
  fan_ui_update(s_fan_on, s_fan_speed);
  if (publish_mqtt) {
    (void)app_mqtt_publish_fan_state(s_fan_on, s_fan_speed);
  }
}

esp_err_t fan_ctrl_init(void) {
  ESP_LOGI(TAG, "Fan control initialized");
  fan_ctrl_sync(false);
  return ESP_OK;
}

void fan_ctrl_set_power(bool on) {
  ESP_LOGI(TAG, "Fan Power: %s", on ? "ON" : "OFF");
  s_fan_on = on;
  if (s_fan_on && s_fan_speed == 0) {
    s_fan_speed = 50;
  }
  fan_ctrl_sync(true);
}

void fan_ctrl_set_speed(uint8_t speed) {
  if (speed > 100)
    speed = 100;
  ESP_LOGI(TAG, "Fan Speed: %d", speed);
  s_fan_speed = speed;
  s_fan_on = (speed > 0);
  fan_ctrl_sync(true);
}

bool fan_ctrl_get_power(void) { return s_fan_on; }

uint8_t fan_ctrl_get_speed(void) { return s_fan_speed; }
