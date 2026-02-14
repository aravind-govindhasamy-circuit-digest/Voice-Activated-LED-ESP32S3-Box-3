#include "fan_ctrl.h"
#include "app_mqtt.h"
#include "esp_log.h"


static const char *TAG = "fan_ctrl";
static bool s_fan_on = false;
static uint8_t s_fan_speed = 50;

esp_err_t fan_ctrl_init(void) {
  ESP_LOGI(TAG, "Fan control initialized");
  return ESP_OK;
}

void fan_ctrl_set_power(bool on) {
  ESP_LOGI(TAG, "Fan Power: %s", on ? "ON" : "OFF");
  s_fan_on = on;
  // Notify UI or other components if needed
}

void fan_ctrl_set_speed(uint8_t speed) {
  if (speed > 100)
    speed = 100;
  ESP_LOGI(TAG, "Fan Speed: %d", speed);
  s_fan_speed = speed;
  if (speed > 0)
    s_fan_on = true;
  else
    s_fan_on = false;
}

bool fan_ctrl_get_power(void) { return s_fan_on; }

uint8_t fan_ctrl_get_speed(void) { return s_fan_speed; }
