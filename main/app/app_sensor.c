#include "app_sensor.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "app_sensor";
static SemaphoreHandle_t i2c_mux = NULL;

// Docker I2C Bus (ESP32-S3-BOX-3 Sensor Dock)
#define DOCK_I2C_SDA 41
#define DOCK_I2C_SCL 40
#define DOCK_I2C_NUM I2C_NUM_1

// Presence Sensor
#define PRESENCE_GPIO 21
// IR Power (Sometimes needed for dock power)
#define IR_POWER_GPIO 44

static uint8_t aht20_addr = 0x38;
static bool aht20_found = false;

esp_err_t app_sensor_init(void) {
  if (i2c_mux == NULL) {
    i2c_mux = xSemaphoreCreateMutex();
  }

  // 1. Presence GPIO init
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << PRESENCE_GPIO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);

  /*
    // 2. IR Power (Enable dock components)
    // Initially setting HIGH (Disabled) to rule out interference
    // WARNING: GPIO 44 is often used as Console RX. Hijacking it will break
    serial. gpio_config_t ir_pwr_conf = { .pin_bit_mask = (1ULL <<
    IR_POWER_GPIO), .mode = GPIO_MODE_OUTPUT, .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&ir_pwr_conf);
    gpio_set_level(IR_POWER_GPIO, 1); // Logical OFF (Physical HIGH)
  */

  // 3. I2C Bus 1 Initialization (for Dock)
  // We use I2C_NUM_1 to avoid conflict with internal components on I2C_NUM_0
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = DOCK_I2C_SDA,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = DOCK_I2C_SCL,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 100000,
  };

  // Clean up if already installed (safety for re-init)
  i2c_driver_delete(DOCK_I2C_NUM);

  esp_err_t err = i2c_param_config(DOCK_I2C_NUM, &conf);
  if (err != ESP_OK)
    ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));

  err = i2c_driver_install(DOCK_I2C_NUM, conf.mode, 0, 0, 0);
  if (err != ESP_OK)
    ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));

  ESP_LOGI(TAG, "✅ Sensor Dock Bus Initialized (I2C_NUM_1) on Pins 41/40");

  // Delay to let the bus settle before probing
  vTaskDelay(pdMS_TO_TICKS(100));

  // Initial probe AHT20
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (aht20_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(DOCK_I2C_NUM, cmd, pdMS_TO_TICKS(500));
  if (err == ESP_OK) {
    aht20_found = true;
    ESP_LOGI(TAG, "✅ AHT20 found at 0x38");
  } else {
    ESP_LOGE(TAG, "❌ AHT20 NOT found! Error: %d (%s)", err,
             esp_err_to_name(err));
    ESP_LOGW(TAG, "Is the Sensor Dock firmly connected to the BOX-3?");
  }
  i2c_cmd_link_delete(cmd);

  return ESP_OK;
}

esp_err_t app_sensor_get_values(float *temp, float *hum) {
  if (!aht20_found)
    return ESP_ERR_NOT_FOUND;
  if (xSemaphoreTake(i2c_mux, pdMS_TO_TICKS(500)) != pdTRUE)
    return ESP_ERR_TIMEOUT;

  // Trigger AHT20 Measurement
  uint8_t trigger[] = {0xAC, 0x33, 0x00};
  i2c_master_write_to_device(DOCK_I2C_NUM, aht20_addr, trigger, 3,
                             pdMS_TO_TICKS(100));

  vTaskDelay(pdMS_TO_TICKS(80)); // Wait for measurement

  uint8_t data[6];
  esp_err_t ret = i2c_master_read_from_device(DOCK_I2C_NUM, aht20_addr, data, 6,
                                              pdMS_TO_TICKS(100));

  if (ret == ESP_OK) {
    // AHT20 calculation
    uint32_t hum_raw =
        ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    uint32_t temp_raw =
        (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];

    if (hum)
      *hum = (float)hum_raw * 100.0f / 1048576.0f;
    if (temp)
      *temp = (float)temp_raw * 200.0f / 1048576.0f - 50.0f;
  }

  xSemaphoreGive(i2c_mux);
  return ret;
}

bool app_sensor_get_presence(void) {
  return gpio_get_level(PRESENCE_GPIO) ==
         1; // Presence is often active high on these modules
}

float app_sensor_get_lux(void) {
  // BH1750 not in the official dock repository, but keeping stub for
  // compatibility
  return 0.0f;
}
