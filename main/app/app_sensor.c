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

// Radar Sensor
#define RADAR_GPIO (GPIO_NUM_21)

// I2C Pins for ESP32-S3-BOX-3 Sensor Dock (Expansion Port)
// I2C Addresses
#define AHT20_ADDR 0x38
#define SHT20_ADDR 0x40
#define ICM42607_ADDR 0x68

static bool aht20_found = false;
static bool sht20_found = false;
static bool icm42607_found = false;
static i2c_port_t SENSOR_I2C_NUM = I2C_NUM_1; 

#define SENSOR_I2C_FREQ  20000 // Very slow for long expansion cables
#define ICM42607_WHO_AM_I       0x75
#define ICM42607_PWR_MGMT0      0x1F
#define ICM42607_ACCEL_CONFIG0  0x21
#define ICM42607_GYRO_CONFIG0   0x20
#define ICM42607_ACCEL_DATA_X1  0x0B
#define ICM42607_GYRO_DATA_X1   0x11

static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t data[] = {reg, val};
    return i2c_master_write_to_device(SENSOR_I2C_NUM, addr, data, sizeof(data), pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_write_read_device(SENSOR_I2C_NUM, addr, &reg, 1, buf, len, pdMS_TO_TICKS(100));
}

esp_err_t app_sensor_init(void) {
    if (i2c_mux == NULL) {
        i2c_mux = xSemaphoreCreateMutex();
    }

    // 1. Radar GPIO init
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RADAR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // 2. Deep Clean I2C Port 1
    // Delete existing driver if BSP or previous init left it in a weird state
    i2c_driver_delete(SENSOR_I2C_NUM);
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (GPIO_NUM_41),
        .scl_io_num = (GPIO_NUM_40),
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = SENSOR_I2C_FREQ,
    };
    i2c_param_config(SENSOR_I2C_NUM, &conf);
    i2c_driver_install(SENSOR_I2C_NUM, conf.mode, 0, 0, 0);

    // Maximize drive strength
    gpio_set_drive_capability(GPIO_NUM_41, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(GPIO_NUM_40, GPIO_DRIVE_CAP_3);
    
    vTaskDelay(pdMS_TO_TICKS(200));

    // 3. Auto-detect Bus (Try Port 1 then Port 0)
    i2c_port_t ports[] = {I2C_NUM_1, I2C_NUM_0};
    bool found_bus = false;

    for (int p = 0; p < 2; p++) {
        i2c_port_t current_port = ports[p];
        uint8_t dummy = 0;
        
        ESP_LOGI(TAG, "Probing Bus %d for sensors...", current_port);
        // Check for 0x40 (SHT) or 0x38 (AHT) or 0x18 (Alt AHT)
        if (i2c_master_read_from_device(current_port, 0x40, &dummy, 1, pdMS_TO_TICKS(50)) == ESP_OK ||
            i2c_master_read_from_device(current_port, 0x38, &dummy, 1, pdMS_TO_TICKS(50)) == ESP_OK ||
            i2c_master_read_from_device(current_port, 0x18, &dummy, 1, pdMS_TO_TICKS(50)) == ESP_OK) {
            
            SENSOR_I2C_NUM = current_port;
            found_bus = true;
            ESP_LOGI(TAG, "✅ Sensors detected on Bus %d", SENSOR_I2C_NUM);
            break;
        }
    }

    if (!found_bus) {
        ESP_LOGW(TAG, "⚠️ No sensors detected during bus probe, defaulting to Port 1");
        SENSOR_I2C_NUM = I2C_NUM_1;
    }

    // ── AHT2x Initialization (Try 0x38 and 0x18) ──
    uint8_t aht_addrs[] = {AHT20_ADDR, 0x18};
    for (int i = 0; i < 2; i++) {
        uint8_t addr = aht_addrs[i];
        uint8_t aht_reset = 0xBA;
        if (i2c_master_write_to_device(SENSOR_I2C_NUM, addr, &aht_reset, 1, pdMS_TO_TICKS(100)) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(50));
            uint8_t aht_calib[3] = {0xBE, 0x08, 0x00};
            if (i2c_master_write_to_device(SENSOR_I2C_NUM, addr, aht_calib, 3, pdMS_TO_TICKS(100)) == ESP_OK) {
                aht20_found = true;
                ESP_LOGI(TAG, "✅ AHT sensor initialized at 0x%02X", addr);
                break;
            }
        }
    }

    // ── SHT20 Initialization ──
    uint8_t sht_reset = 0xFE; // Soft Reset
    if (i2c_master_write_to_device(SENSOR_I2C_NUM, SHT20_ADDR, &sht_reset, 1, pdMS_TO_TICKS(100)) == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(50));
        sht20_found = true;
        ESP_LOGI(TAG, "✅ SHT20/HTU21D initialized at 0x40");
    }

    // Probe and Init ICM-42607-P
    uint8_t whoami = 0;
    if (i2c_read_regs(ICM42607_ADDR, ICM42607_WHO_AM_I, &whoami, 1) == ESP_OK) {
        icm42607_found = true;
        ESP_LOGI(TAG, "✅ ICM-42607-P found on Bus %d, WHO_AM_I=0x%02X", SENSOR_I2C_NUM, whoami);
        i2c_write_reg(ICM42607_ADDR, ICM42607_PWR_MGMT0, 0x0F); 
    }

    return ESP_OK;
}

esp_err_t app_sensor_get_values(float *temp, float *hum) {
    if (xSemaphoreTake(i2c_mux, pdMS_TO_TICKS(1000)) != pdTRUE) return ESP_ERR_TIMEOUT;

    esp_err_t ret = ESP_ERR_NOT_FOUND;
    uint8_t data[6];

    // --- Try AHT2x (0x38 or 0x18) ---
    uint8_t aht_addrs[] = {AHT20_ADDR, 0x18};
    for (int i = 0; i < 2; i++) {
        uint8_t addr = aht_addrs[i];
        uint8_t trigger[] = {0xAC, 0x33, 0x00};
        
        // Simple Write then Read (No repeated start - better for AHT)
        if (i2c_master_write_to_device(SENSOR_I2C_NUM, addr, trigger, 3, pdMS_TO_TICKS(100)) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(100)); 
            if (i2c_master_read_from_device(SENSOR_I2C_NUM, addr, data, 6, pdMS_TO_TICKS(100)) == ESP_OK) {
                if (data[0] != 0xFF && data[0] != 0x00) {
                    uint32_t hum_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
                    uint32_t temp_raw = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
                    if (hum) *hum = (float)hum_raw * 100.0f / 1048576.0f;
                    if (temp) *temp = (float)temp_raw * 200.0f / 1048576.0f - 50.0f;
                    ret = ESP_OK;
                    break;
                }
            }
        }
    }
    
    // --- Try SHT20 (0x40) ---
    if (ret != ESP_OK && sht20_found) {
        uint8_t cmd_temp = 0xE3; // Hold Master
        uint8_t cmd_hum = 0xE5;  // Hold Master
        
        // Atomic Read for Temp
        esp_err_t e_ret = i2c_master_write_read_device(SENSOR_I2C_NUM, SHT20_ADDR, &cmd_temp, 1, data, 3, pdMS_TO_TICKS(500));
        if (e_ret == ESP_OK) {
            uint16_t raw_temp = (data[0] << 8) | data[1];
            ESP_LOGI(TAG, "SHT20 Read OK: 0x%04X", raw_temp);
            if (raw_temp != 0xFFFF && raw_temp != 0x0000) {
                raw_temp &= ~0x0003; 
                if (temp) *temp = -46.85f + 175.72f * (float)raw_temp / 65536.0f;
                ret = ESP_OK;
            }
        }
        
        // Atomic Read for Hum
        if (i2c_master_write_read_device(SENSOR_I2C_NUM, SHT20_ADDR, &cmd_hum, 1, data, 3, pdMS_TO_TICKS(500)) == ESP_OK) {
            uint16_t raw_hum = (data[0] << 8) | data[1];
            if (raw_hum != 0xFFFF && raw_hum != 0x0000) {
                raw_hum &= ~0x0003; 
                if (hum) *hum = -6.0f + 125.0f * (float)raw_hum / 65536.0f;
                ret = ESP_OK;
            }
        }
    }

    xSemaphoreGive(i2c_mux);
    return ret;
}


esp_err_t app_sensor_get_imu(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    if (!icm42607_found) return ESP_ERR_NOT_FOUND;
    if (xSemaphoreTake(i2c_mux, pdMS_TO_TICKS(500)) != pdTRUE) return ESP_ERR_TIMEOUT;

    uint8_t buf[6];
    if (i2c_read_regs(ICM42607_ADDR, ICM42607_ACCEL_DATA_X1, buf, 6) == ESP_OK) {
        if (ax) *ax = (int16_t)((buf[0] << 8) | buf[1]) / 8192.0f;
        if (ay) *ay = (int16_t)((buf[2] << 8) | buf[3]) / 8192.0f;
        if (az) *az = (int16_t)((buf[4] << 8) | buf[5]) / 8192.0f;
    }

    if (i2c_read_regs(ICM42607_ADDR, ICM42607_GYRO_DATA_X1, buf, 6) == ESP_OK) {
        if (gx) *gx = (int16_t)((buf[0] << 8) | buf[1]) / 65.5f;
        if (gy) *gy = (int16_t)((buf[2] << 8) | buf[3]) / 65.5f;
        if (gz) *gz = (int16_t)((buf[4] << 8) | buf[5]) / 65.5f;
    }

    xSemaphoreGive(i2c_mux);
    return ESP_OK;
}

bool app_sensor_get_presence(void) {
    return gpio_get_level(RADAR_GPIO) == 1; 
}

float app_sensor_get_lux(void) {
    return 0.0f;
}

