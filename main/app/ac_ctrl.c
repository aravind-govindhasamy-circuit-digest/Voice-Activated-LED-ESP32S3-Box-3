#include "ac_ctrl.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "ac_ctrl";
static bool s_ac_power = false;
static uint8_t s_ac_temp = 24;
static uint8_t s_ac_mode = 0; // 0=Cool, 1=Heat, 2=Fan, 3=Dry

esp_err_t ac_ctrl_init(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        uint8_t power = 0;
        nvs_get_u8(my_handle, "ac_power", &power);
        s_ac_power = (power != 0);

        nvs_get_u8(my_handle, "ac_temp", &s_ac_temp);
        if (s_ac_temp < 16 || s_ac_temp > 30) {
            s_ac_temp = 24;
        }

        nvs_get_u8(my_handle, "ac_mode", &s_ac_mode);
        if (s_ac_mode > 3) {
            s_ac_mode = 0;
        }

        nvs_close(my_handle);
    }
    ESP_LOGI(TAG, "AC Initialized: Power=%s, Temp=%u, Mode=%u", s_ac_power ? "ON" : "OFF", s_ac_temp, s_ac_mode);
    return ESP_OK;
}

esp_err_t ac_ctrl_set_power(bool on) {
    s_ac_power = on;
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_u8(my_handle, "ac_power", on ? 1 : 0);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
    ESP_LOGI(TAG, "AC Power Set: %s", on ? "ON" : "OFF");
    return ESP_OK;
}

bool ac_ctrl_get_power(void) {
    return s_ac_power;
}

esp_err_t ac_ctrl_set_temp(uint8_t temp) {
    if (temp < 16) temp = 16;
    if (temp > 30) temp = 30;
    s_ac_temp = temp;
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_u8(my_handle, "ac_temp", temp);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
    ESP_LOGI(TAG, "AC Temperature Set: %u", temp);
    return ESP_OK;
}

uint8_t ac_ctrl_get_temp(void) {
    return s_ac_temp;
}

esp_err_t ac_ctrl_set_mode(uint8_t mode) {
    if (mode > 3) mode = 0;
    s_ac_mode = mode;
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_u8(my_handle, "ac_mode", mode);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
    ESP_LOGI(TAG, "AC Mode Set: %u", mode);
    return ESP_OK;
}

uint8_t ac_ctrl_get_mode(void) {
    return s_ac_mode;
}
