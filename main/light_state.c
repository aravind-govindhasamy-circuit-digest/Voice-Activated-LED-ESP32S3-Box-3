/*
 * Minimal light state (latched) stored in NVS.
 */

#include "light_state.h"

#include "esp_check.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "light_state";

static bool s_inited = false;
static bool s_on = false;

esp_err_t light_state_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase failed");
        ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "nvs init failed after erase");
    } else {
        ESP_RETURN_ON_ERROR(err, TAG, "nvs init failed");
    }

    nvs_handle_t h = 0;
    ESP_RETURN_ON_ERROR(nvs_open("light", NVS_READWRITE, &h), TAG, "nvs_open failed");

    uint8_t v = 0;
    err = nvs_get_u8(h, "on", &v);
    if (err == ESP_OK) {
        s_on = (v != 0);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        s_on = false;
        (void)nvs_set_u8(h, "on", 0);
        (void)nvs_commit(h);
        err = ESP_OK;
    }

    nvs_close(h);
    ESP_RETURN_ON_ERROR(err, TAG, "nvs_get_u8 failed");

    s_inited = true;
    return ESP_OK;
}

bool light_state_get(void)
{
    return s_on;
}

esp_err_t light_state_set(bool on)
{
    s_on = on;

    nvs_handle_t h = 0;
    ESP_RETURN_ON_ERROR(nvs_open("light", NVS_READWRITE, &h), TAG, "nvs_open failed");
    esp_err_t err = nvs_set_u8(h, "on", on ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    ESP_RETURN_ON_ERROR(err, TAG, "nvs write failed");

    return ESP_OK;
}

