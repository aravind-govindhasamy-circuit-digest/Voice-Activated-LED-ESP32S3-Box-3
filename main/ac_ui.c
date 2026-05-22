#include "ac_ui.h"
#include "app/ac_ctrl.h"
#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "lvgl.h"
#include "light_ui.h"
#include "app_mqtt.h"

static const char *TAG = "ac_ui";

static lv_obj_t *s_scr_ac = NULL;
static lv_obj_t *s_label_temp = NULL;
static lv_obj_t *s_btn_power = NULL;
static lv_obj_t *s_label_power = NULL;
static lv_obj_t *s_btn_mode = NULL;
static lv_obj_t *s_label_mode = NULL;

static const char *s_modes[] = {"COOL", "HEAT", "FAN", "DRY"};

static void refresh_ac_widgets(void) {
    if (!s_scr_ac) return;

    bool power = ac_ctrl_get_power();
    uint8_t temp = ac_ctrl_get_temp();
    uint8_t mode = ac_ctrl_get_mode();

    // 1. Update Temperature Label
    if (s_label_temp) {
        lv_label_set_text_fmt(s_label_temp, "%u °C", temp);
    }

    // 2. Update Power Button Style
    if (s_btn_power && s_label_power) {
        if (power) {
            lv_label_set_text(s_label_power, "AC: ON");
            lv_obj_set_style_bg_color(s_btn_power, lv_color_hex(0x2EC4B6), 0); // Vibrant teal for ON
        } else {
            lv_label_set_text(s_label_power, "AC: OFF");
            lv_obj_set_style_bg_color(s_btn_power, lv_color_hex(0xE71D36), 0); // Red for OFF
        }
    }

    // 3. Update Mode Label
    if (s_label_mode) {
        lv_label_set_text_fmt(s_label_mode, "Mode: %s", s_modes[mode]);
    }
}

static void on_power_click(lv_event_t *e) {
    (void)e;
    bool new_power = !ac_ctrl_get_power();
    ac_ctrl_set_power(new_power);
    refresh_ac_widgets();

    // Publish state to MQTT
    app_mqtt_publish_sensor_data(0, 0, 0); // Triggers overall telemetry publish
}

static void on_mode_click(lv_event_t *e) {
    (void)e;
    uint8_t next_mode = (ac_ctrl_get_mode() + 1) % 4;
    ac_ctrl_set_mode(next_mode);
    refresh_ac_widgets();

    app_mqtt_publish_sensor_data(0, 0, 0);
}

static void on_temp_up(lv_event_t *e) {
    (void)e;
    uint8_t current_temp = ac_ctrl_get_temp();
    if (current_temp < 30) {
        ac_ctrl_set_temp(current_temp + 1);
        refresh_ac_widgets();
        app_mqtt_publish_sensor_data(0, 0, 0);
    }
}

static void on_temp_down(lv_event_t *e) {
    (void)e;
    uint8_t current_temp = ac_ctrl_get_temp();
    if (current_temp > 16) {
        ac_ctrl_set_temp(current_temp - 1);
        refresh_ac_widgets();
        app_mqtt_publish_sensor_data(0, 0, 0);
    }
}

static void on_back_click(lv_event_t *e) {
    (void)e;
    light_ui_show_status_screen();
}

esp_err_t ac_ui_start(void) {
    ESP_LOGI(TAG, "Starting AC UI panel");
    bsp_display_lock(0);

    if (s_scr_ac) {
        bsp_display_unlock();
        return ESP_OK;
    }

    s_scr_ac = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_scr_ac, lv_color_hex(0x111B24), 0); // Beautiful deep navy-black dark mode
    lv_obj_clear_flag(s_scr_ac, LV_OBJ_FLAG_SCROLLABLE);

    // Title Label
    lv_obj_t *title = lv_label_create(s_scr_ac);
    lv_label_set_text(title, "AC Climate Control");
    lv_obj_set_style_text_color(title, lv_color_hex(0xE0FBFC), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    // Temperature Container/Display
    s_label_temp = lv_label_create(s_scr_ac);
    lv_label_set_text(s_label_temp, "24 °C");
    lv_obj_set_style_text_color(s_label_temp, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(s_label_temp, &lv_font_montserrat_32, 0);
    lv_obj_align(s_label_temp, LV_ALIGN_CENTER, 0, -25);

    // Minus (-) Button
    lv_obj_t *btn_down = lv_btn_create(s_scr_ac);
    lv_obj_set_size(btn_down, 55, 55);
    lv_obj_align(btn_down, LV_ALIGN_CENTER, -85, -25);
    lv_obj_set_style_radius(btn_down, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(btn_down, lv_color_hex(0x3D5A80), 0);
    lv_obj_add_event_cb(btn_down, on_temp_down, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_down = lv_label_create(btn_down);
    lv_label_set_text(label_down, LV_SYMBOL_MINUS);
    lv_obj_center(label_down);

    // Plus (+) Button
    lv_obj_t *btn_up = lv_btn_create(s_scr_ac);
    lv_obj_set_size(btn_up, 55, 55);
    lv_obj_align(btn_up, LV_ALIGN_CENTER, 85, -25);
    lv_obj_set_style_radius(btn_up, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(btn_up, lv_color_hex(0xEE6C4D), 0); // Warm red-orange for hot/plus
    lv_obj_add_event_cb(btn_up, on_temp_up, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_up = lv_label_create(btn_up);
    lv_label_set_text(label_up, LV_SYMBOL_PLUS);
    lv_obj_center(label_up);

    // Power Toggle Button
    s_btn_power = lv_btn_create(s_scr_ac);
    lv_obj_set_size(s_btn_power, 125, 45);
    lv_obj_align(s_btn_power, LV_ALIGN_BOTTOM_LEFT, 20, -75);
    lv_obj_add_event_cb(s_btn_power, on_power_click, LV_EVENT_CLICKED, NULL);
    s_label_power = lv_label_create(s_btn_power);
    lv_label_set_text(s_label_power, "AC: OFF");
    lv_obj_center(s_label_power);

    // Mode Selector Button
    s_btn_mode = lv_btn_create(s_scr_ac);
    lv_obj_set_size(s_btn_mode, 125, 45);
    lv_obj_align(s_btn_mode, LV_ALIGN_BOTTOM_RIGHT, -20, -75);
    lv_obj_add_event_cb(s_btn_mode, on_mode_click, LV_EVENT_CLICKED, NULL);
    s_label_mode = lv_label_create(s_btn_mode);
    lv_label_set_text(s_label_mode, "Mode: COOL");
    lv_obj_set_style_bg_color(s_btn_mode, lv_color_hex(0x293241), 0);
    lv_obj_center(s_label_mode);

    // Back Button
    lv_obj_t *btn_back = lv_btn_create(s_scr_ac);
    lv_obj_set_size(btn_back, 85, 38);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_MID, 0, -15);
    lv_obj_set_style_bg_color(btn_back, lv_color_hex(0x334155), 0);
    lv_obj_add_event_cb(btn_back, on_back_click, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_back = lv_label_create(btn_back);
    lv_label_set_text(label_back, "Back");
    lv_obj_center(label_back);

    refresh_ac_widgets();

    bsp_display_unlock();
    return ESP_OK;
}

void ac_ui_show_screen(void) {
    bsp_display_lock(0);
    if (s_scr_ac) {
        refresh_ac_widgets();
        lv_scr_load(s_scr_ac);
    }
    bsp_display_unlock();
}

void ac_ui_update(bool power, uint8_t temp, uint8_t mode) {
    bsp_display_lock(0);
    refresh_ac_widgets();
    bsp_display_unlock();
}
