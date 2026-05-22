#include "fan_ui.h"

#include "app_mqtt.h"
#include "app_sensor.h"
#include "app_status.h"
#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "fan_ctrl.h"
#include "light_ctrl.h"
#include "lvgl.h"
#include "machi_images.h"
#include "light_ui.h"

#include <stdint.h>
#include <stdio.h>

static const char *TAG = "machi_ui";

static lv_obj_t *s_scr_main = NULL;
static lv_obj_t *s_face_img = NULL;
static lv_obj_t *s_label_sensor = NULL;
static lv_obj_t *s_title = NULL;

static lv_timer_t *s_status_timer = NULL;
static lv_timer_t *s_blink_timer = NULL;

static const lv_img_dsc_t *default_emotion = &img_eyes_neutral;
static uint32_t emotion_lock_time = 0;

static void on_touch_cb(lv_event_t *e) {
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    // Provide touch emotion according to side of screen!
    if (p.x < 160) {
        lv_img_set_src(s_face_img, &img_touch_left);
    } else {
        lv_img_set_src(s_face_img, &img_touch_right);
    }
    
    // Lock this emotion for 2 seconds
    emotion_lock_time = lv_tick_get() + 2000;
}

static void restore_blink_cb(lv_timer_t *t) {
    lv_timer_del(t);
    // Only restore if not currently locked by touch interactions
    if (lv_tick_get() >= emotion_lock_time) {
        lv_img_set_src(s_face_img, default_emotion);
    }
}

static void blink_timer_cb(lv_timer_t *t) {
    (void)t;
    // Skip blink if user is interacting
    if (lv_tick_get() < emotion_lock_time) return;

    lv_img_set_src(s_face_img, &img_eyes_blink);
    lv_timer_create(restore_blink_cb, 150, NULL);
}

static void format_sensor_line(char *out, size_t out_len, float temp, float hum, bool presence) {
    snprintf(out, out_len, "Temp: %.1f C  Hum: %.1f %%\nPresence: %s", 
             temp, hum, presence ? "Detected" : "None");
}

static void refresh_widgets(void) {
    app_status_t st = app_status_get();
    
    if (s_label_sensor) {
        if (st.sensor_valid) {
            char line[96];
            format_sensor_line(line, sizeof(line), st.last_temp, st.last_hum, st.last_presence);
            lv_label_set_text(s_label_sensor, line);
            lv_obj_align(s_label_sensor, LV_ALIGN_BOTTOM_MID, 0, -10);
            
            // Logic for default ambient emotions
            if (st.last_presence) {
                default_emotion = &img_eyes_happy;
                lv_obj_set_style_bg_color(s_scr_main, lv_color_hex(0x1B2A1E), 0);
            } else if (st.last_temp > 28.0f) {
                default_emotion = &img_eyes_dizzy; // Hot
                lv_obj_set_style_bg_color(s_scr_main, lv_color_hex(0x3B1F2A), 0);
            } else if (st.last_hum > 60.0f) {
                default_emotion = &img_eyes_sad; // Muggy
            } else {
                default_emotion = &img_eyes_neutral;
                lv_obj_set_style_bg_color(s_scr_main, lv_color_hex(0x1B1F2A), 0);
            }
        } else {
            lv_label_set_text(s_label_sensor, "Sensor Dock: Waiting...");
            lv_obj_align(s_label_sensor, LV_ALIGN_BOTTOM_MID, 0, -10);
        }
    }
}

static void status_timer_cb(lv_timer_t *timer) {
    (void)timer;
    refresh_widgets();

    // Revert visual face if touch lock has expired
    if (lv_tick_get() >= emotion_lock_time) {
        lv_img_set_src(s_face_img, default_emotion);
    }
}

static void on_back_click(lv_event_t *e) {
    (void)e;
    light_ui_show_status_screen();
}

static void create_machi_screen(void) {
    s_scr_main = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_scr_main, lv_color_hex(0x1B1F2A), 0);
    lv_obj_clear_flag(s_scr_main, LV_OBJ_FLAG_SCROLLABLE);

    // Ensure we can click the background for touch events!
    lv_obj_add_flag(s_scr_main, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_scr_main, on_touch_cb, LV_EVENT_CLICKED, NULL);

    // Back Button to exit Machi screen
    lv_obj_t *btn_back = lv_btn_create(s_scr_main);
    lv_obj_set_size(btn_back, 70, 35);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_hex(0x334155), 0);
    lv_obj_add_event_cb(btn_back, on_back_click, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_back = lv_label_create(btn_back);
    lv_label_set_text(label_back, "Back");
    lv_obj_center(label_back);

    // Title
    s_title = lv_label_create(s_scr_main);
    lv_label_set_text(s_title, "Machi Buddy");
    lv_obj_set_style_text_color(s_title, lv_color_hex(0xCBD6E2), 0);
    lv_obj_align(s_title, LV_ALIGN_TOP_MID, 0, 10);

    // Image for eyes/face
    s_face_img = lv_img_create(s_scr_main);
    lv_img_set_src(s_face_img, default_emotion);
    lv_obj_align(s_face_img, LV_ALIGN_CENTER, 0, -15);
    // Make sure click events pass through the image to the screen
    lv_obj_add_flag(s_face_img, LV_OBJ_FLAG_EVENT_BUBBLE);

    // Sensor Label
    s_label_sensor = lv_label_create(s_scr_main);
    lv_label_set_text(s_label_sensor, "Sensor Dock: Waiting...");
    lv_obj_set_style_text_color(s_label_sensor, lv_color_hex(0x8EE0F5), 0);
    lv_obj_set_style_text_align(s_label_sensor, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_label_sensor, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    // Blinking timer every ~3.5 seconds
    s_blink_timer = lv_timer_create(blink_timer_cb, 3500, NULL);
}

esp_err_t fan_ui_start(void) {
    ESP_LOGI(TAG, "Starting Machi screen");
    bsp_display_lock(0);

    if (s_scr_main) {
        bsp_display_unlock();
        return ESP_OK;
    }

    create_machi_screen();
    // lv_scr_load(s_scr_main); // Do not load Machi screen on startup

    if (!s_status_timer) {
        s_status_timer = lv_timer_create(status_timer_cb, 1000, NULL);
    }
    refresh_widgets();

    bsp_display_unlock();
    return ESP_OK;
}

void fan_ui_show_fan_screen(void) {
    bsp_display_lock(0);
    if (s_scr_main) {
        lv_scr_load(s_scr_main);
    }
    bsp_display_unlock();
}

void fan_ui_show_sensor_screen(void) {
    bsp_display_lock(0);
    if (s_scr_main) {
        lv_scr_load(s_scr_main);
    }
    bsp_display_unlock();
}

void fan_ui_update(bool on, uint8_t speed) {
    (void)on;
    (void)speed;
    bsp_display_lock(0);
    refresh_widgets();
    bsp_display_unlock();
}

