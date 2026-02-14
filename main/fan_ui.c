#include "fan_ui.h"
#include "app_mqtt.h"
#include "app_sensor.h"
#include "app_status.h"
#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "fan_ctrl.h"
#include "light_ctrl.h"
#include "lvgl.h"

static const char *TAG = "fan_ui";
static lv_obj_t *s_fan_cont = NULL;
static lv_obj_t *s_blades[3];
static lv_obj_t *s_label_state = NULL;
static lv_anim_t s_anim;
static bool s_anim_running = false;
static lv_obj_t *s_mqtt_card = NULL;
static lv_obj_t *s_label_link = NULL;
static lv_obj_t *s_label_sensor = NULL;
static lv_obj_t *s_btn_fan = NULL;
static lv_obj_t *s_slider_speed = NULL;
static lv_obj_t *s_label_speed = NULL;
static lv_timer_t *s_status_timer = NULL;
static bool s_slider_sync = false;

static void rotate_cb(void *var, int32_t v) {
  lv_obj_t *cont = (lv_obj_t *)var;
  lv_obj_set_style_transform_angle(cont, v, 0);
}

static void update_fan_btn_label(void) {
  if (!s_btn_fan) {
    return;
  }
  lv_obj_t *label = lv_obj_get_child(s_btn_fan, 0);
  if (!label) {
    return;
  }
  lv_label_set_text(label, fan_ctrl_get_power() ? "Fan OFF" : "Fan ON");
}

static void mqtt_status_timer_cb(lv_timer_t *timer) {
  (void)timer;
  app_status_t st = app_status_get();

  if (s_label_link) {
    lv_label_set_text_fmt(
        s_label_link,
        "WiFi: %s\nMQTT: %s\nTX:%u RX:%u",
        st.wifi_connected ? "UP" : "DOWN", st.mqtt_connected ? "UP" : "DOWN",
        (unsigned)st.mqtt_tx_count, (unsigned)st.mqtt_rx_count);
  }

  if (s_label_sensor) {
    if (st.sensor_valid) {
      int32_t t10 = (int32_t)(st.last_temp * 10.0f);
      int32_t h10 = (int32_t)(st.last_hum * 10.0f);
      int32_t t_abs = (t10 < 0) ? -t10 : t10;
      int32_t h_abs = (h10 < 0) ? -h10 : h10;
      lv_label_set_text_fmt(s_label_sensor,
                            "T:%s%ld.%ld C  H:%ld.%ld%%\nPresence: %s",
                            (t10 < 0) ? "-" : "", (long)(t_abs / 10),
                            (long)(t_abs % 10), (long)(h_abs / 10),
                            (long)(h_abs % 10),
                            st.last_presence ? "ON" : "OFF");
    } else {
      lv_label_set_text(s_label_sensor, "Sensor: waiting...");
    }
  }

  if (s_slider_speed) {
    uint8_t speed = fan_ctrl_get_speed();
    if (lv_slider_get_value(s_slider_speed) != speed) {
      s_slider_sync = true;
      lv_slider_set_value(s_slider_speed, speed, LV_ANIM_OFF);
      s_slider_sync = false;
    }
  }

  if (s_label_speed) {
    lv_label_set_text_fmt(s_label_speed, "Speed: %u%%",
                          (unsigned)fan_ctrl_get_speed());
  }

  update_fan_btn_label();
}

static void fan_toggle_cb(lv_event_t *e) {
  (void)e;
  fan_ctrl_set_power(!fan_ctrl_get_power());
}

static void fan_speed_cb(lv_event_t *e) {
  if (s_slider_sync || e == NULL) {
    return;
  }
  lv_obj_t *slider = lv_event_get_target(e);
  fan_ctrl_set_speed((uint8_t)lv_slider_get_value(slider));
}

static void mqtt_sync_cb(lv_event_t *e) {
  (void)e;

  float t = 0;
  float h = 0;
  bool presence = app_sensor_get_presence();

  if (app_sensor_get_values(&t, &h) == ESP_OK) {
    app_status_update_sensor(t, h, presence);
    (void)app_mqtt_publish_sensor_data(t, h, presence);
  }

  (void)app_mqtt_publish_state(light_ctrl_get());
  (void)app_mqtt_publish_fan_state(fan_ctrl_get_power(), fan_ctrl_get_speed());
  ESP_LOGI(TAG, "Manual MQTT sync sent");
}

static void create_mqtt_card(lv_obj_t *scr) {
  s_mqtt_card = lv_obj_create(scr);
  lv_obj_set_size(s_mqtt_card, 170, 178);
  lv_obj_align(s_mqtt_card, LV_ALIGN_TOP_LEFT, 8, 8);
  lv_obj_set_style_bg_color(s_mqtt_card, lv_color_hex(0x1D2732), 0);
  lv_obj_set_style_bg_opa(s_mqtt_card, LV_OPA_70, 0);
  lv_obj_set_style_border_color(s_mqtt_card, lv_color_hex(0x59C1FF), 0);
  lv_obj_set_style_border_width(s_mqtt_card, 1, 0);
  lv_obj_set_style_radius(s_mqtt_card, 12, 0);
  lv_obj_set_style_pad_all(s_mqtt_card, 8, 0);
  lv_obj_clear_flag(s_mqtt_card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *title = lv_label_create(s_mqtt_card);
  lv_label_set_text(title, "MQTT Control");
  lv_obj_set_style_text_color(title, lv_color_hex(0xDFF4FF), 0);
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 0);

  s_label_link = lv_label_create(s_mqtt_card);
  lv_obj_set_style_text_color(s_label_link, lv_color_hex(0xCFE8FF), 0);
  lv_obj_align_to(s_label_link, title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);

  s_btn_fan = lv_btn_create(s_mqtt_card);
  lv_obj_set_size(s_btn_fan, 72, 28);
  lv_obj_align_to(s_btn_fan, s_label_link, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 6);
  lv_obj_add_event_cb(s_btn_fan, fan_toggle_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *label_fan = lv_label_create(s_btn_fan);
  lv_obj_center(label_fan);
  lv_label_set_text(label_fan, "Fan ON");

  lv_obj_t *btn_sync = lv_btn_create(s_mqtt_card);
  lv_obj_set_size(btn_sync, 72, 28);
  lv_obj_align_to(btn_sync, s_btn_fan, LV_ALIGN_OUT_RIGHT_MID, 8, 0);
  lv_obj_add_event_cb(btn_sync, mqtt_sync_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *label_sync = lv_label_create(btn_sync);
  lv_label_set_text(label_sync, "Sync");
  lv_obj_center(label_sync);

  s_slider_speed = lv_slider_create(s_mqtt_card);
  lv_obj_set_width(s_slider_speed, 154);
  lv_obj_align_to(s_slider_speed, s_btn_fan, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
  lv_slider_set_range(s_slider_speed, 0, 100);
  lv_slider_set_value(s_slider_speed, fan_ctrl_get_speed(), LV_ANIM_OFF);
  lv_obj_add_event_cb(s_slider_speed, fan_speed_cb, LV_EVENT_VALUE_CHANGED, NULL);

  s_label_speed = lv_label_create(s_mqtt_card);
  lv_obj_set_style_text_color(s_label_speed, lv_color_hex(0xCFE8FF), 0);
  lv_obj_align_to(s_label_speed, s_slider_speed, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);
  lv_label_set_text(s_label_speed, "Speed: 50%");

  s_label_sensor = lv_label_create(s_mqtt_card);
  lv_obj_set_style_text_color(s_label_sensor, lv_color_hex(0xE8EEF5), 0);
  lv_obj_set_width(s_label_sensor, 154);
  lv_obj_align_to(s_label_sensor, s_label_speed, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 3);
  lv_label_set_text(s_label_sensor, "Sensor: waiting...");
}

esp_err_t fan_ui_start(void) {
  bsp_display_lock(0);
  lv_obj_t *scr = lv_scr_act();

  create_mqtt_card(scr);

  s_fan_cont = lv_obj_create(scr);
  lv_obj_set_size(s_fan_cont, 100, 100);
  lv_obj_align(s_fan_cont, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
  lv_obj_set_style_bg_opa(s_fan_cont, LV_OPA_0, 0);
  lv_obj_set_style_border_width(s_fan_cont, 0, 0);
  lv_obj_clear_flag(s_fan_cont, LV_OBJ_FLAG_SCROLLABLE);

  for (int i = 0; i < 3; i++) {
    s_blades[i] = lv_arc_create(s_fan_cont);
    lv_obj_set_size(s_blades[i], 80, 80);
    lv_obj_center(s_blades[i]);
    lv_arc_set_bg_angles(s_blades[i], 0, 0);
    lv_arc_set_angles(s_blades[i], i * 120, i * 120 + 60);
    lv_obj_set_style_arc_width(s_blades[i], 10, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_blades[i], lv_color_hex(0x00AAFF),
                               LV_PART_INDICATOR);
    lv_obj_remove_style(s_blades[i], NULL, LV_PART_KNOB);
  }

  lv_anim_init(&s_anim);
  lv_anim_set_var(&s_anim, s_fan_cont);
  lv_anim_set_exec_cb(&s_anim, rotate_cb);
  lv_anim_set_values(&s_anim, 0, 3600);
  lv_anim_set_time(&s_anim, 1000);
  lv_anim_set_repeat_count(&s_anim, LV_ANIM_REPEAT_INFINITE);

  s_label_state = lv_label_create(scr);
  lv_obj_set_style_text_color(s_label_state, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(s_label_state, LV_ALIGN_RIGHT_MID, -10, 0);
  lv_label_set_text(s_label_state, "Fan: OFF");

  if (!s_status_timer) {
    s_status_timer = lv_timer_create(mqtt_status_timer_cb, 1000, NULL);
  }

  mqtt_status_timer_cb(NULL);

  bsp_display_unlock();
  return ESP_OK;
}

void fan_ui_update(bool on, uint8_t speed) {
  bsp_display_lock(0);
  if (!s_fan_cont) {
    bsp_display_unlock();
    return;
  }

  if (!on || speed == 0) {
    if (s_anim_running) {
      lv_anim_del(s_fan_cont, rotate_cb);
      s_anim_running = false;
    }
  } else {
    // Map speed 1-100 to animation time (1000ms down to 100ms)
    uint32_t time = 1000 - (speed * 9);
    if (time < 100)
      time = 100;

    lv_anim_set_time(&s_anim, time);
    if (!s_anim_running) {
      lv_anim_start(&s_anim);
      s_anim_running = true;
    } else {
      // Update existing animation time
      lv_anim_start(&s_anim);
    }
  }

  if (s_label_state) {
    if (on && speed > 0) {
      lv_label_set_text_fmt(s_label_state, "Fan: ON\nSpeed: %d%%", speed);
    } else {
      lv_label_set_text(s_label_state, "Fan: OFF");
    }
  }

  if (s_slider_speed && lv_slider_get_value(s_slider_speed) != speed) {
    s_slider_sync = true;
    lv_slider_set_value(s_slider_speed, speed, LV_ANIM_OFF);
    s_slider_sync = false;
  }

  update_fan_btn_label();

  bsp_display_unlock();
}
