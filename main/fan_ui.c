#include "fan_ui.h"

#include "app_mqtt.h"
#include "app_sensor.h"
#include "app_status.h"
#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "fan_ctrl.h"
#include "light_ctrl.h"
#include "lvgl.h"

#include <stdint.h>
#include <stdio.h>

static const char *TAG = "fan_ui";

static lv_obj_t *s_scr_main = NULL;
static lv_obj_t *s_scr_fan = NULL;
static lv_obj_t *s_scr_sensor = NULL;

static lv_obj_t *s_fan_cont = NULL;
static lv_obj_t *s_blades[3] = {NULL};
static lv_obj_t *s_label_state = NULL;
static lv_obj_t *s_slider_speed = NULL;
static lv_obj_t *s_label_speed = NULL;
static lv_obj_t *s_label_mqtt = NULL;

static lv_obj_t *s_label_sensor = NULL;
static lv_obj_t *s_label_net = NULL;
static lv_obj_t *s_label_last = NULL;

static lv_anim_t s_anim;
static bool s_anim_running = false;
static bool s_slider_sync = false;
static lv_timer_t *s_status_timer = NULL;

static void rotate_cb(void *var, int32_t v) {
  lv_obj_t *cont = (lv_obj_t *)var;
  lv_obj_set_style_transform_angle(cont, v, 0);
}

static void fan_anim_apply(bool on, uint8_t speed) {
  if (!s_fan_cont) {
    return;
  }

  if (!on || speed == 0) {
    if (s_anim_running) {
      lv_anim_del(s_fan_cont, rotate_cb);
      s_anim_running = false;
    }
  } else {
    uint32_t time = 1000U - ((uint32_t)speed * 9U);
    if (time < 100U) {
      time = 100U;
    }
    lv_anim_set_time(&s_anim, time);
    lv_anim_start(&s_anim);
    s_anim_running = true;
  }
}

static void format_sensor_line(char *out, size_t out_len, float temp, float hum,
                               bool presence) {
  int32_t t10 = (int32_t)(temp * 10.0f);
  int32_t h10 = (int32_t)(hum * 10.0f);
  int32_t t_abs = (t10 < 0) ? -t10 : t10;
  int32_t h_abs = (h10 < 0) ? -h10 : h10;

  snprintf(out, out_len, "T:%s%ld.%ld C  H:%ld.%ld%%\nPresence: %s",
           (t10 < 0) ? "-" : "", (long)(t_abs / 10), (long)(t_abs % 10),
           (long)(h_abs / 10), (long)(h_abs % 10), presence ? "ON" : "OFF");
}

static void fan_ui_refresh_widgets(void) {
  bool on = fan_ctrl_get_power();
  uint8_t speed = fan_ctrl_get_speed();
  app_status_t st = app_status_get();

  fan_anim_apply(on, speed);

  if (s_label_state) {
    if (on && speed > 0) {
      lv_label_set_text_fmt(s_label_state, "Fan: ON  (%u%%)", (unsigned)speed);
    } else {
      lv_label_set_text(s_label_state, "Fan: OFF");
    }
  }

  if (s_slider_speed && lv_slider_get_value(s_slider_speed) != speed) {
    s_slider_sync = true;
    lv_slider_set_value(s_slider_speed, speed, LV_ANIM_OFF);
    s_slider_sync = false;
  }

  if (s_label_speed) {
    lv_label_set_text_fmt(s_label_speed, "Speed: %u%%", (unsigned)speed);
  }

  if (s_label_mqtt) {
    lv_label_set_text_fmt(s_label_mqtt, "WiFi:%s  MQTT:%s  TX:%u RX:%u",
                          st.wifi_connected ? "UP" : "DOWN",
                          st.mqtt_connected ? "UP" : "DOWN",
                          (unsigned)st.mqtt_tx_count,
                          (unsigned)st.mqtt_rx_count);
  }

  if (s_label_sensor) {
    if (st.sensor_valid) {
      char line[96];
      format_sensor_line(line, sizeof(line), st.last_temp, st.last_hum,
                         st.last_presence);
      lv_label_set_text(s_label_sensor, line);
    } else {
      lv_label_set_text(s_label_sensor, "Sensor: waiting...");
    }
  }

  if (s_label_net) {
    lv_label_set_text_fmt(s_label_net, "SSID: %s\nIP: %s",
                          st.ssid[0] ? st.ssid : "---",
                          st.ip[0] ? st.ip : "0.0.0.0");
  }

  if (s_label_last) {
    lv_label_set_text_fmt(s_label_last, "Last topic:\n%s\nPayload: %s",
                          st.mqtt_last_topic[0] ? st.mqtt_last_topic : "(none)",
                          st.mqtt_last_payload[0] ? st.mqtt_last_payload : "(none)");
  }
}

static void mqtt_status_timer_cb(lv_timer_t *timer) {
  (void)timer;
  fan_ui_refresh_widgets();
}

static void on_back_main_cb(lv_event_t *e) {
  (void)e;
  if (s_scr_main) {
    lv_scr_load(s_scr_main);
  }
}

static void on_open_fan_cb(lv_event_t *e) {
  (void)e;
  fan_ui_show_fan_screen();
}

static void on_open_sensor_cb(lv_event_t *e) {
  (void)e;
  fan_ui_show_sensor_screen();
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

static void light_set_cb(lv_event_t *e) {
  uintptr_t on = (uintptr_t)lv_event_get_user_data(e);
  (void)light_ctrl_set(on != 0);
}

static void fan_preset_cb(lv_event_t *e) {
  uint8_t speed = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  fan_ctrl_set_speed(speed);
}

static lv_obj_t *create_button(lv_obj_t *parent, const char *text, lv_coord_t w,
                               lv_coord_t h, lv_event_cb_t cb,
                               void *user_data) {
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_size(btn, w, h);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);
  lv_obj_t *label = lv_label_create(btn);
  lv_label_set_text(label, text);
  lv_obj_center(label);
  return btn;
}

static void create_fan_screen(void) {
  s_scr_fan = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s_scr_fan, lv_color_hex(0x141C24), 0);
  lv_obj_clear_flag(s_scr_fan, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *btn_back =
      create_button(s_scr_fan, "Back", 64, 32, on_back_main_cb, NULL);
  lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 8, 8);

  lv_obj_t *btn_sensor =
      create_button(s_scr_fan, "Sensor", 72, 32, on_open_sensor_cb, NULL);
  lv_obj_align(btn_sensor, LV_ALIGN_TOP_RIGHT, -8, 8);

  lv_obj_t *title = lv_label_create(s_scr_fan);
  lv_label_set_text(title, "Fan + MQTT Controls");
  lv_obj_set_style_text_color(title, lv_color_hex(0xD7F0FF), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 14);

  s_fan_cont = lv_obj_create(s_scr_fan);
  lv_obj_set_size(s_fan_cont, 96, 96);
  lv_obj_align(s_fan_cont, LV_ALIGN_TOP_MID, 0, 44);
  lv_obj_set_style_bg_opa(s_fan_cont, LV_OPA_0, 0);
  lv_obj_set_style_border_width(s_fan_cont, 0, 0);
  lv_obj_clear_flag(s_fan_cont, LV_OBJ_FLAG_SCROLLABLE);

  for (int i = 0; i < 3; i++) {
    s_blades[i] = lv_arc_create(s_fan_cont);
    lv_obj_set_size(s_blades[i], 76, 76);
    lv_obj_center(s_blades[i]);
    lv_arc_set_bg_angles(s_blades[i], 0, 0);
    lv_arc_set_angles(s_blades[i], i * 120, i * 120 + 60);
    lv_obj_set_style_arc_width(s_blades[i], 10, LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_blades[i], lv_color_hex(0x29B5FF),
                               LV_PART_INDICATOR);
    lv_obj_remove_style(s_blades[i], NULL, LV_PART_KNOB);
  }

  lv_anim_init(&s_anim);
  lv_anim_set_var(&s_anim, s_fan_cont);
  lv_anim_set_exec_cb(&s_anim, rotate_cb);
  lv_anim_set_values(&s_anim, 0, 3600);
  lv_anim_set_time(&s_anim, 1000);
  lv_anim_set_repeat_count(&s_anim, LV_ANIM_REPEAT_INFINITE);

  s_label_state = lv_label_create(s_scr_fan);
  lv_obj_set_style_text_color(s_label_state, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(s_label_state, LV_ALIGN_TOP_MID, 0, 145);
  lv_label_set_text(s_label_state, "Fan: OFF");

  s_slider_speed = lv_slider_create(s_scr_fan);
  lv_obj_set_width(s_slider_speed, 220);
  lv_obj_align(s_slider_speed, LV_ALIGN_TOP_MID, 0, 167);
  lv_slider_set_range(s_slider_speed, 0, 100);
  lv_obj_add_event_cb(s_slider_speed, fan_speed_cb, LV_EVENT_VALUE_CHANGED, NULL);

  s_label_speed = lv_label_create(s_scr_fan);
  lv_obj_set_style_text_color(s_label_speed, lv_color_hex(0xD6E6F2), 0);
  lv_obj_align(s_label_speed, LV_ALIGN_TOP_MID, 0, 187);
  lv_label_set_text(s_label_speed, "Speed: 0%");

  lv_obj_t *btn_toggle =
      create_button(s_scr_fan, "Fan ON/OFF", 100, 28, fan_toggle_cb, NULL);
  lv_obj_align(btn_toggle, LV_ALIGN_BOTTOM_LEFT, 8, -8);

  lv_obj_t *btn_sync =
      create_button(s_scr_fan, "Sync MQTT", 100, 28, mqtt_sync_cb, NULL);
  lv_obj_align(btn_sync, LV_ALIGN_BOTTOM_MID, 0, -8);

  lv_obj_t *btn_light =
      create_button(s_scr_fan, "Light ON", 100, 28, light_set_cb, (void *)1);
  lv_obj_align(btn_light, LV_ALIGN_BOTTOM_RIGHT, -8, -8);

  s_label_mqtt = lv_label_create(s_scr_fan);
  lv_obj_set_style_text_color(s_label_mqtt, lv_color_hex(0xA6CBE0), 0);
  lv_obj_align(s_label_mqtt, LV_ALIGN_BOTTOM_MID, 0, -40);
  lv_label_set_text(s_label_mqtt, "WiFi:DOWN MQTT:DOWN");
}

static void create_sensor_screen(void) {
  s_scr_sensor = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s_scr_sensor, lv_color_hex(0x1B1F2A), 0);
  lv_obj_clear_flag(s_scr_sensor, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *btn_back =
      create_button(s_scr_sensor, "Back", 64, 32, on_back_main_cb, NULL);
  lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 8, 8);

  lv_obj_t *btn_fan =
      create_button(s_scr_sensor, "Fan", 64, 32, on_open_fan_cb, NULL);
  lv_obj_align(btn_fan, LV_ALIGN_TOP_RIGHT, -8, 8);

  lv_obj_t *btn_light_off =
      create_button(s_scr_sensor, "Light OFF", 88, 28, light_set_cb, (void *)0);
  lv_obj_align(btn_light_off, LV_ALIGN_TOP_RIGHT, -8, 46);

  lv_obj_t *title = lv_label_create(s_scr_sensor);
  lv_label_set_text(title, "Sensor + MQTT Monitor");
  lv_obj_set_style_text_color(title, lv_color_hex(0xF4E9CC), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 14);

  s_label_sensor = lv_label_create(s_scr_sensor);
  lv_obj_set_style_text_color(s_label_sensor, lv_color_hex(0xF0F6FF), 0);
  lv_obj_align(s_label_sensor, LV_ALIGN_TOP_LEFT, 10, 54);
  lv_label_set_text(s_label_sensor, "Sensor: waiting...");

  s_label_net = lv_label_create(s_scr_sensor);
  lv_obj_set_style_text_color(s_label_net, lv_color_hex(0xC3D9F5), 0);
  lv_obj_align(s_label_net, LV_ALIGN_TOP_LEFT, 10, 96);
  lv_label_set_text(s_label_net, "SSID: ---\nIP: 0.0.0.0");

  s_label_last = lv_label_create(s_scr_sensor);
  lv_obj_set_style_text_color(s_label_last, lv_color_hex(0xB8C4D1), 0);
  lv_obj_set_width(s_label_last, 300);
  lv_label_set_long_mode(s_label_last, LV_LABEL_LONG_CLIP);
  lv_obj_align(s_label_last, LV_ALIGN_TOP_LEFT, 10, 140);
  lv_label_set_text(s_label_last, "Last topic:\n(none)\nPayload: (none)");

  lv_obj_t *btn_sync =
      create_button(s_scr_sensor, "Sync", 74, 28, mqtt_sync_cb, NULL);
  lv_obj_align(btn_sync, LV_ALIGN_BOTTOM_LEFT, 8, -8);

  lv_obj_t *btn_p25 =
      create_button(s_scr_sensor, "Fan 25", 74, 28, fan_preset_cb, (void *)25);
  lv_obj_align(btn_p25, LV_ALIGN_BOTTOM_LEFT, 88, -8);

  lv_obj_t *btn_p50 =
      create_button(s_scr_sensor, "Fan 50", 74, 28, fan_preset_cb, (void *)50);
  lv_obj_align(btn_p50, LV_ALIGN_BOTTOM_LEFT, 168, -8);

  lv_obj_t *btn_p100 = create_button(s_scr_sensor, "Fan 100", 74, 28,
                                     fan_preset_cb, (void *)100);
  lv_obj_align(btn_p100, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
}

esp_err_t fan_ui_start(void) {
  bsp_display_lock(0);

  if (s_scr_fan && s_scr_sensor) {
    bsp_display_unlock();
    return ESP_OK;
  }

  s_scr_main = lv_scr_act();
  create_fan_screen();
  create_sensor_screen();

  if (!s_status_timer) {
    s_status_timer = lv_timer_create(mqtt_status_timer_cb, 1000, NULL);
  }
  fan_ui_refresh_widgets();

  bsp_display_unlock();
  return ESP_OK;
}

void fan_ui_show_fan_screen(void) {
  bsp_display_lock(0);
  if (s_scr_fan) {
    lv_scr_load(s_scr_fan);
  }
  bsp_display_unlock();
}

void fan_ui_show_sensor_screen(void) {
  bsp_display_lock(0);
  if (s_scr_sensor) {
    lv_scr_load(s_scr_sensor);
  }
  bsp_display_unlock();
}

void fan_ui_update(bool on, uint8_t speed) {
  (void)on;
  (void)speed;
  bsp_display_lock(0);
  fan_ui_refresh_widgets();
  bsp_display_unlock();
}
