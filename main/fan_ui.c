#include "fan_ui.h"
#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "lvgl.h"

// static const char *TAG = "fan_ui";
static lv_obj_t *s_fan_cont = NULL;
static lv_obj_t *s_blades[3];
static lv_obj_t *s_label_state = NULL;
static lv_anim_t s_anim;
static bool s_anim_running = false;

static void rotate_cb(void *var, int32_t v) {
  lv_obj_t *cont = (lv_obj_t *)var;
  lv_obj_set_style_transform_angle(cont, v, 0);
}

esp_err_t fan_ui_start(void) {
  bsp_display_lock(0);
  lv_obj_t *scr = lv_scr_act();

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
  bsp_display_unlock();
}
