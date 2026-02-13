/*
 * Minimal LVGL UI: full-screen image button that toggles the light.
 *
 * Uses PNG bytes from `gui/image/switchOn.h` and `gui/image/switchOff.h`.
 * LVGL's PNG decoder must be enabled (CONFIG_LV_USE_PNG=y).
 */

#include "light_ui.h"

#include "esp_check.h"
#include "esp_log.h"

#include "app_status.h"
#include "bsp/esp-bsp.h"
#include "light_ctrl.h"
#include "lvgl.h"

/* The provided headers come from image_to_c and contain full BMP files. */
#ifndef PROGMEM
#define PROGMEM
#endif

#include "gui/image/switchOff.h"
#include "gui/image/switchOn.h"

static const char *TAG = "light_ui";

static lv_obj_t *s_scr_main = NULL;
static lv_obj_t *s_scr_status = NULL;
static lv_obj_t *s_img = NULL;
static lv_obj_t *s_label_info = NULL;
static lv_timer_t *s_status_timer = NULL;

/* Runtime-converted LVGL images (RGB565), using LVGL's color helpers so colors
 * are correct. */
static lv_img_dsc_t s_img_on;
static lv_img_dsc_t s_img_off;
static uint8_t *s_buf_on = NULL;
static uint8_t *s_buf_off = NULL;
static bool s_imgs_ready = false;

/* Minimal BMP (24bpp, no compression) -> LVGL TRUE_COLOR image for our 320x240
 * assets. */
static esp_err_t bmp_to_lv_img(const uint8_t *bmp, size_t len,
                               lv_img_dsc_t *out_dsc, uint8_t **out_buf) {
  if (!bmp || len < 54) {
    ESP_LOGE(TAG, "BMP too small");
    return ESP_ERR_INVALID_ARG;
  }
  if (bmp[0] != 0x42 || bmp[1] != 0x4d) {
    ESP_LOGE(TAG, "Not a BMP header");
    return ESP_ERR_INVALID_ARG;
  }

  uint32_t data_offset = *(const uint32_t *)&bmp[10];
  int32_t width = *(const int32_t *)&bmp[18];
  int32_t height = *(const int32_t *)&bmp[22];
  uint16_t bpp = *(const uint16_t *)&bmp[28];
  uint32_t compression = *(const uint32_t *)&bmp[30];

  if (width <= 0 || height <= 0 || bpp != 24 || compression != 0) {
    ESP_LOGE(TAG, "Unsupported BMP: w=%ld h=%ld bpp=%u comp=%lu", (long)width,
             (long)height, (unsigned)bpp, (unsigned long)compression);
    return ESP_ERR_NOT_SUPPORTED;
  }

  uint32_t row_stride = ((width * 3) + 3) & ~3U; /* 4-byte aligned rows */
  uint32_t needed = data_offset + row_stride * (uint32_t)height;
  if (needed > len) {
    ESP_LOGE(TAG, "BMP truncated: needed=%lu len=%lu", (unsigned long)needed,
             (unsigned long)len);
    return ESP_ERR_INVALID_SIZE;
  }

  size_t out_size = (size_t)width * (size_t)height * 2; /* RGB565 */
  uint8_t *buf =
      heap_caps_malloc(out_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!buf) {
    ESP_LOGE(TAG, "No mem for RGB565 buffer (%lu bytes)",
             (unsigned long)out_size);
    return ESP_ERR_NO_MEM;
  }

  const uint8_t *src_base = bmp + data_offset;

  /* BMP is bottom-up: last row first */
  for (int y = 0; y < height; y++) {
    const uint8_t *src_row = src_base + row_stride * (height - 1 - y);
    uint16_t *dst_row = (uint16_t *)(buf + (size_t)y * (size_t)width * 2);
    for (int x = 0; x < width; x++) {
      uint8_t b = src_row[x * 3 + 0];
      uint8_t g = src_row[x * 3 + 1];
      uint8_t r = src_row[x * 3 + 2];
      /* Let LVGL handle RGB565 layout / byte order */
      lv_color_t c = lv_color_make(r, g, b);
      dst_row[x] = c.full;
    }
  }

  out_dsc->header.always_zero = 0;
  out_dsc->header.w = (uint32_t)width;
  out_dsc->header.h = (uint32_t)height;
  out_dsc->header.cf = LV_IMG_CF_TRUE_COLOR;
  out_dsc->data_size = out_size;
  out_dsc->data = buf;

  *out_buf = buf;
  return ESP_OK;
}

static esp_err_t ensure_images_ready(void) {
  if (s_imgs_ready) {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Converting BMP switch images to LVGL RGB565");
  esp_err_t err =
      bmp_to_lv_img(switch_on, sizeof(switch_on), &s_img_on, &s_buf_on);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to convert switch_on BMP: %d", (int)err);
    return err;
  }
  err = bmp_to_lv_img(switch_off, sizeof(switch_off), &s_img_off, &s_buf_off);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to convert switch_off BMP: %d", (int)err);
    return err;
  }

  ESP_LOGI(TAG, "Switch images converted OK: %lux%lu",
           (unsigned long)s_img_on.header.w, (unsigned long)s_img_on.header.h);
  s_imgs_ready = true;
  return ESP_OK;
}

static void apply_img(bool on) {
  if (!s_img) {
    return;
  }
  if (ensure_images_ready() != ESP_OK) {
    ESP_LOGW(TAG, "apply_img: images not ready, skipping");
    return;
  }
  ESP_LOGI(TAG, "apply_img: state=%s", on ? "ON" : "OFF");
  lv_img_set_src(s_img, on ? &s_img_on : &s_img_off);
}

static void on_click(lv_event_t *e) {
  (void)e;

  /* Toggle HW + persist */
  if (light_ctrl_toggle() != ESP_OK) {
    ESP_LOGW(TAG, "toggle failed");
    return;
  }

  /* Update UI */
  apply_img(light_ctrl_get());
}

static void on_status_click(lv_event_t *e) {
  (void)e;
  lv_scr_load(s_scr_status);
}

static void on_back_click(lv_event_t *e) {
  (void)e;
  lv_scr_load(s_scr_main);
}

static void update_status_timer_cb(lv_timer_t *timer) {
  if (lv_scr_act() != s_scr_status)
    return;

  app_status_t status = app_status_get();
  lv_label_set_text_fmt(
      s_label_info,
      "WiFi: %s\n"
      "SSID: %s\n"
      "IP:   %s\n\n"
      "MQTT: %s",
      status.wifi_connected ? "#00FF00 Connected#" : "#FF0000 Disconnected#",
      status.ssid[0] ? status.ssid : "---",
      status.ip[0] ? status.ip : "0.0.0.0",
      status.mqtt_connected ? "#00FF00 Connected#" : "#FF0000 Disconnected#");
}

static void create_status_screen(void) {
  s_scr_status = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(s_scr_status, lv_color_hex(0x222222), 0);

  lv_obj_t *btn_back = lv_btn_create(s_scr_status);
  lv_obj_set_size(btn_back, 80, 40);
  lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
  lv_obj_add_event_cb(btn_back, on_back_click, LV_EVENT_CLICKED, NULL);

  lv_obj_t *label_back = lv_label_create(btn_back);
  lv_label_set_text(label_back, "Back");
  lv_obj_center(label_back);

  lv_obj_t *title = lv_label_create(s_scr_status);
  lv_label_set_text(title, "Connectivity Status");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

  s_label_info = lv_label_create(s_scr_status);
  lv_label_set_recolor(s_label_info, true);
  lv_obj_set_width(s_label_info, 280);
  lv_obj_align(s_label_info, LV_ALIGN_CENTER, 0, 20);

  s_status_timer = lv_timer_create(update_status_timer_cb, 1000, NULL);
}

esp_err_t light_ui_start(void) {
  ESP_LOGI(TAG, "light_ui_start");
  bsp_display_lock(0);

  s_scr_main = lv_scr_act();
  lv_obj_clear_flag(s_scr_main, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_pad_all(s_scr_main, 0, LV_PART_MAIN);

  s_img = lv_img_create(s_scr_main);
  lv_obj_clear_flag(s_img, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(s_img, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(s_img, on_click, LV_EVENT_CLICKED, NULL);

  /* Full screen */
  lv_obj_set_size(s_img, 320, 240);
  lv_obj_align(s_img, LV_ALIGN_CENTER, 0, 0);

  /* Info button */
  lv_obj_t *btn_info = lv_btn_create(s_scr_main);
  lv_obj_set_size(btn_info, 40, 40);
  lv_obj_align(btn_info, LV_ALIGN_TOP_RIGHT, -10, 10);
  lv_obj_set_style_radius(btn_info, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(btn_info, lv_color_hex(0x555555), 0);
  lv_obj_set_style_bg_opa(btn_info, LV_OPA_50, 0);
  lv_obj_add_event_cb(btn_info, on_status_click, LV_EVENT_CLICKED, NULL);

  lv_obj_t *label_i = lv_label_create(btn_info);
  lv_label_set_text(label_i, LV_SYMBOL_SETTINGS); // Use setting/info icon
  lv_obj_center(label_i);

  create_status_screen();

  apply_img(light_ctrl_get());

  bsp_display_unlock();
  return ESP_OK;
}

void light_ui_set(bool on) {
  /* Can be called from SR task; lock LVGL. */
  bsp_display_lock(0);
  ESP_LOGI(TAG, "light_ui_set(%s)", on ? "ON" : "OFF");
  apply_img(on);
  bsp_display_unlock();
}
