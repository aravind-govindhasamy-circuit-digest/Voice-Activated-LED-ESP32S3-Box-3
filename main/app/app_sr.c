/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/queue.h>
#include <sys/time.h>

#include "app_sr.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "app_sr_handler.h"
#include "bsp_board.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "esp_process_sdkconfig.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "gui/ui_sr.h"
#include "model_path.h"

static const char *TAG = "app_sr";

/* Standard SLIST head for commands */
SLIST_HEAD(sr_cmd_list_s, sr_cmd_t);
typedef struct sr_cmd_list_s sr_cmd_list_t;

typedef struct {
  sr_language_t lang;
  char *mn_name;
  model_iface_data_t *model_data;
  const esp_mn_iface_t *multinet;
  const esp_afe_sr_iface_t *afe_handle;
  esp_afe_sr_data_t *afe_data;
  int16_t *afe_in_buffer;
  int16_t *afe_out_buffer;
  sr_cmd_list_t cmd_list;
  uint16_t cmd_num;
  TaskHandle_t feed_task;
  TaskHandle_t detect_task;
  TaskHandle_t handle_task;
  QueueHandle_t result_que;
  EventGroupHandle_t event_group;
  int16_t *utterance_buf;
  size_t utterance_samples;

  FILE *fp;
  bool b_record_en;
} sr_data_t;

static esp_afe_sr_iface_t *afe_handle = NULL;
static srmodel_list_t *models = NULL;

static sr_data_t *g_sr_data = NULL;

#define NEED_DELETE BIT0
#define FEED_DELETED BIT1
#define DETECT_DELETED BIT2

#define SR_RESULT_QUEUE_LEN 8
#define SR_FEED_TASK_CORE 0
#define SR_DETECT_TASK_CORE 1
#define SR_HANDLE_TASK_CORE 0
#define SR_FEED_TASK_PRIO 11
#define SR_DETECT_TASK_PRIO 12
#define SR_HANDLE_TASK_PRIO 5
#define SR_STOP_WAIT_MS 1500
#define SR_I2S_INPUT_CHANNELS 2
#define SR_WAKE_DEBOUNCE_MS 1200
#define SR_WAKE_SESSION_GAP_MS 1800
#define SR_FETCH_WARN_THROTTLE 50
#define SR_UTTERANCE_MAX_SAMPLES (16000 * 4)

static portMUX_TYPE s_utterance_lock = portMUX_INITIALIZER_UNLOCKED;

static bool sr_should_stop(void) {
  return (g_sr_data != NULL) && (g_sr_data->event_group != NULL) &&
         ((xEventGroupGetBits(g_sr_data->event_group) & NEED_DELETE) != 0);
}

static void sr_result_queue_push(const sr_result_t *result) {
  if ((g_sr_data == NULL) || (g_sr_data->result_que == NULL) || (result == NULL)) {
    return;
  }

  if (xQueueSend(g_sr_data->result_que, result, 0) != pdTRUE) {
    sr_result_t dropped;
    (void)xQueueReceive(g_sr_data->result_que, &dropped, 0);
    (void)xQueueSend(g_sr_data->result_que, result, 0);
  }
}

/**
 * @brief all default commands
 */
static const sr_cmd_t g_default_cmd_info[] = {
    /* English only */
    {SR_CMD_LIGHT_ON, SR_LANG_EN, 0, "LIGHT ON", "LIGHT ON", {NULL}},
    {SR_CMD_LIGHT_ON, SR_LANG_EN, 0, "TURN ON", "TURN ON", {NULL}},
    {SR_CMD_LIGHT_ON, SR_LANG_EN, 0, "SWITCH ON", "SWITCH ON", {NULL}},
    {SR_CMD_LIGHT_ON, SR_LANG_EN, 0, "TURN ON LIGHT", "TURN ON LIGHT", {NULL}},
    {SR_CMD_LIGHT_ON, SR_LANG_EN, 0, "TURN LIGHT ON", "TURN LIGHT ON", {NULL}},
    {SR_CMD_LIGHT_ON, SR_LANG_EN, 0, "SWITCH ON LIGHT", "SWITCH ON LIGHT",
     {NULL}},
    {SR_CMD_LIGHT_ON, SR_LANG_EN, 0, "LIGHTS ON", "LIGHTS ON", {NULL}},
    {SR_CMD_LIGHT_OFF, SR_LANG_EN, 0, "LIGHT OFF", "LIGHT OFF", {NULL}},
    {SR_CMD_LIGHT_OFF, SR_LANG_EN, 0, "TURN OFF", "TURN OFF", {NULL}},
    {SR_CMD_LIGHT_OFF, SR_LANG_EN, 0, "SWITCH OFF", "SWITCH OFF", {NULL}},
    {SR_CMD_LIGHT_OFF, SR_LANG_EN, 0, "TURN OFF LIGHT", "TURN OFF LIGHT",
     {NULL}},
    {SR_CMD_LIGHT_OFF, SR_LANG_EN, 0, "TURN LIGHT OFF", "TURN LIGHT OFF",
     {NULL}},
    {SR_CMD_LIGHT_OFF, SR_LANG_EN, 0, "SWITCH OFF LIGHT", "SWITCH OFF LIGHT",
     {NULL}},
    {SR_CMD_LIGHT_OFF, SR_LANG_EN, 0, "LIGHTS OFF", "LIGHTS OFF", {NULL}},
    {SR_CMD_JOKE, SR_LANG_EN, 0, "TELL ME A JOKE", "TELL ME A JOKE", {NULL}},
    {SR_CMD_SING, SR_LANG_EN, 0, "SING A SONG", "SING A SONG", {NULL}},
    {SR_CMD_ALPHABET,
     SR_LANG_EN,
     0,
     "WHAT IS THE ALPHABET",
     "WHAT IS THE ALPHABET",
     {NULL}},
    {SR_CMD_WHO_ARE_YOU, SR_LANG_EN, 0, "WHO ARE YOU", "WHO ARE YOU", {NULL}},
    {SR_CMD_LOVE_YOU, SR_LANG_EN, 0, "I LOVE YOU", "I LOVE YOU", {NULL}},
    {SR_CMD_STOP, SR_LANG_EN, 0, "STOP", "STOP", {NULL}},
    {SR_CMD_CLOSE, SR_LANG_EN, 0, "CLOSE", "CLOSE", {NULL}},
    {SR_CMD_HI_BRO, SR_LANG_EN, 0, "HI BRO", "HI BRO", {NULL}},
    {SR_CMD_CHECK_SENSORS,
     SR_LANG_EN,
     0,
     "CHECK SENSORS",
     "CHECK SENSORS",
     {NULL}},
    {SR_CMD_ROBOT_DANCE, SR_LANG_EN, 0, "ROBOT DANCE", "ROBOT DANCE", {NULL}},
    {SR_CMD_CHANGE_MOOD, SR_LANG_EN, 0, "CHANGE MOOD", "CHANGE MOOD", {NULL}},
    {SR_CMD_RAINBOW_MODE, SR_LANG_EN, 0, "RAINBOW MODE", "RAINBOW MODE",
     {NULL}},
    {SR_CMD_GO_PLAY, SR_LANG_EN, 0, "GO PLAY", "GO PLAY", {NULL}},
};

static void audio_feed_task(void *arg) {
  size_t bytes_read = 0;
  esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)arg;
  int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
  int feed_channel = afe_handle->get_total_channel_num(afe_data);
  size_t i2s_frame_bytes =
      (size_t)audio_chunksize * sizeof(int16_t) * SR_I2S_INPUT_CHANNELS;
  ESP_LOGI(TAG, "audio_feed_task: chunksize=%d, channel=%d", audio_chunksize,
           feed_channel);

  /* Allocate audio buffer and check for result */
  int16_t *audio_buffer =
      heap_caps_malloc(audio_chunksize * sizeof(int16_t) * feed_channel,
                       MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (NULL == audio_buffer) {
    esp_system_abort("No mem for audio buffer");
  }
  g_sr_data->afe_in_buffer = audio_buffer;

#if CONFIG_ESP_TASK_WDT_EN
  esp_task_wdt_add(NULL);
#endif

  while (!sr_should_stop()) {
    esp_err_t read_ret = bsp_i2s_read((char *)audio_buffer, i2s_frame_bytes,
                                      &bytes_read, pdMS_TO_TICKS(100));
    bool full_frame = (bytes_read == i2s_frame_bytes);
    if (bytes_read != i2s_frame_bytes || (read_ret != ESP_OK && !full_frame)) {
      static uint32_t short_read_cnt = 0;
      if ((short_read_cnt++ % 20U) == 0U) {
        ESP_LOGW(TAG, "i2s read mismatch ret=%s bytes=%u/%u",
                 esp_err_to_name(read_ret), (unsigned)bytes_read,
                 (unsigned)i2s_frame_bytes);
      }

      vTaskDelay(pdMS_TO_TICKS(1));
#if CONFIG_ESP_TASK_WDT_EN
      esp_task_wdt_reset();
#endif
      continue;
    }

    for (int i = audio_chunksize - 1; i >= 0; i--) {
      audio_buffer[i * 3 + 2] = 0;
      audio_buffer[i * 3 + 1] = audio_buffer[i * 2 + 1];
      audio_buffer[i * 3 + 0] = audio_buffer[i * 2 + 0];
    }

    /* Zero out buffer if audio is playing to prevent feedback loop (ghost triggers) */
    if (sr_echo_is_playing()) {
      memset(audio_buffer, 0, (size_t)audio_chunksize * sizeof(int16_t) * 3);
    }

    /* Feed audio data to AFE for processing */
    afe_handle->feed(afe_data, audio_buffer);

    // Diagnostic: Check if we have any signal (RMS)
    static int rms_count = 0;
    if (rms_count++ % 30 == 0) {
      float rms = 0;
      for (int i = 0; i < audio_chunksize; i++) {
        float sample = (float)audio_buffer[i * feed_channel];
        rms += sample * sample;
      }
      rms = sqrtf(rms / audio_chunksize);
      ESP_LOGI(TAG, "Audio In RMS: %.2f (Mic1: %d)", rms, audio_buffer[0]);
    }

#if CONFIG_ESP_TASK_WDT_EN
    esp_task_wdt_reset();
#endif

    /* Write recorded data to SD card if enabled - DISABLED to prevent lag in
     * real-time feed */
    /*
    if (g_sr_data->b_record_en && g_sr_data->fp) {
      fwrite(audio_buffer, 1, audio_chunksize * sizeof(int16_t) * feed_channel,
             g_sr_data->fp);
    }
    */
  }

  if (g_sr_data && g_sr_data->event_group) {
    xEventGroupSetBits(g_sr_data->event_group, FEED_DELETED);
  }
#if CONFIG_ESP_TASK_WDT_EN
  esp_task_wdt_delete(NULL);
#endif

  heap_caps_free(audio_buffer);
  if (g_sr_data) {
    g_sr_data->afe_in_buffer = NULL;
  }
  vTaskDelete(NULL);
}

static bool manul_detect_flag = false;

static void audio_detect_task(void *arg) {
  esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)arg;
  bool wait_for_command = false;
  uint32_t loop_cnt = 0;
  uint32_t fetch_fail_cnt = 0;
  TickType_t last_wake_tick = 0;
  TickType_t last_session_tick = 0;

#if CONFIG_ESP_TASK_WDT_EN
  esp_task_wdt_add(NULL);
#endif

  while (!sr_should_stop()) {

    afe_fetch_result_t *res = afe_handle->fetch(afe_data);
    if (!res || res->ret_value == ESP_FAIL) {
      if (!sr_echo_is_playing() &&
          ((fetch_fail_cnt++ % SR_FETCH_WARN_THROTTLE) == 0U)) {
        ESP_LOGW(TAG, "AFE fetch error or no data");
      }
      vTaskDelay(pdMS_TO_TICKS(1));
#if CONFIG_ESP_TASK_WDT_EN
      esp_task_wdt_reset();
#endif
      continue;
    }
    fetch_fail_cnt = 0;

    bool wake_hit = (res->wakeup_state == WAKENET_DETECTED) ||
                    (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) ||
                    manul_detect_flag;
    TickType_t now_tick = xTaskGetTickCount();
    bool wake_debounced = (last_wake_tick == 0) ||
                          ((now_tick - last_wake_tick) >
                           pdMS_TO_TICKS(SR_WAKE_DEBOUNCE_MS));
    bool session_gap_ok = (last_session_tick == 0) ||
                          ((now_tick - last_session_tick) >
                           pdMS_TO_TICKS(SR_WAKE_SESSION_GAP_MS));
    if (wake_hit && !wait_for_command && wake_debounced && session_gap_ok &&
        !sr_echo_is_playing()) {
      if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
        ESP_LOGI(TAG, "Wakeup channel verified ch=%d", res->trigger_channel_id);
      } else {
        ESP_LOGI(TAG, "Wakeup state=%d", (int)res->wakeup_state);
      }
      ESP_LOGI(TAG, "Wakeup detected%s (WakeID: %d)",
               manul_detect_flag ? " (Manual)" : "", res->wake_word_index);

      manul_detect_flag = false;
      wait_for_command = true;
      last_wake_tick = now_tick;
      if (g_sr_data->utterance_buf) {
        taskENTER_CRITICAL(&s_utterance_lock);
        g_sr_data->utterance_samples = 0;
        taskEXIT_CRITICAL(&s_utterance_lock);
      }

      sr_result_t result = {
          .wakenet_mode = WAKENET_DETECTED,
          .state = ESP_MN_STATE_DETECTING,
          .command_id = 0,
      };
      sr_result_queue_push(&result);

      if (g_sr_data->multinet && g_sr_data->multinet->clean &&
          g_sr_data->model_data) {
        g_sr_data->multinet->clean(g_sr_data->model_data);
      }
      if (g_sr_data->afe_handle && g_sr_data->afe_handle->disable_wakenet) {
        g_sr_data->afe_handle->disable_wakenet(afe_data);
      }
    }

    if (wait_for_command && g_sr_data->model_data) {
      if (sr_echo_is_playing()) {
        vTaskDelay(pdMS_TO_TICKS(1));
#if CONFIG_ESP_TASK_WDT_EN
        esp_task_wdt_reset();
#endif
        continue;
      }

      if (res->data == NULL || res->data_size <= 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
#if CONFIG_ESP_TASK_WDT_EN
        esp_task_wdt_reset();
#endif
        continue;
      }

      if (g_sr_data->utterance_buf && res->data && res->data_size > 0 &&
          res->vad_state == AFE_VAD_SPEECH) {
        size_t in_samples = (size_t)res->data_size / sizeof(int16_t);
        taskENTER_CRITICAL(&s_utterance_lock);
        if (g_sr_data->utterance_samples < SR_UTTERANCE_MAX_SAMPLES) {
          size_t free_samples =
              SR_UTTERANCE_MAX_SAMPLES - g_sr_data->utterance_samples;
          size_t cp_samples = (in_samples < free_samples) ? in_samples : free_samples;
          if (cp_samples > 0) {
            memcpy(g_sr_data->utterance_buf + g_sr_data->utterance_samples, res->data,
                   cp_samples * sizeof(int16_t));
            g_sr_data->utterance_samples += cp_samples;
          }
        }
        taskEXIT_CRITICAL(&s_utterance_lock);
      }

      esp_mn_state_t mn_state =
          g_sr_data->multinet->detect(g_sr_data->model_data, res->data);

#if CONFIG_ESP_TASK_WDT_EN
      esp_task_wdt_reset();
#endif

      if (mn_state == ESP_MN_STATE_DETECTED) {
        esp_mn_results_t *mn_res =
            g_sr_data->multinet->get_results(g_sr_data->model_data);
        if (mn_res == NULL || mn_res->num <= 0) {
          wait_for_command = false;
          continue;
        }
        for (int i = 0; i < mn_res->num; i++) {
          ESP_LOGI(TAG, "Command ID%d, phrase ID%d, prob %f",
                   mn_res->command_id[i], mn_res->phrase_id[i],
                   mn_res->prob[i]);
        }

        sr_result_t result = {
            .wakenet_mode = WAKENET_NO_DETECT,
            .state = mn_state,
            .command_id = mn_res->command_id[0],
        };
        sr_result_queue_push(&result);
        if (g_sr_data->afe_handle && g_sr_data->afe_handle->enable_wakenet) {
          g_sr_data->afe_handle->enable_wakenet(afe_data);
        }
        wait_for_command = false;
        last_session_tick = xTaskGetTickCount();
        last_wake_tick = last_session_tick;
      } else if (mn_state == ESP_MN_STATE_TIMEOUT) {
        sr_result_t result = {
            .wakenet_mode = WAKENET_NO_DETECT,
            .state = mn_state,
            .command_id = 0,
        };
        sr_result_queue_push(&result);
        if (g_sr_data->afe_handle && g_sr_data->afe_handle->enable_wakenet) {
          g_sr_data->afe_handle->enable_wakenet(afe_data);
        }
        wait_for_command = false;
        last_session_tick = xTaskGetTickCount();
        last_wake_tick = last_session_tick;
      }
    }

#if CONFIG_ESP_TASK_WDT_EN
    esp_task_wdt_reset();
#endif

    /* Guarantee core-1 idle slices so task WDT doesn't trip on IDLE1. */
    if ((loop_cnt++ & 0x01U) == 0U) {
      vTaskDelay(pdMS_TO_TICKS(1));
    } else {
      taskYIELD();
    }
  }

  if (g_sr_data && g_sr_data->event_group) {
    xEventGroupSetBits(g_sr_data->event_group, DETECT_DELETED);
  }
#if CONFIG_ESP_TASK_WDT_EN
  esp_task_wdt_delete(NULL);
#endif
  vTaskDelete(NULL);
}

esp_err_t app_sr_set_language(sr_language_t new_lang) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_FAIL, TAG, "SR is not running");

  if (new_lang == g_sr_data->lang) {
    return ESP_OK;
  }

  if (g_sr_data->model_data) {
    g_sr_data->multinet->destroy(g_sr_data->model_data);
    g_sr_data->model_data = NULL;
  }

  char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX,
                                     (new_lang == SR_LANG_EN) ? "en" : "cn");
  ESP_RETURN_ON_FALSE(mn_name != NULL, ESP_FAIL, TAG,
                      "Failed to find multinet model");
  esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
  ESP_RETURN_ON_FALSE(multinet != NULL, ESP_FAIL, TAG, "Failed to load multinet");
  model_iface_data_t *model_data = multinet->create(mn_name, 6000);
  ESP_RETURN_ON_FALSE(model_data != NULL, ESP_FAIL, TAG,
                      "Failed to create multinet model");

  esp_err_t mn_alloc_ret = esp_mn_commands_alloc(multinet, model_data);
  if (mn_alloc_ret != ESP_OK) {
    multinet->destroy(model_data);
    ESP_LOGE(TAG, "Failed to init speech command list");
    return mn_alloc_ret;
  }

  g_sr_data->lang = new_lang;
  g_sr_data->mn_name = mn_name;
  g_sr_data->multinet = multinet;
  g_sr_data->model_data = model_data;

  esp_err_t cmd_ret = app_sr_update_cmds();
  ESP_RETURN_ON_FALSE(cmd_ret == ESP_OK, cmd_ret, TAG,
                      "Failed to update commands");

  return ESP_OK;
}

esp_err_t app_sr_start(bool record_en) {
  esp_err_t ret = ESP_OK;
  if (g_sr_data) {
    ESP_LOGW(TAG, "SR is already running");
    return ESP_OK;
  }

  g_sr_data = heap_caps_calloc(1, sizeof(sr_data_t), MALLOC_CAP_SPIRAM);
  ESP_GOTO_ON_FALSE(NULL != g_sr_data, ESP_ERR_NO_MEM, err, TAG,
                    "Failed allocate sr_data");

  g_sr_data->result_que = xQueueCreate(SR_RESULT_QUEUE_LEN, sizeof(sr_result_t));
  ESP_GOTO_ON_FALSE(NULL != g_sr_data->result_que, ESP_ERR_NO_MEM, err, TAG,
                    "Failed create result queue");

  g_sr_data->event_group = xEventGroupCreate();
  ESP_GOTO_ON_FALSE(NULL != g_sr_data->event_group, ESP_ERR_NO_MEM, err, TAG,
                    "Failed create event_group");

  g_sr_data->utterance_buf = heap_caps_malloc(
      SR_UTTERANCE_MAX_SAMPLES * sizeof(int16_t),
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  ESP_GOTO_ON_FALSE(NULL != g_sr_data->utterance_buf, ESP_ERR_NO_MEM, err, TAG,
                    "Failed allocate utterance buffer");
  g_sr_data->utterance_samples = 0;

  SLIST_INIT(&g_sr_data->cmd_list);

  /* Create file if record to SD card enabled*/
  g_sr_data->b_record_en = record_en;
  if (record_en) {
    char file_name[32];
    for (size_t i = 0; i < 100; i++) {
      snprintf(file_name, sizeof(file_name), "/sdcard/Record_%02u.pcm",
               (unsigned int)i);
      g_sr_data->fp = fopen(file_name, "r");
      if (NULL == g_sr_data->fp) {
        break;
      }
      fclose(g_sr_data->fp);
    }
    g_sr_data->fp = fopen(file_name, "w");
    ESP_GOTO_ON_FALSE(NULL != g_sr_data->fp, ESP_FAIL, err, TAG,
                      "Failed create record file");
    ESP_LOGI(TAG, "File created at %s", file_name);
  }

  BaseType_t ret_val;

  models = esp_srmodel_init("model");
  afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
  afe_config_t afe_config = AFE_CONFIG_DEFAULT();

  afe_config.wakenet_model_name =
      esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
  afe_config.aec_init = true;
  afe_config.voice_communication_agc_init = false;
  afe_config.voice_communication_agc_gain = 15;

  /* Increased ringbuffer size to 300 to survive longer contentions */
  afe_config.afe_ringbuf_size = 300;
  afe_config.agc_mode = AFE_MN_PEAK_AGC_MODE_2;
  afe_config.wakenet_mode = DET_MODE_2CH_95;
  /* Keep voice communication path disabled for SR-only use. */
  afe_config.voice_communication_init = false;
  afe_config.wakenet_init = true;

  esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);
  ESP_GOTO_ON_FALSE(NULL != afe_data, ESP_FAIL, err, TAG,
                    "Failed to create afe_data");
  g_sr_data->afe_handle = afe_handle;
  g_sr_data->afe_data = afe_data;

  g_sr_data->lang = SR_LANG_MAX;
  ret = app_sr_set_language(SR_LANG_EN);
  ESP_GOTO_ON_FALSE(ESP_OK == ret, ESP_FAIL, err, TAG,
                    "Failed to set language");

  /* Task Affinities & Priorities:
     Feed (Core 0, Prio 12): Handles I2S DMA and AFE Pre-processing (heavy).
     Detect (Core 1, Prio 15): Handles MultiNet detection (heavy math). High
     throughput core. Handler (Core 0, Prio 5): Processes commands. Core 0 has
     less SR math load. IMPORTANT: Handler task must NOT be registered with Task
     WDT as it blocks indefinitely on queue. */

  ret_val =
      xTaskCreatePinnedToCore(&audio_feed_task, "Feed Task", 8 * 1024,
                              (void *)afe_data, SR_FEED_TASK_PRIO,
                              &g_sr_data->feed_task, SR_FEED_TASK_CORE);
  ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG,
                    "Failed create audio feed task");

  ret_val =
      xTaskCreatePinnedToCore(&audio_detect_task, "Detect Task", 16 * 1024,
                              (void *)afe_data, SR_DETECT_TASK_PRIO,
                              &g_sr_data->detect_task, SR_DETECT_TASK_CORE);
  ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG,
                    "Failed create audio detect task");

  /* sr_handler_task is implemented in app_sr_handler.c */
  ret_val =
      xTaskCreatePinnedToCore(&sr_handler_task, "SR Handler Task", 6 * 1024,
                              NULL, SR_HANDLE_TASK_PRIO, &g_sr_data->handle_task,
                              SR_HANDLE_TASK_CORE);
  ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG,
                    "Failed create audio handler task");

  return ESP_OK;
err:
  app_sr_stop();
  return ret;
}

esp_err_t app_sr_stop(void) {
  if (g_sr_data == NULL) {
    return ESP_OK;
  }

  EventBits_t stop_bits = 0;
  if (g_sr_data->event_group) {
    xEventGroupSetBits(g_sr_data->event_group, NEED_DELETE);
    stop_bits = xEventGroupWaitBits(g_sr_data->event_group,
                                    FEED_DELETED | DETECT_DELETED, pdFALSE,
                                    pdTRUE, pdMS_TO_TICKS(SR_STOP_WAIT_MS));
  }

  if (g_sr_data->feed_task && ((stop_bits & FEED_DELETED) == 0)) {
    vTaskDelete(g_sr_data->feed_task);
  }
  g_sr_data->feed_task = NULL;

  if (g_sr_data->detect_task && ((stop_bits & DETECT_DELETED) == 0)) {
    vTaskDelete(g_sr_data->detect_task);
  }
  g_sr_data->detect_task = NULL;

  if (g_sr_data->handle_task) {
    vTaskDelete(g_sr_data->handle_task);
    g_sr_data->handle_task = NULL;
  }

  if (g_sr_data->result_que) {
    vQueueDelete(g_sr_data->result_que);
    g_sr_data->result_que = NULL;
  }

  if (g_sr_data->event_group) {
    vEventGroupDelete(g_sr_data->event_group);
    g_sr_data->event_group = NULL;
  }

  if (g_sr_data->fp) {
    fclose(g_sr_data->fp);
    g_sr_data->fp = NULL;
  }

  if (g_sr_data->model_data) {
    g_sr_data->multinet->destroy(g_sr_data->model_data);
    g_sr_data->model_data = NULL;
  }

  if (g_sr_data->afe_data) {
    g_sr_data->afe_handle->destroy(g_sr_data->afe_data);
    g_sr_data->afe_data = NULL;
  }

  (void)esp_mn_commands_free();

  sr_cmd_t *it, *tmp_it;
  SLIST_FOREACH_SAFE(it, &g_sr_data->cmd_list, next, tmp_it) {
    SLIST_REMOVE(&g_sr_data->cmd_list, it, sr_cmd_t, next);
    heap_caps_free(it);
  }

  if (g_sr_data->afe_in_buffer) {
    heap_caps_free(g_sr_data->afe_in_buffer);
    g_sr_data->afe_in_buffer = NULL;
  }

  if (g_sr_data->afe_out_buffer) {
    heap_caps_free(g_sr_data->afe_out_buffer);
    g_sr_data->afe_out_buffer = NULL;
  }

  if (g_sr_data->utterance_buf) {
    heap_caps_free(g_sr_data->utterance_buf);
    g_sr_data->utterance_buf = NULL;
    g_sr_data->utterance_samples = 0;
  }

  heap_caps_free(g_sr_data);
  g_sr_data = NULL;
  return ESP_OK;
}

esp_err_t app_sr_get_result(sr_result_t *result, TickType_t xTicksToWait) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG,
                      "SR is not running");

  if (xQueueReceive(g_sr_data->result_que, result, xTicksToWait) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}

esp_err_t app_sr_add_cmd(const sr_cmd_t *cmd) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG,
                      "SR is not running");
  ESP_RETURN_ON_FALSE(NULL != cmd, ESP_ERR_INVALID_ARG, TAG,
                      "pointer of cmd is invalid");
  ESP_RETURN_ON_FALSE(cmd->lang == g_sr_data->lang, ESP_ERR_INVALID_ARG, TAG,
                      "cmd lang error");
  ESP_RETURN_ON_FALSE((uint16_t)ESP_MN_MAX_PHRASE_NUM > g_sr_data->cmd_num,
                      ESP_ERR_INVALID_STATE, TAG, "cmd is full");

  sr_cmd_t *item = (sr_cmd_t *)heap_caps_calloc(
      1, sizeof(sr_cmd_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  ESP_RETURN_ON_FALSE(NULL != item, ESP_ERR_NO_MEM, TAG,
                      "memory for sr cmd is not enough");
  memcpy(item, cmd, sizeof(sr_cmd_t));
  item->id = g_sr_data->cmd_num + 1;

  esp_err_t mn_ret = ESP_FAIL;
  if (strstr(g_sr_data->mn_name, "mn6_en")) {
    mn_ret = esp_mn_commands_add(item->id, (char *)cmd->str);
  } else {
    mn_ret = esp_mn_commands_add(item->id, (char *)cmd->phoneme);
  }
  if (mn_ret != ESP_OK) {
    ESP_LOGW(TAG, "Skip unsupported SR command: %s", cmd->str);
    heap_caps_free(item);
    return mn_ret;
  }

  /* Use standard SLIST insert head */
  SLIST_INSERT_HEAD(&g_sr_data->cmd_list, item, next);
  g_sr_data->cmd_num++;
  return ESP_OK;
}

esp_err_t app_sr_modify_cmd(uint32_t id, const sr_cmd_t *cmd) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG,
                      "SR is not running");
  ESP_RETURN_ON_FALSE(NULL != cmd, ESP_ERR_INVALID_ARG, TAG,
                      "pointer of cmd is invalid");
  ESP_RETURN_ON_FALSE((id > 0) && (id <= g_sr_data->cmd_num), ESP_ERR_INVALID_ARG,
                      TAG, "cmd id out of range");
  ESP_RETURN_ON_FALSE(cmd->lang == g_sr_data->lang, ESP_ERR_INVALID_ARG, TAG,
                      "cmd lang error");

  sr_cmd_t *it;
  SLIST_FOREACH(it, &g_sr_data->cmd_list, next) {
    if (it->id == id) {
      ESP_LOGI(TAG, "modify cmd [%d] from %s to %s", (int)id, it->str,
               cmd->str);
      if (strstr(g_sr_data->mn_name, "mn6_en")) {
        esp_mn_commands_modify(it->str, (char *)cmd->str);
      } else {
        esp_mn_commands_modify(it->phoneme, (char *)cmd->phoneme);
      }
      /* Keep the links when copying - standard SLIST doesn't need to save next
       * manually if we just copy data */
      sr_cmd_t *saved_next = SLIST_NEXT(it, next);
      uint32_t saved_id = it->id;
      memcpy(it, cmd, sizeof(sr_cmd_t));
      it->id = saved_id;
      SLIST_NEXT(it, next) = saved_next;
      break;
    }
  }

  return ESP_OK;
}

esp_err_t app_sr_remove_cmd(uint32_t id) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG,
                      "SR is not running");
  ESP_RETURN_ON_FALSE((id > 0) && (id <= g_sr_data->cmd_num), ESP_ERR_INVALID_ARG,
                      TAG, "cmd id out of range");

  sr_cmd_t *it, *tmp_it;
  SLIST_FOREACH_SAFE(it, &g_sr_data->cmd_list, next, tmp_it) {
    if (it->id == id) {
      SLIST_REMOVE(&g_sr_data->cmd_list, it, sr_cmd_t, next);
      esp_mn_commands_remove((char *)it->str);
      heap_caps_free(it);
      break;
    }
  }

  return ESP_OK;
}

esp_err_t app_sr_remove_all_cmd(void) {
  if (g_sr_data == NULL) {
    return ESP_OK;
  }

  sr_cmd_t *it, *tmp;
  SLIST_FOREACH_SAFE(it, &g_sr_data->cmd_list, next, tmp) {
    SLIST_REMOVE(&g_sr_data->cmd_list, it, sr_cmd_t, next);
    heap_caps_free(it);
  }
  esp_mn_commands_clear();
  g_sr_data->cmd_num = 0;
  return ESP_OK;
}

esp_err_t app_sr_update_cmds(void) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG,
                      "SR is not running");

  app_sr_remove_all_cmd();

  const sr_cmd_t *cmd_info = g_default_cmd_info;
  size_t cmd_count = sizeof(g_default_cmd_info) / sizeof(g_default_cmd_info[0]);
  size_t skipped = 0;
  for (size_t i = 0; i < cmd_count; i++) {
    if (cmd_info[i].lang == g_sr_data->lang) {
      if (app_sr_add_cmd(&cmd_info[i]) != ESP_OK) {
        skipped++;
      }
    }
  }

  esp_mn_error_t *mn_err = esp_mn_commands_update();
  if (mn_err != NULL && mn_err->num > 0) {
    ESP_LOGW(TAG, "MN rejected %d phrases during update", mn_err->num);
    for (int i = 0; i < mn_err->num; i++) {
      if (mn_err->phrases[i] && mn_err->phrases[i]->string) {
        ESP_LOGW(TAG, "Rejected phrase: %s", mn_err->phrases[i]->string);
      }
    }
  }
  if (g_sr_data->multinet && g_sr_data->multinet->print_active_speech_commands) {
    g_sr_data->multinet->print_active_speech_commands(g_sr_data->model_data);
  }

  ESP_LOGI(TAG, "SR commands loaded: active=%u skipped=%u",
           (unsigned)g_sr_data->cmd_num, (unsigned)skipped);
  ESP_RETURN_ON_FALSE(g_sr_data->cmd_num > 0, ESP_FAIL, TAG,
                      "No valid SR command available");
  return ESP_OK;
}

const sr_cmd_t *app_sr_get_cmd_from_id(uint32_t id) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, NULL, TAG, "SR is not running");

  sr_cmd_t *it;
  SLIST_FOREACH(it, &g_sr_data->cmd_list, next) {
    if (it->id == id) {
      return (const sr_cmd_t *)it;
    }
  }

  return NULL;
}

uint8_t app_sr_search_cmd_from_user_cmd(sr_user_cmd_t user_cmd,
                                        uint8_t *id_list, uint16_t max_len) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, 0, TAG, "SR is not running");
  ESP_RETURN_ON_FALSE(id_list != NULL, 0, TAG, "id_list is NULL");

  uint8_t count = 0;
  sr_cmd_t *it;
  SLIST_FOREACH(it, &g_sr_data->cmd_list, next) {
    if (it->cmd == user_cmd) {
      id_list[count++] = (uint8_t)it->id;
      if (count >= max_len) {
        break;
      }
    }
  }

  return count;
}

uint8_t app_sr_search_cmd_from_phoneme(const char *phoneme, uint8_t *id_list,
                                       uint16_t max_len) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, 0, TAG, "SR is not running");
  ESP_RETURN_ON_FALSE(id_list != NULL, 0, TAG, "id_list is NULL");

  uint8_t count = 0;
  sr_cmd_t *it;
  SLIST_FOREACH(it, &g_sr_data->cmd_list, next) {
    if (strcmp(it->phoneme, phoneme) == 0) {
      id_list[count++] = (uint8_t)it->id;
      if (count >= max_len) {
        break;
      }
    }
  }

  return count;
}

size_t app_sr_copy_last_utterance(int16_t *dst, size_t max_samples) {
  if (g_sr_data == NULL || g_sr_data->utterance_buf == NULL || dst == NULL ||
      max_samples == 0) {
    return 0;
  }

  size_t copied = 0;
  taskENTER_CRITICAL(&s_utterance_lock);
  size_t available = g_sr_data->utterance_samples;
  copied = (available < max_samples) ? available : max_samples;
  if (copied > 0) {
    memcpy(dst, g_sr_data->utterance_buf, copied * sizeof(int16_t));
  }
  taskEXIT_CRITICAL(&s_utterance_lock);
  return copied;
}
