/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "app_sr.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/queue.h>
#include <sys/time.h>

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

/* Define the head structure for the command list using the struct tag */
SLIST_HEAD(sr_cmd_list_t, sr_cmd_t);

typedef struct {
  sr_language_t lang;
  char *mn_name;
  model_iface_data_t *model_data;
  const esp_mn_iface_t *multinet;
  const esp_afe_sr_iface_t *afe_handle;
  esp_afe_sr_data_t *afe_data;
  int16_t *afe_in_buffer;
  int16_t *afe_out_buffer;
  struct sr_cmd_list_t cmd_list;
  uint16_t cmd_num; /* Changed to uint16_t to match comparison */
  TaskHandle_t feed_task;
  TaskHandle_t detect_task;
  TaskHandle_t handle_task;
  QueueHandle_t result_que;
  EventGroupHandle_t event_group;

  FILE *fp;
  bool b_record_en;
} sr_data_t;

static esp_afe_sr_iface_t *afe_handle = NULL;
static srmodel_list_t *models = NULL;

static sr_data_t *g_sr_data = NULL;

#define I2S_CHANNEL_NUM (2)
#define NEED_DELETE BIT0
#define FEED_DELETED BIT1
#define DETECT_DELETED BIT2

/**
 * @brief all default commands
 */
static const sr_cmd_t g_default_cmd_info[] = {
    /* English only, minimal command set */
    {SR_CMD_LIGHT_ON, SR_LANG_EN, 0, "turn on light", "TkN nN LiT", {NULL}},
    {SR_CMD_LIGHT_OFF, SR_LANG_EN, 0, "turn off light", "TkN eF LiT", {NULL}},
};

static void audio_feed_task(void *arg) {
  size_t bytes_read = 0;
  esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)arg;
  int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
  int feed_channel = 3;
  ESP_LOGI(TAG, "audio_chunksize=%d, feed_channel=%d", audio_chunksize,
           feed_channel);

  /* Allocate audio buffer and check for result */
  int16_t *audio_buffer =
      heap_caps_malloc(audio_chunksize * sizeof(int16_t) * feed_channel,
                       MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (NULL == audio_buffer) {
    esp_system_abort("No mem for audio buffer");
  }
  g_sr_data->afe_in_buffer = audio_buffer;

  while (true) {
    if (NEED_DELETE && xEventGroupGetBits(g_sr_data->event_group)) {
      xEventGroupSetBits(g_sr_data->event_group, FEED_DELETED);
      vTaskDelete(NULL);
    }

    /* Read audio data from I2S bus */
    bsp_i2s_read((char *)audio_buffer,
                 audio_chunksize * sizeof(int16_t) * feed_channel, &bytes_read,
                 portMAX_DELAY);

    /* Feed audio data to AFE for processing */
    afe_handle->feed(afe_data, audio_buffer);

    /* Write recorded data to SD card if enabled */
    if (g_sr_data->b_record_en && g_sr_data->fp) {
      fwrite(audio_buffer, 1, audio_chunksize * sizeof(int16_t) * feed_channel,
             g_sr_data->fp);
    }
  }

  heap_caps_free(audio_buffer);
  vTaskDelete(NULL);
}

static void audio_detect_task(void *arg) {
  esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)arg;
  int audio_chunksize = afe_handle->get_fetch_chunksize(afe_data);
  int16_t *buff = heap_caps_malloc(audio_chunksize * sizeof(int16_t),
                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  assert(buff);
  g_sr_data->afe_out_buffer = buff;

  while (true) {
    if (NEED_DELETE && xEventGroupGetBits(g_sr_data->event_group)) {
      xEventGroupSetBits(g_sr_data->event_group, DETECT_DELETED);
      vTaskDelete(NULL);
    }

    afe_fetch_result_t *res = afe_handle->fetch(afe_data);
    if (!res || res->ret_value == ESP_FAIL) {
      ESP_LOGW(TAG, "AFE fetch error");
      continue;
    }

    if (res->wakeup_state == WAKENET_DETECTED) {
      ESP_LOGI(TAG, "Wakeup detected");
      sr_result_t result = {
          .wakenet_mode = WAKENET_DETECTED,
          .state = ESP_MN_STATE_DETECTING,
          .command_id = 0,
      };
      xQueueSend(g_sr_data->result_que, &result, 0);
    }

    if (g_sr_data->model_data) {
      esp_mn_state_t mn_state =
          g_sr_data->multinet->detect(g_sr_data->model_data, res->data);

      if (mn_state == ESP_MN_STATE_DETECTED) {
        esp_mn_results_t *mn_res =
            g_sr_data->multinet->get_results(g_sr_data->model_data);
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
        xQueueSend(g_sr_data->result_que, &result, 0);
      }
    }
  }

  heap_caps_free(buff);
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
  esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
  model_iface_data_t *model_data = multinet->create(mn_name, 6000);

  g_sr_data->lang = new_lang;
  g_sr_data->mn_name = mn_name;
  g_sr_data->multinet = multinet;
  g_sr_data->model_data = model_data;

  app_sr_update_cmds();

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

  g_sr_data->result_que = xQueueCreate(3, sizeof(sr_result_t));
  ESP_GOTO_ON_FALSE(NULL != g_sr_data->result_que, ESP_ERR_NO_MEM, err, TAG,
                    "Failed create result queue");

  g_sr_data->event_group = xEventGroupCreate();
  ESP_GOTO_ON_FALSE(NULL != g_sr_data->event_group, ESP_ERR_NO_MEM, err, TAG,
                    "Failed create event_group");

  SLIST_INIT(&g_sr_data->cmd_list);

  /* Create file if record to SD card enabled*/
  g_sr_data->b_record_en = record_en;
  if (record_en) {
    char file_name[32];
    for (size_t i = 0; i < 100; i++) {
      sprintf(file_name, "/sdcard/Record_%02d.pcm", i);
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
  /* Adjusting gain for improved hearing capability */
  afe_config.voice_communication_agc_init = true;
  afe_config.voice_communication_agc_gain = 20;

  afe_config.agc_mode = AFE_MN_PEAK_AGC_MODE_2;
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

  ret_val =
      xTaskCreatePinnedToCore(&audio_feed_task, "Feed Task", 4 * 1024,
                              (void *)afe_data, 5, &g_sr_data->feed_task, 0);
  ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG,
                    "Failed create audio feed task");

  ret_val =
      xTaskCreatePinnedToCore(&audio_detect_task, "Detect Task", 8 * 1024,
                              (void *)afe_data, 5, &g_sr_data->detect_task, 1);
  ESP_GOTO_ON_FALSE(pdPASS == ret_val, ESP_FAIL, err, TAG,
                    "Failed create audio detect task");

  /* sr_handler_task is implemented in app_sr_handler.c */
  ret_val = xTaskCreatePinnedToCore(&sr_handler_task, "SR Handler Task",
                                    6 * 1024, NULL, configMAX_PRIORITIES - 1,
                                    &g_sr_data->handle_task, 0);
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

  /**
   * Waiting for all task stopped
   * TODO: A task creation failure cannot be handled correctly now
   * */
  xEventGroupSetBits(g_sr_data->event_group, NEED_DELETE);
  xEventGroupWaitBits(g_sr_data->event_group,
                      NEED_DELETE | FEED_DELETED | DETECT_DELETED, 1, 1,
                      portMAX_DELAY / 10);

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
  }

  if (g_sr_data->afe_data) {
    g_sr_data->afe_handle->destroy(g_sr_data->afe_data);
  }

  sr_cmd_t *it;
  while (!SLIST_EMPTY(&g_sr_data->cmd_list)) {
    it = SLIST_FIRST(&g_sr_data->cmd_list);
    SLIST_REMOVE_HEAD(&g_sr_data->cmd_list, next);
    heap_caps_free(it);
  }

  if (g_sr_data->afe_in_buffer) {
    heap_caps_free(g_sr_data->afe_in_buffer);
  }

  if (g_sr_data->afe_out_buffer) {
    heap_caps_free(g_sr_data->afe_out_buffer);
  }

  heap_caps_free(g_sr_data);
  g_sr_data = NULL;
  return ESP_OK;
}

esp_err_t app_sr_get_result(sr_result_t *result, TickType_t xTicksToWait) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG,
                      "SR is not running");

  xQueueReceive(g_sr_data->result_que, result, xTicksToWait);
  return ESP_OK;
}

esp_err_t app_sr_add_cmd(const sr_cmd_t *cmd) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG,
                      "SR is not running");
  ESP_RETURN_ON_FALSE(NULL != cmd, ESP_ERR_INVALID_ARG, TAG,
                      "pointer of cmd is invalid");
  ESP_RETURN_ON_FALSE(cmd->lang == g_sr_data->lang, ESP_ERR_INVALID_ARG, TAG,
                      "cmd lang error");
  ESP_RETURN_ON_FALSE(ESP_MN_MAX_PHRASE_NUM >= g_sr_data->cmd_num,
                      ESP_ERR_INVALID_STATE, TAG, "cmd is full");

  sr_cmd_t *item = (sr_cmd_t *)heap_caps_calloc(
      1, sizeof(sr_cmd_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  ESP_RETURN_ON_FALSE(NULL != item, ESP_ERR_NO_MEM, TAG,
                      "memory for sr cmd is not enough");
  memcpy(item, cmd, sizeof(sr_cmd_t));

#if 1 // insert after
  sr_cmd_t *last = SLIST_FIRST(&g_sr_data->cmd_list);
  if (last == NULL) {
    SLIST_INSERT_HEAD(&g_sr_data->cmd_list, item, next);
  } else {
    sr_cmd_t *it;
    while ((it = SLIST_NEXT(last, next)) != NULL) {
      last = it;
    }
    SLIST_INSERT_AFTER(last, item, next);
  }
#else // insert head
  SLIST_INSERT_HEAD(&g_sr_data->cmd_list, item, next);
#endif

  if (strstr(g_sr_data->mn_name, "mn6_en")) {
    esp_mn_commands_add(g_sr_data->cmd_num, (char *)cmd->str);
  } else {
    esp_mn_commands_add(g_sr_data->cmd_num, (char *)cmd->phoneme);
  }
  g_sr_data->cmd_num++;
  return ESP_OK;
}

esp_err_t app_sr_modify_cmd(uint32_t id, const sr_cmd_t *cmd) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG,
                      "SR is not running");
  ESP_RETURN_ON_FALSE(NULL != cmd, ESP_ERR_INVALID_ARG, TAG,
                      "pointer of cmd is invalid");
  ESP_RETURN_ON_FALSE(id < g_sr_data->cmd_num, ESP_ERR_INVALID_ARG, TAG,
                      "cmd id out of range");
  ESP_RETURN_ON_FALSE(cmd->lang == g_sr_data->lang, ESP_ERR_INVALID_ARG, TAG,
                      "cmd lang error");

  sr_cmd_t *it;
  SLIST_FOREACH(it, &g_sr_data->cmd_list, next) {
    if (it->id == id) {
      ESP_LOGI(TAG, "modify cmd [%d] from %s to %s", id, it->str, cmd->str);
      if (strstr(g_sr_data->mn_name, "mn6_en")) {
        esp_mn_commands_modify(it->str, (char *)cmd->str);
      } else {
        esp_mn_commands_modify(it->phoneme, (char *)cmd->phoneme);
      }
      memcpy(it, cmd, sizeof(sr_cmd_t));
      break;
    }
  }

  return ESP_OK;
}

esp_err_t app_sr_remove_cmd(uint32_t id) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG,
                      "SR is not running");
  ESP_RETURN_ON_FALSE(id < g_sr_data->cmd_num, ESP_ERR_INVALID_ARG, TAG,
                      "cmd id out of range");

  sr_cmd_t *it;
  SLIST_FOREACH(it, &g_sr_data->cmd_list, next) {
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
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, ESP_ERR_INVALID_STATE, TAG,
                      "SR is not running");

  sr_cmd_t *it;
  while (!SLIST_EMPTY(&g_sr_data->cmd_list)) {
    it = SLIST_FIRST(&g_sr_data->cmd_list);
    SLIST_REMOVE_HEAD(&g_sr_data->cmd_list, next);
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
  for (size_t i = 0; i < sizeof(g_default_cmd_info) / sizeof(sr_cmd_t); i++) {
    if (cmd_info[i].lang == g_sr_data->lang) {
      app_sr_add_cmd(&cmd_info[i]);
    }
  }

  return ESP_OK;
}

const sr_cmd_t *app_sr_get_cmd_from_id(uint32_t id) {
  ESP_RETURN_ON_FALSE(NULL != g_sr_data, NULL, TAG, "SR is not running");
  ESP_RETURN_ON_FALSE(id < g_sr_data->cmd_num, NULL, TAG,
                      "cmd id out of range");

  sr_cmd_t *it;
  SLIST_FOREACH(it, &g_sr_data->cmd_list, next) {
    if (it->id == id) {
      return it;
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
      id_list[count++] = it->id;
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
      id_list[count++] = it->id;
      if (count >= max_len) {
        break;
      }
    }
  }

  return count;
}
