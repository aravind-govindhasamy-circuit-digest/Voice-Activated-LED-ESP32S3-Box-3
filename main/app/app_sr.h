/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#pragma once

#include "esp_afe_sr_models.h"
#include "esp_err.h"
#include "esp_mn_models.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <sys/queue.h>


#ifdef __cplusplus
extern "C" {
#endif

#define SR_CONTINUE_DET 1
#define SR_RUN_TEST 0 /**< Just for sr experiment in laboratory >*/
#if SR_RUN_TEST
#ifdef SR_CONTINUE_DET
#undef SR_CONTINUE_DET
#define SR_CONTINUE_DET 0
#endif
#endif

#define SR_CMD_STR_LEN_MAX 64
#define SR_CMD_PHONEME_LEN_MAX 64

typedef struct {
  wakenet_state_t wakenet_mode;
  esp_mn_state_t state;
  int command_id;
} sr_result_t;

/**
 * @brief User defined command list
 *
 */
typedef enum {
  SR_CMD_SET_RED = 0,
  SR_CMD_SET_GREEN,
  SR_CMD_SET_BLUE,
  SR_CMD_LIGHT_ON,
  SR_CMD_LIGHT_OFF,
  SR_CMD_CUSTOMIZE_COLOR,
  SR_CMD_NEXT,
  SR_CMD_PLAY,
  SR_CMD_PAUSE,
  SR_CMD_AC_ON,
  SR_CMD_AC_OFF,
  SR_CMD_JOKE,
  SR_CMD_SING,
  SR_CMD_ALPHABET,
  SR_CMD_WHO_ARE_YOU,
  SR_CMD_LOVE_YOU,
  SR_CMD_STOP,
  SR_CMD_CLOSE,
  SR_CMD_HI_BRO,
  SR_CMD_CHECK_SENSORS,
  SR_CMD_ROBOT_DANCE,
  SR_CMD_CHANGE_MOOD,
  SR_CMD_RAINBOW_MODE,
  SR_CMD_GO_PLAY,
  SR_CMD_MAX,
} sr_user_cmd_t;

typedef enum {
  SR_LANG_EN,
  SR_LANG_CN,
  SR_LANG_MAX,
} sr_language_t;

typedef struct sr_cmd_t {
  sr_user_cmd_t cmd;
  sr_language_t lang;
  uint32_t id;
  char str[SR_CMD_STR_LEN_MAX];
  char phoneme[SR_CMD_PHONEME_LEN_MAX];
  SLIST_ENTRY(sr_cmd_t) next;
} sr_cmd_t;

esp_err_t app_sr_start(bool record_en);
esp_err_t app_sr_stop(void);
esp_err_t app_sr_get_result(sr_result_t *result, TickType_t xTicksToWait);
esp_err_t app_sr_set_language(sr_language_t new_lang);
esp_err_t app_sr_add_cmd(const sr_cmd_t *cmd);
esp_err_t app_sr_modify_cmd(uint32_t id, const sr_cmd_t *cmd);
esp_err_t app_sr_remove_cmd(uint32_t id);
esp_err_t app_sr_remove_all_cmd(void);
const sr_cmd_t *app_sr_get_cmd_from_id(uint32_t id);
uint8_t app_sr_search_cmd_from_user_cmd(sr_user_cmd_t user_cmd,
                                        uint8_t *id_list, uint16_t max_len);
uint8_t app_sr_search_cmd_from_phoneme(const char *phoneme, uint8_t *id_list,
                                       uint16_t max_len);
esp_err_t app_sr_update_cmds(void);

#ifdef __cplusplus
}
#endif
