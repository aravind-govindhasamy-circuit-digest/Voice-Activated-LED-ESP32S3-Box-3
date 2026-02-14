/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp/esp-bsp.h"
#include "bsp_board.h"

#include "app_sr.h"
#include "app_sr_handler.h"

#include "light_ctrl.h"
#include "light_ui.h"
#include "ui_sr.h"

static const char *TAG = "sr_handler";

static bool s_audio_playing = false;
static bool s_repeat_mode = true;

#define SR_REPEAT_MAX_SAMPLES (16000 * 4)
#define SR_REPEAT_MIN_SAMPLES 1200
#define SR_REPEAT_SILENCE_THRESH 180
#define SR_REPEAT_CHUNK_SAMPLES 512

typedef enum {
  AUDIO_WAKE,
  AUDIO_OK,
  AUDIO_END,
  AUDIO_MAX,
} audio_segment_t;

typedef struct {
  uint8_t *buf;
  size_t len;
} audio_data_t;

static audio_data_t s_audio[AUDIO_MAX];
static int16_t *s_repeat_buf = NULL;

static esp_err_t sr_codec_prepare(int sample_rate, int bits_per_sample) {
  if (sample_rate <= 0 || bits_per_sample <= 0) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Keep SR path fixed at 16k/16 to avoid runtime I2S channel re-init, which
     can stall AFE feed and trigger rb_in slow loops. */
  if (sample_rate != 16000 || bits_per_sample != 16) {
    ESP_LOGW(TAG, "Unexpected prompt format %dHz/%dbit, playing without reconfig",
             sample_rate, bits_per_sample);
  }

  bsp_codec_mute_set(false);
  int vol = 100;
  bsp_codec_volume_set(vol, &vol);
  return ESP_OK;
}

static size_t sr_trim_utterance(int16_t *samples, size_t sample_count) {
  if (samples == NULL || sample_count == 0) {
    return 0;
  }

  size_t start = 0;
  while (start < sample_count &&
         abs((int)samples[start]) < SR_REPEAT_SILENCE_THRESH) {
    start++;
  }

  size_t end = sample_count;
  while (end > start && abs((int)samples[end - 1]) < SR_REPEAT_SILENCE_THRESH) {
    end--;
  }

  size_t trimmed = end - start;
  if (trimmed > 0 && start > 0) {
    memmove(samples, samples + start, trimmed * sizeof(int16_t));
  }

  return trimmed;
}

static esp_err_t load_wav_to_mem(audio_segment_t seg, const char *path) {
  FILE *fp = fopen(path, "rb");
  if (!fp) {
    return ESP_ERR_NOT_FOUND;
  }
  fseek(fp, 0, SEEK_END);
  long sz = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  if (sz <= 0) {
    fclose(fp);
    return ESP_FAIL;
  }

  if (s_audio[seg].buf) {
    heap_caps_free(s_audio[seg].buf);
    s_audio[seg].buf = NULL;
    s_audio[seg].len = 0;
  }

  s_audio[seg].len = (size_t)sz;
  s_audio[seg].buf =
      heap_caps_malloc(s_audio[seg].len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!s_audio[seg].buf) {
    fclose(fp);
    return ESP_ERR_NO_MEM;
  }
  fread(s_audio[seg].buf, 1, s_audio[seg].len, fp);
  fclose(fp);
  return ESP_OK;
}

static esp_err_t sr_echo_init(void) {
  /* English only */
  ESP_LOGI(TAG, "Loading SR echo wavs from SPIFFS");
  ESP_RETURN_ON_ERROR(load_wav_to_mem(AUDIO_WAKE, "/spiffs/echo_en_wake.wav"),
                      TAG, "load wake wav failed");
  ESP_RETURN_ON_ERROR(load_wav_to_mem(AUDIO_OK, "/spiffs/echo_en_ok.wav"), TAG,
                      "load ok wav failed");
  ESP_RETURN_ON_ERROR(load_wav_to_mem(AUDIO_END, "/spiffs/echo_en_end.wav"),
                      TAG, "load end wav failed");
  ESP_LOGI(TAG, "SR echo wavs loaded: wake=%u bytes, ok=%u bytes, end=%u bytes",
           (unsigned)s_audio[AUDIO_WAKE].len, (unsigned)s_audio[AUDIO_OK].len,
           (unsigned)s_audio[AUDIO_END].len);

  if (s_repeat_buf == NULL) {
    s_repeat_buf = heap_caps_malloc(SR_REPEAT_MAX_SAMPLES * sizeof(int16_t),
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_repeat_buf == NULL) {
      ESP_LOGW(TAG, "Failed to allocate repeat buffer");
    }
  }
  return ESP_OK;
}

static esp_err_t sr_echo_play(audio_segment_t seg) {
  typedef struct {
    uint8_t ChunkID[4];
    int32_t ChunkSize;
    uint8_t Format[4];
    uint8_t Subchunk1ID[4];
    int32_t Subchunk1Size;
    int16_t AudioFormat;
    int16_t NumChannels;
    int32_t SampleRate;
    int32_t ByteRate;
    int16_t BlockAlign;
    int16_t BitsPerSample;
    uint8_t Subchunk2ID[4];
    int32_t Subchunk2Size;
  } wav_header_t;

  if (!s_audio[seg].buf || s_audio[seg].len <= sizeof(wav_header_t)) {
    ESP_LOGW(TAG, "sr_echo_play(%d): buffer invalid (len=%u)", seg,
             (unsigned)s_audio[seg].len);
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t *p = s_audio[seg].buf;
  wav_header_t *h = (wav_header_t *)p;
  p += sizeof(wav_header_t);
  size_t len = s_audio[seg].len - sizeof(wav_header_t);
  len &= 0xfffffffc;

  /* Keep SR prompts short to avoid starving capture path. */
  size_t max_len = len;
  if (seg == AUDIO_WAKE) {
    max_len = 8 * 1024;
  } else if (seg == AUDIO_OK || seg == AUDIO_END) {
    max_len = 12 * 1024;
  }
  if (len > max_len) {
    len = max_len & 0xfffffffc;
  }

  if (sr_codec_prepare((int)h->SampleRate, (int)h->BitsPerSample) != ESP_OK) {
    ESP_LOGW(TAG, "sr_echo_play(%d): codec prepare failed", seg);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "sr_echo_play(%d): %d Hz, %d bits, data_len=%u", seg,
           (int)h->SampleRate, (int)h->BitsPerSample, (unsigned)len);

  if (len == 0) {
    return ESP_OK;
  }

  size_t bytes_written = 0;
  size_t chunk_size = 4 * 1024;
  s_audio_playing = true;

  for (size_t offset = 0; offset < len; offset += chunk_size) {
    size_t to_write = (len - offset > chunk_size) ? chunk_size : (len - offset);
    size_t written = 0;
    esp_err_t write_ret =
        bsp_i2s_write((char *)(p + offset), to_write, &written, pdMS_TO_TICKS(200));
    if (write_ret != ESP_OK) {
      ESP_LOGW(TAG, "sr_echo_play(%d): write failed ret=%s", seg,
               esp_err_to_name(write_ret));
      break;
    }
    bytes_written += written;
    /* Yield to allow other tasks (Feed, Detect, IDLE) to run */
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  ESP_LOGI(TAG, "sr_echo_play(%d): wrote %u bytes", seg,
           (unsigned)bytes_written);
  vTaskDelay(pdMS_TO_TICKS(20));
  s_audio_playing = false;
  return ESP_OK;
}

static esp_err_t sr_echo_repeat_last_utterance(void) {
  if (s_repeat_buf == NULL) {
    return ESP_ERR_NO_MEM;
  }

  size_t sample_count =
      app_sr_copy_last_utterance(s_repeat_buf, SR_REPEAT_MAX_SAMPLES);
  sample_count = sr_trim_utterance(s_repeat_buf, sample_count);
  if (sample_count < SR_REPEAT_MIN_SAMPLES) {
    return ESP_ERR_NOT_FOUND;
  }

  if (sr_codec_prepare(16000, 16) != ESP_OK) {
    return ESP_FAIL;
  }

  int16_t stereo_buf[SR_REPEAT_CHUNK_SAMPLES * 2];
  size_t total_bytes = 0;
  s_audio_playing = true;

  for (size_t offset = 0; offset < sample_count; offset += SR_REPEAT_CHUNK_SAMPLES) {
    size_t this_samples = sample_count - offset;
    if (this_samples > SR_REPEAT_CHUNK_SAMPLES) {
      this_samples = SR_REPEAT_CHUNK_SAMPLES;
    }

    for (size_t i = 0; i < this_samples; i++) {
      int16_t s = s_repeat_buf[offset + i];
      stereo_buf[i * 2] = s;
      stereo_buf[i * 2 + 1] = s;
    }

    size_t to_write = this_samples * sizeof(int16_t) * 2;
    size_t written = 0;
    esp_err_t write_ret =
        bsp_i2s_write((char *)stereo_buf, to_write, &written, pdMS_TO_TICKS(200));
    if (write_ret != ESP_OK) {
      ESP_LOGW(TAG, "Repeat write failed ret=%s", esp_err_to_name(write_ret));
      break;
    }
    total_bytes += written;
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  ESP_LOGI(TAG, "Repeat playback wrote %u bytes", (unsigned)total_bytes);
  vTaskDelay(pdMS_TO_TICKS(20));
  s_audio_playing = false;
  return total_bytes > 0 ? ESP_OK : ESP_FAIL;
}

bool sr_echo_is_playing(void) { return s_audio_playing; }

void sr_handler_task(void *pvParam) {
  (void)pvParam;

  /* SR Handler Task blocks indefinitely on queue, so we should NOT register it
     with Task WDT unless we use a timeout. Removing registration is safer. */

  /* Needs SPIFFS mounted in main */
  if (sr_echo_init() != ESP_OK) {
    ESP_LOGW(TAG, "SR echo wav init failed (tones disabled)");
  }

  while (true) {
    sr_result_t result;
    if (app_sr_get_result(&result, portMAX_DELAY) != ESP_OK) {
      continue;
    }

    if (result.wakenet_mode == WAKENET_DETECTED) {
      sr_anim_start();
      sr_anim_set_text(s_repeat_mode ? "Say anything" : "Say command");
      (void)sr_echo_play(AUDIO_WAKE);
      continue;
    }

    if (result.state == ESP_MN_STATE_TIMEOUT) {
      bool repeated = false;
      if (s_repeat_mode) {
        sr_anim_set_text("Repeating...");
        repeated = (sr_echo_repeat_last_utterance() == ESP_OK);
      }
      if (!repeated) {
        sr_anim_set_text("Timeout");
        (void)sr_echo_play(AUDIO_END);
      }
      sr_anim_stop();
      continue;
    }

    if (result.state == ESP_MN_STATE_DETECTED) {
      const sr_cmd_t *cmd = app_sr_get_cmd_from_id(result.command_id);
      if (cmd) {
        ESP_LOGI(TAG, "command:%s, act:%d", cmd->str, cmd->cmd);
        sr_anim_set_text((char *)cmd->str);
      } else {
        ESP_LOGW(TAG, "Unknown command_id=%d", result.command_id);
      }

      switch (cmd ? cmd->cmd : SR_CMD_MAX) {
      case SR_CMD_LIGHT_ON:
        (void)light_ctrl_set(true);
        light_ui_set(true);
        break;
      case SR_CMD_LIGHT_OFF:
        (void)light_ctrl_set(false);
        light_ui_set(false);
        break;
      case SR_CMD_JOKE:
        ESP_LOGI(TAG, "Command: TELL ME A JOKE");
        break;
      case SR_CMD_SING:
        ESP_LOGI(TAG, "Command: SING A SONG");
        break;
      case SR_CMD_ALPHABET:
        ESP_LOGI(TAG, "Command: WHAT IS THE ALPHABET");
        break;
      case SR_CMD_WHO_ARE_YOU:
        ESP_LOGI(TAG, "Command: WHO ARE YOU");
        break;
      case SR_CMD_LOVE_YOU:
        ESP_LOGI(TAG, "Command: I LOVE YOU");
        break;
      case SR_CMD_STOP:
        ESP_LOGI(TAG, "Command: STOP");
        s_repeat_mode = false;
        sr_anim_set_text("Repeat: OFF");
        break;
      case SR_CMD_CLOSE:
        ESP_LOGI(TAG, "Command: CLOSE");
        break;
      case SR_CMD_HI_BRO:
        ESP_LOGI(TAG, "Command: HI BRO");
        break;
      case SR_CMD_CHECK_SENSORS:
        ESP_LOGI(TAG, "Command: CHECK SENSORS");
        break;
      case SR_CMD_ROBOT_DANCE:
        ESP_LOGI(TAG, "Command: ROBOT DANCE");
        break;
      case SR_CMD_CHANGE_MOOD:
        ESP_LOGI(TAG, "Command: CHANGE MOOD");
        break;
      case SR_CMD_RAINBOW_MODE:
        ESP_LOGI(TAG, "Command: RAINBOW MODE");
        break;
      case SR_CMD_GO_PLAY:
        ESP_LOGI(TAG, "Command: GO PLAY");
        s_repeat_mode = !s_repeat_mode;
        sr_anim_set_text(s_repeat_mode ? "Repeat: ON" : "Repeat: OFF");
        break;
      default:
        ESP_LOGW(TAG, "Unhandled cmd: %d", cmd ? cmd->cmd : -1);
        break;
      }

      bool repeated = false;
      if (s_repeat_mode &&
          (cmd == NULL || cmd->cmd != SR_CMD_GO_PLAY)) {
        repeated = (sr_echo_repeat_last_utterance() == ESP_OK);
      }
      if (!repeated) {
        (void)sr_echo_play(AUDIO_OK);
      }
      sr_anim_stop();
    }
  }
}
