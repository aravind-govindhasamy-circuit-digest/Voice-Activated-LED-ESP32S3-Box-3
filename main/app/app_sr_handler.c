/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_check.h"

#include "bsp_board.h"
#include "bsp/esp-bsp.h"

#include "app_sr.h"
#include "app_sr_handler.h"

#include "ui_sr.h"
#include "light_ctrl.h"
#include "light_ui.h"

static const char *TAG = "sr_handler";

static bool s_audio_playing = false;

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

static esp_err_t load_wav_to_mem(audio_segment_t seg, const char *path)
{
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
    s_audio[seg].buf = heap_caps_malloc(s_audio[seg].len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_audio[seg].buf) {
        fclose(fp);
        return ESP_ERR_NO_MEM;
    }
    fread(s_audio[seg].buf, 1, s_audio[seg].len, fp);
    fclose(fp);
    return ESP_OK;
}

static esp_err_t sr_echo_init(void)
{
    /* English only */
    ESP_LOGI(TAG, "Loading SR echo wavs from SPIFFS");
    ESP_RETURN_ON_ERROR(load_wav_to_mem(AUDIO_WAKE, "/spiffs/echo_en_wake.wav"), TAG, "load wake wav failed");
    ESP_RETURN_ON_ERROR(load_wav_to_mem(AUDIO_OK, "/spiffs/echo_en_ok.wav"), TAG, "load ok wav failed");
    ESP_RETURN_ON_ERROR(load_wav_to_mem(AUDIO_END, "/spiffs/echo_en_end.wav"), TAG, "load end wav failed");
    ESP_LOGI(TAG, "SR echo wavs loaded: wake=%u bytes, ok=%u bytes, end=%u bytes",
             (unsigned)s_audio[AUDIO_WAKE].len,
             (unsigned)s_audio[AUDIO_OK].len,
             (unsigned)s_audio[AUDIO_END].len);
    return ESP_OK;
}

static esp_err_t sr_echo_play(audio_segment_t seg)
{
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
        ESP_LOGW(TAG, "sr_echo_play(%d): buffer invalid (len=%u)", seg, (unsigned)s_audio[seg].len);
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t *p = s_audio[seg].buf;
    wav_header_t *h = (wav_header_t *)p;
    p += sizeof(wav_header_t);
    size_t len = s_audio[seg].len - sizeof(wav_header_t);
    len &= 0xfffffffc;

    /* Configure codec for wav format */
    ESP_LOGI(TAG, "sr_echo_play(%d): %d Hz, %d bits, data_len=%u", seg,
             (int)h->SampleRate, (int)h->BitsPerSample, (unsigned)len);
    bsp_codec_set_fs(h->SampleRate, h->BitsPerSample, I2S_SLOT_MODE_STEREO);
    /* Ensure codec is unmuted and volume is reasonable for feedback tones */
    bsp_codec_mute_set(false);
    int vol = 100;
    bsp_codec_volume_set(vol, &vol);

    size_t bytes_written = 0;
    s_audio_playing = true;
    bsp_i2s_write((char *)p, len, &bytes_written, portMAX_DELAY);
    ESP_LOGI(TAG, "sr_echo_play(%d): wrote %u bytes", seg, (unsigned)bytes_written);
    vTaskDelay(pdMS_TO_TICKS(20));
    s_audio_playing = false;
    return ESP_OK;
}

bool sr_echo_is_playing(void)
{
    return s_audio_playing;
}

void sr_handler_task(void *pvParam)
{
    (void)pvParam;

    /* Needs SPIFFS mounted in main */
    if (sr_echo_init() != ESP_OK) {
        ESP_LOGW(TAG, "SR echo wav init failed (tones disabled)");
    }

    while (true) {
        sr_result_t result;
        app_sr_get_result(&result, portMAX_DELAY);

        if (result.wakenet_mode == WAKENET_DETECTED) {
            sr_anim_start();
            sr_anim_set_text("Say command");
            (void)sr_echo_play(AUDIO_WAKE);
            continue;
        }

        if (result.state == ESP_MN_STATE_TIMEOUT) {
            sr_anim_set_text("Timeout");
            (void)sr_echo_play(AUDIO_END);
            sr_anim_stop();
            continue;
        }

        if (ESP_MN_STATE_DETECTED & result.state) {
            const sr_cmd_t *cmd = app_sr_get_cmd_from_id(result.command_id);
            if (cmd) {
                ESP_LOGI(TAG, "command:%s, act:%d", cmd->str, cmd->cmd);
                sr_anim_set_text((char *)cmd->str);
            }
            (void)sr_echo_play(AUDIO_OK);
            sr_anim_stop();

            switch (cmd ? cmd->cmd : SR_CMD_MAX) {
            case SR_CMD_LIGHT_ON:
                (void)light_ctrl_set(true);
                light_ui_set(true);
                break;
            case SR_CMD_LIGHT_OFF:
                (void)light_ctrl_set(false);
                light_ui_set(false);
                break;
            default:
                ESP_LOGW(TAG, "Unhandled cmd");
                break;
            }
        }
    }
}
