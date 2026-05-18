/*
 * app_mqtt.c — Raw-socket MQTT 3.1.1 client (PubSub style)
 *
 * Bypasses esp-mqtt entirely to avoid the transport-layer colon bug.
 * Uses standard POSIX sockets + lwIP getaddrinfo directly.
 */

#include <errno.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lwip/dns.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"

#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "cJSON.h"
#include "sdkconfig.h"

#include "app_mqtt.h"
#include "app_sensor.h"
#include "app_status.h"
#include "fan_ctrl.h"
#include "light_ctrl.h"

static const char *TAG = "app_mqtt";

/* ── Broker settings ───────────────────────────────────────────────────────── */
#define MQTT_HOST           "mqtt.circuitdigest.cloud"
#define MQTT_PORT           1883
#define MQTT_KEEPALIVE_SEC  60          /* broker keepalive (seconds)          */
#define MQTT_PING_INTERVAL  30          /* send PINGREQ every 30 s             */
#define MQTT_RECV_BUF_SIZE  1024
#define MQTT_STACK_SIZE     6144
#define MQTT_TASK_PRIO      5

/* ── State ─────────────────────────────────────────────────────────────────── */
static int                s_sock        = -1;
static volatile bool      s_connected   = false;
static SemaphoreHandle_t  s_mutex       = NULL;
static TaskHandle_t       s_task_handle = NULL;
static uint16_t           s_packet_id   = 1;

static char s_topic_base[256];
static char s_topic_wildcard_cmd[256];

/* ══════════════════════════════════════════════════════════════════════════════
 * Low-level socket helpers
 * ══════════════════════════════════════════════════════════════════════════════*/

static int sock_write(const uint8_t *buf, int len)
{
    int sent = 0;
    while (sent < len) {
        int r = send(s_sock, buf + sent, len - sent, 0);
        if (r <= 0) return -1;
        sent += r;
    }
    return sent;
}

/* Read with millisecond timeout.  Returns bytes read, 0 = closed, <0 = error. */
static int sock_read(uint8_t *buf, int len, int timeout_ms)
{
    struct timeval tv = {
        .tv_sec  = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000,
    };
    setsockopt(s_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return recv(s_sock, buf, len, 0);
}

/* ══════════════════════════════════════════════════════════════════════════════
 * MQTT 3.1.1 packet builders
 * ══════════════════════════════════════════════════════════════════════════════*/

/* Encode variable-length "remaining length" field. Returns bytes written. */
static int encode_remaining_len(uint8_t *buf, int len)
{
    int i = 0;
    do {
        buf[i] = (uint8_t)(len & 0x7F);
        len >>= 7;
        if (len > 0) buf[i] |= 0x80;
        i++;
    } while (len > 0);
    return i;
}

/* Write a UTF-8 string with 2-byte big-endian length prefix. */
static int write_str(uint8_t *buf, const char *s)
{
    uint16_t l = (uint16_t)strlen(s);
    buf[0] = (uint8_t)(l >> 8);
    buf[1] = (uint8_t)(l & 0xFF);
    memcpy(buf + 2, s, l);
    return 2 + l;
}

/* ── CONNECT ─────────────────────────────────────────────────────────────── */
static esp_err_t mqtt_send_connect(void)
{
    const char *client_id = CONFIG_EXAMPLE_MQTT_CLIENT_ID;
    const char *username  = CONFIG_EXAMPLE_MQTT_USERNAME;
    const char *password  = CONFIG_EXAMPLE_MQTT_PASSWORD;

    /*  Variable header (10 bytes):
     *    4-byte protocol name "MQTT", 1-byte level (4), 1-byte flags, 2-byte keepalive
     *  Payload:
     *    client_id, username, password — each 2-byte-length-prefixed
     */
    int payload_len = 2 + strlen(client_id)
                    + 2 + strlen(username)
                    + 2 + strlen(password);
    int remaining   = 10 + payload_len;

    uint8_t buf[512];
    int idx = 0;

    buf[idx++] = 0x10;                                    /* CONNECT          */
    idx += encode_remaining_len(buf + idx, remaining);

    /* Protocol Name */
    buf[idx++] = 0x00; buf[idx++] = 0x04;
    buf[idx++] = 'M';  buf[idx++] = 'Q';
    buf[idx++] = 'T';  buf[idx++] = 'T';

    buf[idx++] = 0x04;  /* Protocol Level 4 = MQTT 3.1.1 */
    buf[idx++] = 0xC2;  /* Connect Flags: username | password | clean_session */

    buf[idx++] = (uint8_t)(MQTT_KEEPALIVE_SEC >> 8);
    buf[idx++] = (uint8_t)(MQTT_KEEPALIVE_SEC & 0xFF);

    idx += write_str(buf + idx, client_id);
    idx += write_str(buf + idx, username);
    idx += write_str(buf + idx, password);

    return sock_write(buf, idx) == idx ? ESP_OK : ESP_FAIL;
}

/* ── SUBSCRIBE ───────────────────────────────────────────────────────────── */
static esp_err_t mqtt_send_subscribe(const char *topic, uint8_t qos)
{
    int topic_len = (int)strlen(topic);
    int remaining = 2 + 2 + topic_len + 1;   /* pid + len + topic + qos */

    uint8_t buf[512];
    int idx = 0;

    buf[idx++] = 0x82;  /* SUBSCRIBE, QoS-1 required by spec */
    idx += encode_remaining_len(buf + idx, remaining);

    uint16_t pid = s_packet_id++;
    buf[idx++] = (uint8_t)(pid >> 8);
    buf[idx++] = (uint8_t)(pid & 0xFF);

    idx += write_str(buf + idx, topic);
    buf[idx++] = qos;

    return sock_write(buf, idx) == idx ? ESP_OK : ESP_FAIL;
}

/* ── PUBLISH (thread-safe) ───────────────────────────────────────────────── */
static esp_err_t mqtt_send_publish(const char *topic, const char *payload,
                                   uint8_t qos, bool retain)
{
    if (s_sock < 0 || !s_connected) return ESP_FAIL;

    int topic_len   = (int)strlen(topic);
    int payload_len = (int)strlen(payload);
    int remaining   = 2 + topic_len + payload_len + (qos > 0 ? 2 : 0);

    uint8_t buf[1024];
    int idx = 0;

    uint8_t fixed = 0x30 | (qos << 1) | (retain ? 0x01 : 0x00);
    buf[idx++] = fixed;
    idx += encode_remaining_len(buf + idx, remaining);

    idx += write_str(buf + idx, topic);

    if (qos > 0) {
        uint16_t pid = s_packet_id++;
        buf[idx++] = (uint8_t)(pid >> 8);
        buf[idx++] = (uint8_t)(pid & 0xFF);
    }

    memcpy(buf + idx, payload, payload_len);
    idx += payload_len;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int r = sock_write(buf, idx);
    xSemaphoreGive(s_mutex);

    return (r == idx) ? ESP_OK : ESP_FAIL;
}

/* ── PINGREQ ─────────────────────────────────────────────────────────────── */
static esp_err_t mqtt_send_pingreq(void)
{
    uint8_t buf[2] = {0xC0, 0x00};
    return sock_write(buf, 2) == 2 ? ESP_OK : ESP_FAIL;
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Incoming PUBLISH handler
 * ══════════════════════════════════════════════════════════════════════════════*/

static void handle_publish(const uint8_t *pkt, int pkt_len)
{
    if (pkt_len < 4) return;

    /* Skip fixed header + remaining-length bytes */
    int idx = 1;
    while (idx < pkt_len && (pkt[idx] & 0x80)) idx++;
    idx++;  /* last remaining-length byte */

    /* Topic */
    if (idx + 2 > pkt_len) return;
    int topic_len = (pkt[idx] << 8) | pkt[idx + 1];
    idx += 2;
    if (idx + topic_len > pkt_len) return;

    char topic[256];
    int tl = topic_len < 255 ? topic_len : 255;
    memcpy(topic, pkt + idx, tl);
    topic[tl] = '\0';
    idx += topic_len;

    /* Skip packet-id for QoS > 0 */
    uint8_t qos = (pkt[0] >> 1) & 0x03;
    if (qos > 0) idx += 2;

    /* Payload */
    int pl = pkt_len - idx;
    if (pl <= 0) return;
    char payload[512];
    int pll = pl < 511 ? pl : 511;
    memcpy(payload, pkt + idx, pll);
    payload[pll] = '\0';

    ESP_LOGI(TAG, "MQTT Data on topic: %s", topic);

    /* Route to the correct control handler */
    char *cp = strstr(topic, "/control/");
    if (!cp) return;
    cp += 9;
    char *sp = strstr(cp, "/set");
    if (!sp) return;
    *sp = '\0';

    if (strcmp(cp, "light_1") == 0 || strcmp(cp, "led") == 0) {
        cJSON *root = cJSON_Parse(payload);
        if (root) {
            cJSON *item = cJSON_GetObjectItem(root, "value");
            if (item && cJSON_IsNumber(item))
                light_ctrl_set(item->valueint == 1);
            cJSON_Delete(root);
        }
    } else if (strcmp(cp, "fan") == 0) {
        cJSON *root = cJSON_Parse(payload);
        if (root) {
            cJSON *item = cJSON_GetObjectItem(root, "value");
            if (item && cJSON_IsNumber(item))
                fan_ctrl_set_speed((uint8_t)item->valueint);
            cJSON_Delete(root);
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * TCP connect helper  (uses getaddrinfo directly — no URI parsing)
 * ══════════════════════════════════════════════════════════════════════════════*/

static int tcp_connect_to_broker(void)
{
    struct addrinfo hints = {
        .ai_family   = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res = NULL;

    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%d", MQTT_PORT);

    /* Force Google public DNS to bypass router DNS failures */
    ip_addr_t dns8;
    ipaddr_aton("8.8.8.8", &dns8);
    dns_setserver(0, &dns8);

    ESP_LOGI(TAG, "Resolving host: %s", MQTT_HOST);
    int err = getaddrinfo(MQTT_HOST, port_str, &hints, &res);
    if (err != 0 || res == NULL) {
        ESP_LOGE(TAG, "DNS lookup failed for %s (err=%d)", MQTT_HOST, err);
        if (res) freeaddrinfo(res);
        return -1;
    }

    int sock = socket(res->ai_family, res->ai_socktype, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket creation failed: errno %d", errno);
        freeaddrinfo(res);
        return -1;
    }

    /* 10-second connect timeout */
    struct timeval tv = {.tv_sec = 10, .tv_usec = 0};
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "TCP connect failed to %s:%d (errno %d)",
                 MQTT_HOST, MQTT_PORT, errno);
        close(sock);
        freeaddrinfo(res);
        return -1;
    }

    freeaddrinfo(res);
    ESP_LOGI(TAG, "TCP connected to %s:%d ✅", MQTT_HOST, MQTT_PORT);
    return sock;
}

/* ══════════════════════════════════════════════════════════════════════════════
 * MQTT background task
 * ══════════════════════════════════════════════════════════════════════════════*/

static void mqtt_task(void *arg)
{
    uint8_t recv_buf[MQTT_RECV_BUF_SIZE];

    while (1) {
        /* ── Connect phase ───────────────────────────────────────────── */
        s_connected = false;
        if (s_sock >= 0) { close(s_sock); s_sock = -1; }

        s_sock = tcp_connect_to_broker();
        if (s_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        /* Send MQTT CONNECT */
        if (mqtt_send_connect() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send CONNECT packet");
            close(s_sock); s_sock = -1;
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        /* Wait for CONNACK */
        int r = sock_read(recv_buf, 4, 8000);
        if (r < 4 || recv_buf[0] != 0x20) {
            ESP_LOGE(TAG, "No valid CONNACK (r=%d byte0=0x%02X)",
                     r, r > 0 ? recv_buf[0] : 0xFF);
            close(s_sock); s_sock = -1;
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        if (recv_buf[3] != 0x00) {
            ESP_LOGE(TAG, "Broker refused connection, code=%d", recv_buf[3]);
            close(s_sock); s_sock = -1;
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        s_connected = true;
        ESP_LOGI(TAG, "⭐⭐⭐ MQTT CONNECTED TO CIRCUITDIGEST ⭐⭐⭐");

        /* Subscribe & publish initial state */
        mqtt_send_subscribe(s_topic_wildcard_cmd, 0);

        char online_topic[256];
        snprintf(online_topic, sizeof(online_topic), "%s/state/online", s_topic_base);
        mqtt_send_publish(online_topic, "{\"online\": 1}", 1, true);

        app_mqtt_publish_state(light_ctrl_get());
        app_mqtt_publish_fan_state(fan_ctrl_get_power(), fan_ctrl_get_speed());

        TickType_t last_ping = xTaskGetTickCount();

        /* ── Receive loop ────────────────────────────────────────────── */
        while (s_connected) {

            /* Send keepalive PINGREQ */
            if ((xTaskGetTickCount() - last_ping) >= pdMS_TO_TICKS(MQTT_PING_INTERVAL * 1000)) {
                if (mqtt_send_pingreq() != ESP_OK) {
                    ESP_LOGW(TAG, "PINGREQ failed — reconnecting");
                    break;
                }
                last_ping = xTaskGetTickCount();
                ESP_LOGD(TAG, "PINGREQ sent");
            }

            /* Non-blocking read (200 ms window) */
            r = sock_read(recv_buf, sizeof(recv_buf), 200);

            if (r == 0) {
                ESP_LOGW(TAG, "Broker closed connection");
                break;
            } else if (r < 0) {
                int e = errno;
                if (e == EAGAIN || e == EWOULDBLOCK) continue;  /* timeout OK */
                ESP_LOGW(TAG, "recv error errno=%d — reconnecting", e);
                break;
            }

            /* Dispatch by packet type */
            switch (recv_buf[0] & 0xF0) {
                case 0x30: handle_publish(recv_buf, r); break;
                case 0xD0: ESP_LOGD(TAG, "PINGRESP");   break;
                case 0x90: ESP_LOGI(TAG, "SUBACK — subscribed to controls"); break;
                case 0x40: /* PUBACK — QoS 1 ack */     break;
                default:
                    ESP_LOGD(TAG, "Unhandled packet type 0x%02X", recv_buf[0] & 0xF0);
                    break;
            }
        }

        s_connected = false;
        ESP_LOGW(TAG, "❗❗❗ MQTT DISCONNECTED ❗❗❗");
        if (s_sock >= 0) { close(s_sock); s_sock = -1; }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Public API
 * ══════════════════════════════════════════════════════════════════════════════*/

esp_err_t app_mqtt_init(void)
{
    if (s_task_handle) return ESP_OK;

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    snprintf(s_topic_base, sizeof(s_topic_base),
             "cd/users/%s/devices/%s",
             CONFIG_EXAMPLE_MQTT_USER_ID, CONFIG_EXAMPLE_MQTT_DEVICE_ID);
    snprintf(s_topic_wildcard_cmd, sizeof(s_topic_wildcard_cmd),
             "%s/control/+/set", s_topic_base);

    xTaskCreate(mqtt_task, "mqtt_task", MQTT_STACK_SIZE, NULL,
                MQTT_TASK_PRIO, &s_task_handle);
    ESP_LOGI(TAG, "MQTT task started");
    return ESP_OK;
}

bool app_mqtt_is_connected(void) { return s_connected; }

/* ── Internal publish helper ─────────────────────────────────────────────── */
static esp_err_t app_mqtt_publish(const char *subtopic, const char *payload)
{
    char topic[512];
    snprintf(topic, sizeof(topic), "%s/%s", s_topic_base, subtopic);
    return mqtt_send_publish(topic, payload, 1, true);
}

/* ── Publish light state ─────────────────────────────────────────────────── */
esp_err_t app_mqtt_publish_state(bool state)
{
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"value\": %d}", state ? 1 : 0);
    return app_mqtt_publish("control/light_1/get", payload);
}

/* ── Publish sensor telemetry ────────────────────────────────────────────── */
esp_err_t app_mqtt_publish_sensor_data(float temp, float hum, bool presence)
{
    ESP_LOGI(TAG, "Publishing Telemetry: T=%.2f, H=%.2f, P=%d",
             temp, hum, presence);

    char payload[256];

    snprintf(payload, sizeof(payload),
             "{\"value\": %.2f, \"unit\": \"C\"}", temp);
    app_mqtt_publish("sensor/temperature", payload);

    snprintf(payload, sizeof(payload),
             "{\"value\": %.2f, \"unit\": \"%%\"}", hum);
    app_mqtt_publish("sensor/humidity", payload);

    snprintf(payload, sizeof(payload),
             "{\"value\": %d}", presence ? 1 : 0);
    app_mqtt_publish("sensor/presence", payload);

    return ESP_OK;
}

/* ── Publish fan state ───────────────────────────────────────────────────── */
esp_err_t app_mqtt_publish_fan_state(bool on, uint8_t speed)
{
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"value\": %d}", on ? speed : 0);
    return app_mqtt_publish("control/fan/get", payload);
}

esp_err_t app_mqtt_publish_event(const char *key, const char *payload)
{
    (void)key; (void)payload;
    return ESP_OK;
}

void app_mqtt_report_sensors_now(void)
{
    float temp = 0, hum = 0;
    bool presence = app_sensor_get_presence();
    esp_err_t err = app_sensor_get_values(&temp, &hum);
    ESP_LOGI(TAG, "Voice report: T=%.2f, H=%.2f, P=%d (err=%d)",
             temp, hum, presence, err);
    app_mqtt_publish_sensor_data(temp, hum, presence);
}
