#include "esp_event.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "mqtt_client.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_mqtt.h"
#include "app_status.h"
#include "fan_ctrl.h"
#include "light_ctrl.h"


static const char *TAG = "app_mqtt";

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;

#define MQTT_BROKER_URL CONFIG_EXAMPLE_MQTT_BROKER_URL
#define MQTT_TOPIC_SET CONFIG_EXAMPLE_MQTT_TOPIC_SET
#define MQTT_TOPIC_STATE CONFIG_EXAMPLE_MQTT_TOPIC_STATE

#define MQTT_TOPIC_TEMP CONFIG_EXAMPLE_MQTT_TOPIC_TEMP
#define MQTT_TOPIC_HUM CONFIG_EXAMPLE_MQTT_TOPIC_HUM
#define MQTT_TOPIC_PRESENCE CONFIG_EXAMPLE_MQTT_TOPIC_PRESENCE

#define MQTT_TOPIC_FAN_SET "devices/box/fan/set"
#define MQTT_TOPIC_FAN_SPEED_SET "devices/box/fan/speed/set"
#define MQTT_TOPIC_FAN_STATE "devices/box/fan/state"
#define MQTT_TOPIC_FAN_SPEED "devices/box/fan/speed"

static bool mqtt_topic_equals(esp_mqtt_event_handle_t event, const char *topic) {
  if (!event || !topic) {
    return false;
  }

  size_t expected_len = strlen(topic);
  return ((size_t)event->topic_len == expected_len) &&
         (strncmp(event->topic, topic, expected_len) == 0);
}

static bool mqtt_data_equals(esp_mqtt_event_handle_t event, const char *literal) {
  if (!event || !literal) {
    return false;
  }

  size_t literal_len = strlen(literal);
  return ((size_t)event->data_len == literal_len) &&
         (strncmp(event->data, literal, literal_len) == 0);
}

static void mqtt_copy_segment(char *dst, size_t dst_len, const char *src,
                              size_t src_len) {
  if (!dst || dst_len == 0) {
    return;
  }
  if (!src) {
    dst[0] = '\0';
    return;
  }

  size_t to_copy = src_len;
  if (to_copy >= dst_len) {
    to_copy = dst_len - 1;
  }
  memcpy(dst, src, to_copy);
  dst[to_copy] = '\0';
}

static void mqtt_note_rx_event(esp_mqtt_event_handle_t event) {
  char topic[64];
  char payload[64];
  mqtt_copy_segment(topic, sizeof(topic), event->topic, (size_t)event->topic_len);
  mqtt_copy_segment(payload, sizeof(payload), event->data,
                    (size_t)event->data_len);
  app_status_note_mqtt_rx(topic, payload);
}

static esp_err_t mqtt_publish_with_status(const char *topic, const char *payload,
                                          int qos, int retain) {
  if (!topic || !payload || s_client == NULL || !s_connected) {
    return ESP_FAIL;
  }

  int msg_id = esp_mqtt_client_publish(s_client, topic, payload, 0, qos, retain);
  if (msg_id < 0) {
    return ESP_FAIL;
  }

  app_status_note_mqtt_tx(topic, payload);
  return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  (void)handler_args;
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32,
           base, event_id);
  esp_mqtt_event_handle_t event = event_data;
  esp_mqtt_client_handle_t client = event->client;
  int msg_id = -1;

  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    s_connected = true;
    app_status_update_mqtt(true);
    msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_SET, 0);
    ESP_LOGI(TAG, "subscribe %s msg_id=%d", MQTT_TOPIC_SET, msg_id);
    msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_FAN_SET, 0);
    ESP_LOGI(TAG, "subscribe %s msg_id=%d", MQTT_TOPIC_FAN_SET, msg_id);
    msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_FAN_SPEED_SET, 0);
    ESP_LOGI(TAG, "subscribe %s msg_id=%d", MQTT_TOPIC_FAN_SPEED_SET, msg_id);

    /* Publish retained shadow after reconnection */
    (void)app_mqtt_publish_state(light_ctrl_get());
    (void)app_mqtt_publish_fan_state(fan_ctrl_get_power(), fan_ctrl_get_speed());
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    s_connected = false;
    app_status_update_mqtt(false);
    break;
  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA topic=%.*s data=%.*s", event->topic_len,
             event->topic, event->data_len, event->data);
    mqtt_note_rx_event(event);

    if (mqtt_topic_equals(event, MQTT_TOPIC_SET)) {
      if (mqtt_data_equals(event, "on")) {
        (void)light_ctrl_set(true);
      } else if (mqtt_data_equals(event, "off")) {
        (void)light_ctrl_set(false);
      }
    } else if (mqtt_topic_equals(event, MQTT_TOPIC_FAN_SET)) {
      if (mqtt_data_equals(event, "on")) {
        fan_ctrl_set_power(true);
      } else if (mqtt_data_equals(event, "off")) {
        fan_ctrl_set_power(false);
      }
    } else if (mqtt_topic_equals(event, MQTT_TOPIC_FAN_SPEED_SET)) {
      char buf[16];
      mqtt_copy_segment(buf, sizeof(buf), event->data, (size_t)event->data_len);
      long speed = strtol(buf, NULL, 10);
      if (speed < 0) {
        speed = 0;
      } else if (speed > 100) {
        speed = 100;
      }
      fan_ctrl_set_speed((uint8_t)speed);
    }

    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
}

esp_err_t app_mqtt_init(void) {
  if (s_client) {
    return ESP_OK;
  }

  esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = MQTT_BROKER_URL,
      .credentials.username = CONFIG_EXAMPLE_MQTT_USERNAME,
      .credentials.authentication.password = CONFIG_EXAMPLE_MQTT_PASSWORD,
  };

  s_client = esp_mqtt_client_init(&mqtt_cfg);
  if (s_client == NULL) {
    return ESP_FAIL;
  }

  esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler,
                                 NULL);
  esp_mqtt_client_start(s_client);

  return ESP_OK;
}

bool app_mqtt_is_connected(void) { return s_connected; }

esp_err_t app_mqtt_publish_state(bool state) {
  const char *payload = state ? "on" : "off";
  return mqtt_publish_with_status(MQTT_TOPIC_STATE, payload, 1, 0);
}

esp_err_t app_mqtt_publish_sensor_data(float temp, float hum, bool presence) {
  if (s_client == NULL || !s_connected) {
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Publishing Sensor Data: T=%.2f, H=%.2f, P=%d", temp, hum,
           presence);

  char buf[32];
  snprintf(buf, sizeof(buf), "%.2f", temp);
  ESP_RETURN_ON_ERROR(mqtt_publish_with_status(MQTT_TOPIC_TEMP, buf, 1, 0), TAG,
                      "publish temp failed");

  snprintf(buf, sizeof(buf), "%.2f", hum);
  ESP_RETURN_ON_ERROR(mqtt_publish_with_status(MQTT_TOPIC_HUM, buf, 1, 0), TAG,
                      "publish hum failed");

  const char *presence_str = presence ? "on" : "off";
  ESP_RETURN_ON_ERROR(
      mqtt_publish_with_status(MQTT_TOPIC_PRESENCE, presence_str, 1, 0), TAG,
      "publish presence failed");

  return ESP_OK;
}

esp_err_t app_mqtt_publish_fan_state(bool on, uint8_t speed) {
  char speed_buf[8];
  const char *power = on ? "on" : "off";
  snprintf(speed_buf, sizeof(speed_buf), "%u", (unsigned)speed);

  ESP_RETURN_ON_ERROR(mqtt_publish_with_status(MQTT_TOPIC_FAN_STATE, power, 1, 0),
                      TAG, "publish fan power failed");
  ESP_RETURN_ON_ERROR(
      mqtt_publish_with_status(MQTT_TOPIC_FAN_SPEED, speed_buf, 1, 0), TAG,
      "publish fan speed failed");
  return ESP_OK;
}
