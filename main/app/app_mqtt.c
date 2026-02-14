#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "mqtt_client.h"
#include "sdkconfig.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_mqtt.h"
#include "app_status.h"
#include "fan_ctrl.h"
#include "light_ctrl.h"


static const char *TAG = "app_mqtt";

static esp_mqtt_client_handle_t s_client = NULL;

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

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32,
           base, event_id);
  esp_mqtt_event_handle_t event = event_data;
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;

  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    app_status_update_mqtt(true);
    msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_SET, 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    esp_mqtt_client_subscribe(client, MQTT_TOPIC_FAN_SET, 0);
    esp_mqtt_client_subscribe(client, MQTT_TOPIC_FAN_SPEED_SET, 0);

    // Publish initial states
    app_mqtt_publish_state(light_ctrl_get());
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
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
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);

    if (strncmp(event->topic, MQTT_TOPIC_SET, event->topic_len) == 0) {
      if (strncmp(event->data, "on", event->data_len) == 0) {
        light_ctrl_set(true);
      } else if (strncmp(event->data, "off", event->data_len) == 0) {
        light_ctrl_set(false);
      }
    } else if (strncmp(event->topic, MQTT_TOPIC_FAN_SET, event->topic_len) ==
               0) {
      if (strncmp(event->data, "on", event->data_len) == 0) {
        fan_ctrl_set_power(true);
      } else if (strncmp(event->data, "off", event->data_len) == 0) {
        fan_ctrl_set_power(false);
      }
    } else if (strncmp(event->topic, MQTT_TOPIC_FAN_SPEED_SET,
                       event->topic_len) == 0) {
      char buf[16] = {0};
      int len = (event->data_len < 15) ? event->data_len : 15;
      memcpy(buf, event->data, len);
      int speed = atoi(buf);
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
  esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = MQTT_BROKER_URL,
      .credentials.username = CONFIG_EXAMPLE_MQTT_USERNAME,
      .credentials.authentication.password = CONFIG_EXAMPLE_MQTT_PASSWORD,
  };

  s_client = esp_mqtt_client_init(&mqtt_cfg);
  /* The last argument may be used to pass data to the handler, in this example
   * mqtt_event_handler */
  esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler,
                                 NULL);
  esp_mqtt_client_start(s_client);

  return ESP_OK;
}

esp_err_t app_mqtt_publish_state(bool state) {
  if (s_client == NULL) {
    return ESP_FAIL;
  }

  const char *payload = state ? "on" : "off";
  int msg_id =
      esp_mqtt_client_publish(s_client, MQTT_TOPIC_STATE, payload, 0, 1, 0);
  ESP_LOGI(TAG, "sent publish state successful, msg_id=%d", msg_id);

  return ESP_OK;
}

esp_err_t app_mqtt_publish_sensor_data(float temp, float hum, bool presence) {
  if (s_client == NULL) {
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Publishing Sensor Data: T=%.2f, H=%.2f, P=%d", temp, hum,
           presence);

  char buf[32];
  snprintf(buf, sizeof(buf), "%.2f", temp);
  esp_mqtt_client_publish(s_client, MQTT_TOPIC_TEMP, buf, 0, 1, 0);

  snprintf(buf, sizeof(buf), "%.2f", hum);
  esp_mqtt_client_publish(s_client, MQTT_TOPIC_HUM, buf, 0, 1, 0);

  const char *presence_str = presence ? "on" : "off";
  esp_mqtt_client_publish(s_client, MQTT_TOPIC_PRESENCE, presence_str, 0, 1, 0);

  return ESP_OK;
}
