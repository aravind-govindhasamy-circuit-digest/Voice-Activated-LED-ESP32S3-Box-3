#include "esp_check.h"
#include "esp_event.h"
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

#include "cJSON.h"

#include "app_mqtt.h"
#include "app_status.h"
#include "fan_ctrl.h"
#include "light_ctrl.h"

static const char *TAG = "app_mqtt";

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;

#define MQTT_BROKER_URL CONFIG_EXAMPLE_MQTT_BROKER_URL

// Topic buffers for Functional Architecture
static char s_topic_online[128];
static char s_topic_sensor_temp[128];
static char s_topic_sensor_hum[128];
static char s_topic_sensor_presence[128];
static char s_topic_actuator_light_cmd[128];
static char s_topic_actuator_light_status[128];
static char s_topic_actuator_fan_cmd[128];
static char s_topic_actuator_fan_status[128];
static char s_topic_event[128];

#ifndef CONFIG_EXAMPLE_MQTT_CLIENT_ID
#define CONFIG_EXAMPLE_MQTT_CLIENT_ID "CD_BOX_3_V1"
#endif

// We want to extract the hostname from mqtt://hostname
static const char *get_mqtt_hostname(void) {
  const char *url = MQTT_BROKER_URL;
  if (strncmp(url, "mqtt://", 7) == 0) {
    return url + 7;
  }
  if (strncmp(url, "mqtts://", 8) == 0) {
    return url + 8;
  }
  return url;
}

#ifndef CONFIG_EXAMPLE_MQTT_DEVICE_ID
#define CONFIG_EXAMPLE_MQTT_DEVICE_ID "32ea3b2c-1c49-4fcc-87d9-db8c11148574"
#endif

#ifndef CONFIG_EXAMPLE_MQTT_USER_ID
#define CONFIG_EXAMPLE_MQTT_USER_ID "16e6f1d6-9ac5-4825-bec0-c79107d09490"
#endif

#ifndef CONFIG_EXAMPLE_MQTT_PORT
#define CONFIG_EXAMPLE_MQTT_PORT 1883
#endif

static bool mqtt_topic_equals(esp_mqtt_event_handle_t event,
                              const char *topic) {
  if (!event || !topic) {
    return false;
  }

  size_t expected_len = strlen(topic);
  return ((size_t)event->topic_len == expected_len) &&
         (strncmp(event->topic, topic, expected_len) == 0);
}

static bool mqtt_data_equals(esp_mqtt_event_handle_t event,
                             const char *literal) {
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
  mqtt_copy_segment(topic, sizeof(topic), event->topic,
                    (size_t)event->topic_len);
  mqtt_copy_segment(payload, sizeof(payload), event->data,
                    (size_t)event->data_len);
  app_status_note_mqtt_rx(topic, payload);
}

static esp_err_t mqtt_publish_with_status(const char *topic,
                                          const char *payload, int qos,
                                          int retain) {
  if (!topic || !payload || s_client == NULL || !s_connected) {
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Publishing to %s: %s", topic, payload);
  int msg_id =
      esp_mqtt_client_publish(s_client, topic, payload, 0, qos, retain);
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
    msg_id = esp_mqtt_client_subscribe(client, s_topic_actuator_light_cmd, 0);
    ESP_LOGI(TAG, "subscribe %s msg_id=%d", s_topic_actuator_light_cmd, msg_id);
    msg_id = esp_mqtt_client_subscribe(client, s_topic_actuator_fan_cmd, 0);
    ESP_LOGI(TAG, "subscribe %s msg_id=%d", s_topic_actuator_fan_cmd, msg_id);

    /* Publish online status and retained shadow */
    esp_mqtt_client_publish(client, s_topic_online, "{\"online\": 1}", 0, 1, 1);
    (void)app_mqtt_publish_state(light_ctrl_get());
    (void)app_mqtt_publish_fan_state(fan_ctrl_get_power(),
                                     fan_ctrl_get_speed());
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    s_connected = false;
    app_status_update_mqtt(false);
    /* Attempt to reconnect when disconnected */
    esp_mqtt_client_reconnect(client);
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

    char payload_buf[128];
    mqtt_copy_segment(payload_buf, sizeof(payload_buf), event->data,
                      (size_t)event->data_len);
    cJSON *root = cJSON_Parse(payload_buf);

    if (root) {
      if (mqtt_topic_equals(event, s_topic_actuator_light_cmd)) {
        cJSON *light_item = cJSON_GetObjectItem(root, "light_1");
        if (light_item && cJSON_IsNumber(light_item)) {
          (void)light_ctrl_set(light_item->valueint == 1);
        }
      } else if (mqtt_topic_equals(event, s_topic_actuator_fan_cmd)) {
        cJSON *fan_item = cJSON_GetObjectItem(root, "fan");
        if (fan_item && cJSON_IsNumber(fan_item)) {
          int speed = fan_item->valueint;
          if (speed < 0)
            speed = 0;
          if (speed > 100)
            speed = 100;
          fan_ctrl_set_speed((uint8_t)speed);
        }
      }
      cJSON_Delete(root);
    } else {
      // Fallback for simple string payloads like "on"/"off" (optional)
      if (mqtt_topic_equals(event, s_topic_actuator_light_cmd)) {
        if (mqtt_data_equals(event, "on"))
          (void)light_ctrl_set(true);
        else if (mqtt_data_equals(event, "off"))
          (void)light_ctrl_set(false);
      } else if (mqtt_topic_equals(event, s_topic_actuator_fan_cmd)) {
        if (mqtt_data_equals(event, "on"))
          fan_ctrl_set_power(true);
        else if (mqtt_data_equals(event, "off"))
          fan_ctrl_set_power(false);
      }
    }
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
      ESP_LOGE(TAG, "Last error: %s",
               esp_err_to_name(event->error_handle->esp_tls_last_esp_err));
      ESP_LOGE(TAG, "Last tls stack error: 0x%x",
               event->error_handle->esp_tls_stack_err);
      ESP_LOGE(TAG, "Last tls error code: 0x%x",
               event->error_handle->esp_transport_sock_errno);
    }
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

  // Functional architecture:
  // cd/users/{user_id}/devices/{device_id}/{category}/{key}
  snprintf(s_topic_online, sizeof(s_topic_online),
           "cd/users/%s/devices/%s/state/online", CONFIG_EXAMPLE_MQTT_USER_ID,
           CONFIG_EXAMPLE_MQTT_DEVICE_ID);

  snprintf(s_topic_sensor_temp, sizeof(s_topic_sensor_temp),
           "cd/users/%s/devices/%s/sensor/temperature",
           CONFIG_EXAMPLE_MQTT_USER_ID, CONFIG_EXAMPLE_MQTT_DEVICE_ID);
  snprintf(s_topic_sensor_hum, sizeof(s_topic_sensor_hum),
           "cd/users/%s/devices/%s/sensor/humidity",
           CONFIG_EXAMPLE_MQTT_USER_ID, CONFIG_EXAMPLE_MQTT_DEVICE_ID);
  snprintf(s_topic_sensor_presence, sizeof(s_topic_sensor_presence),
           "cd/users/%s/devices/%s/sensor/presence",
           CONFIG_EXAMPLE_MQTT_USER_ID, CONFIG_EXAMPLE_MQTT_DEVICE_ID);

  snprintf(s_topic_actuator_light_cmd, sizeof(s_topic_actuator_light_cmd),
           "cd/users/%s/devices/%s/actuator/light_1/command",
           CONFIG_EXAMPLE_MQTT_USER_ID, CONFIG_EXAMPLE_MQTT_DEVICE_ID);
  snprintf(s_topic_actuator_light_status, sizeof(s_topic_actuator_light_status),
           "cd/users/%s/devices/%s/actuator/light_1/status",
           CONFIG_EXAMPLE_MQTT_USER_ID, CONFIG_EXAMPLE_MQTT_DEVICE_ID);

  snprintf(s_topic_actuator_fan_cmd, sizeof(s_topic_actuator_fan_cmd),
           "cd/users/%s/devices/%s/actuator/fan/command",
           CONFIG_EXAMPLE_MQTT_USER_ID, CONFIG_EXAMPLE_MQTT_DEVICE_ID);
  snprintf(s_topic_actuator_fan_status, sizeof(s_topic_actuator_fan_status),
           "cd/users/%s/devices/%s/actuator/fan/status",
           CONFIG_EXAMPLE_MQTT_USER_ID, CONFIG_EXAMPLE_MQTT_DEVICE_ID);

  snprintf(s_topic_event, sizeof(s_topic_event),
           "cd/users/%s/devices/%s/event/log", CONFIG_EXAMPLE_MQTT_USER_ID,
           CONFIG_EXAMPLE_MQTT_DEVICE_ID);

  ESP_LOGI(TAG, "Topic Home: cd/users/%s/devices/%s/...",
           CONFIG_EXAMPLE_MQTT_USER_ID, CONFIG_EXAMPLE_MQTT_DEVICE_ID);
  ESP_LOGI(TAG, "Full Actuator Cmd Path: %s", s_topic_actuator_light_cmd);
  ESP_LOGI(TAG, "Full Sensor Temp Path: %s", s_topic_sensor_temp);

  ESP_LOGI(TAG, "Configuring MQTT: Host=%s, Port=%d, ClientID=%s",
           get_mqtt_hostname(), CONFIG_EXAMPLE_MQTT_PORT,
           CONFIG_EXAMPLE_MQTT_CLIENT_ID);

  esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = MQTT_BROKER_URL,
      .credentials.username = CONFIG_EXAMPLE_MQTT_USERNAME,
      .credentials.authentication.password = CONFIG_EXAMPLE_MQTT_PASSWORD,
      /* client_id is left empty so the library generates a unique MAC-based ID
       */
      .session.last_will.topic = s_topic_online,
      .session.last_will.msg = "{\"online\": 0}",
      .session.last_will.qos = 1,
      .session.last_will.retain = 1,
      .session.keepalive = 60,
      .network.timeout_ms = 30000,
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
  const char *payload = state ? "{\"light_1\": 1}" : "{\"light_1\": 0}";
  return mqtt_publish_with_status(s_topic_actuator_light_status, payload, 1, 0);
}

esp_err_t app_mqtt_publish_sensor_data(float temp, float hum, bool presence) {
  if (s_client == NULL || !s_connected) {
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Publishing Sensor Data: T=%.2f, H=%.2f, P=%d", temp, hum,
           presence);

  char buf[64];
  snprintf(buf, sizeof(buf), "{\"temperature\": %.2f}", temp);
  ESP_RETURN_ON_ERROR(mqtt_publish_with_status(s_topic_sensor_temp, buf, 1, 0),
                      TAG, "publish temp failed");

  snprintf(buf, sizeof(buf), "{\"humidity\": %.2f}", hum);
  ESP_RETURN_ON_ERROR(mqtt_publish_with_status(s_topic_sensor_hum, buf, 1, 0),
                      TAG, "publish hum failed");

  snprintf(buf, sizeof(buf), "{\"presence\": %d}", presence ? 1 : 0);
  ESP_RETURN_ON_ERROR(
      mqtt_publish_with_status(s_topic_sensor_presence, buf, 1, 0), TAG,
      "publish presence failed");

  return ESP_OK;
}

esp_err_t app_mqtt_publish_fan_state(bool on, uint8_t speed) {
  char buf[64];

  int val = on ? speed : 0;
  snprintf(buf, sizeof(buf), "{\"fan\": %d}", val);
  ESP_RETURN_ON_ERROR(
      mqtt_publish_with_status(s_topic_actuator_fan_status, buf, 1, 0), TAG,
      "publish fan status failed");

  return ESP_OK;
}

esp_err_t app_mqtt_publish_event(const char *key, const char *payload) {
  char topic[128];

  snprintf(topic, sizeof(topic), "cd/users/%s/devices/%s/event/%s",
           CONFIG_EXAMPLE_MQTT_USER_ID, CONFIG_EXAMPLE_MQTT_DEVICE_ID, key);

  return mqtt_publish_with_status(topic, payload, 1, 0);
}
