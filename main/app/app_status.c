#include "app_status.h"
#include <string.h>

static app_status_t s_status = {0};

static void safe_copy(char *dst, size_t dst_len, const char *src) {
  if (!dst || dst_len == 0) {
    return;
  }

  if (!src) {
    dst[0] = '\0';
    return;
  }

  strncpy(dst, src, dst_len - 1);
  dst[dst_len - 1] = '\0';
}

void app_status_update_wifi(const char *ssid, const char *ip, bool connected) {
  if (ssid)
    safe_copy(s_status.ssid, sizeof(s_status.ssid), ssid);
  if (ip)
    safe_copy(s_status.ip, sizeof(s_status.ip), ip);
  s_status.wifi_connected = connected;
}

void app_status_update_mqtt(bool connected) {
  s_status.mqtt_connected = connected;
}

void app_status_update_sensor(float temp, float hum, bool presence) {
  s_status.last_temp = temp;
  s_status.last_hum = hum;
  s_status.last_presence = presence;
  s_status.sensor_valid = true;
}

void app_status_note_mqtt_tx(const char *topic, const char *payload) {
  s_status.mqtt_tx_count++;
  safe_copy(s_status.mqtt_last_topic, sizeof(s_status.mqtt_last_topic), topic);
  safe_copy(s_status.mqtt_last_payload, sizeof(s_status.mqtt_last_payload),
            payload);
}

void app_status_note_mqtt_rx(const char *topic, const char *payload) {
  s_status.mqtt_rx_count++;
  safe_copy(s_status.mqtt_last_topic, sizeof(s_status.mqtt_last_topic), topic);
  safe_copy(s_status.mqtt_last_payload, sizeof(s_status.mqtt_last_payload),
            payload);
}

app_status_t app_status_get(void) { return s_status; }
