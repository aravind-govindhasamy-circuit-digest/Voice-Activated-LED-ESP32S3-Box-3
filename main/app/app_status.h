#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  char ssid[32];
  char ip[16];
  bool wifi_connected;
  bool mqtt_connected;
  float last_temp;
  float last_hum;
  bool last_presence;
  bool sensor_valid;
  uint32_t mqtt_tx_count;
  uint32_t mqtt_rx_count;
  char mqtt_last_topic[64];
  char mqtt_last_payload[64];
} app_status_t;

void app_status_update_wifi(const char *ssid, const char *ip, bool connected);
void app_status_update_mqtt(bool connected);
void app_status_update_sensor(float temp, float hum, bool presence);
void app_status_note_mqtt_tx(const char *topic, const char *payload);
void app_status_note_mqtt_rx(const char *topic, const char *payload);
app_status_t app_status_get(void);
