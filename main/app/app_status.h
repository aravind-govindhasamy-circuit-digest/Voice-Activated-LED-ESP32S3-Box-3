#pragma once

#include <stdbool.h>

typedef struct {
  char ssid[32];
  char ip[16];
  bool wifi_connected;
  bool mqtt_connected;
} app_status_t;

void app_status_update_wifi(const char *ssid, const char *ip, bool connected);
void app_status_update_mqtt(bool connected);
app_status_t app_status_get(void);
