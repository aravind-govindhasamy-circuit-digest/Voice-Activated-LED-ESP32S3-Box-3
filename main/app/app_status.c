#include "app_status.h"
#include <string.h>

static app_status_t s_status = {0};

void app_status_update_wifi(const char *ssid, const char *ip, bool connected) {
  if (ssid)
    strncpy(s_status.ssid, ssid, sizeof(s_status.ssid) - 1);
  if (ip)
    strncpy(s_status.ip, ip, sizeof(s_status.ip) - 1);
  s_status.wifi_connected = connected;
}

void app_status_update_mqtt(bool connected) {
  s_status.mqtt_connected = connected;
}

app_status_t app_status_get(void) { return s_status; }
