#ifndef __WIFI_OPS_H__
#define __WIFI_OPS_H__

#include <WiFi.h>

constexpr int g_max_ssid_len = 32;
constexpr int g_max_wifi_charpwd_len = 63;

typedef struct
{
    char ssid[g_max_ssid_len + 1];
    char pwd[g_max_wifi_charpwd_len + 1];
}wifi_ssid_pwd_s_t;

wl_status_t curr_wifi_status();
void wifi_init();

void scan_wifi_aps();
void printMacAddress();

wl_status_t connect_wifi(const char* ssid, const char* pwd, bool cpy = true);
wl_status_t connect_wifi(JsonObject ssid_key);
wl_status_t connect_wifi();
wl_status_t disconn_wifi();

#endif
