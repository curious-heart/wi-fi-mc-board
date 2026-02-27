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

void scan_wifi_aps(JsonDocument& scan_json_doc);
const char* get_mac_addr_str();

wl_status_t connect_wifi(const char* ssid, const char* pwd, bool cpy = true);
void connect_wifi(JsonDocument& ssid_key);
wl_status_t connect_wifi();
void disconn_wifi(JsonDocument& doc);
wl_status_t disconn_wifi(bool clear_ap = false);

void rpt_network_info_json();

void wifi_config(const char* ip_str, const char* gw_str, const char* mask_str);

#endif
