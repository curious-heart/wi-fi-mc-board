#include <Arduino.h>
#include <ArduinoJson.h>

#include <stdio.h>

#include "common_defs.h"
#include "json_keys.h"
#include "wifi_ops.h"
#include "debug_ctrl.h"


static wl_status_t gs_wifi_status = WL_IDLE_STATUS;
static wifi_ssid_pwd_s_t gs_curr_wifi_ssid_pwd;

void wifi_init()
{
    gs_curr_wifi_ssid_pwd.ssid[g_max_ssid_len] = 0;
    gs_curr_wifi_ssid_pwd.pwd[g_max_wifi_charpwd_len] = 0;

    gs_wifi_status = (wl_status_t)WiFi.status();
}

wl_status_t curr_wifi_status()
{
    return (gs_wifi_status = (wl_status_t)WiFi.status());
}

wl_status_t connect_wifi(const char* ssid, const char* pwd, bool cpy)
{
    gs_wifi_status = (wl_status_t)WiFi.status();
    if(WL_CONNECTED == gs_wifi_status) disconn_wifi();

    if(cpy)
    {
        strncpy(gs_curr_wifi_ssid_pwd.ssid, ssid, g_max_ssid_len);
        strncpy(gs_curr_wifi_ssid_pwd.pwd, pwd, g_max_wifi_charpwd_len);
    }

    if(strlen(gs_curr_wifi_ssid_pwd.pwd) == 0)
    {
        gs_wifi_status = (wl_status_t)WiFi.begin(gs_curr_wifi_ssid_pwd.ssid);
    }
    else
    {
        gs_wifi_status = (wl_status_t)WiFi.begin(gs_curr_wifi_ssid_pwd.ssid, gs_curr_wifi_ssid_pwd.pwd);
    }
    return gs_wifi_status;
}

void connect_wifi(JsonDocument& ssid_key)
{
    gs_wifi_status = connect_wifi(ssid_key[JSON_KEY_SSID], ssid_key[JSON_KEY_WIFI_PWD], true);
}

wl_status_t connect_wifi()
{
    if(strlen(gs_curr_wifi_ssid_pwd.ssid) > 0)
    {
        gs_wifi_status = connect_wifi(gs_curr_wifi_ssid_pwd.ssid, gs_curr_wifi_ssid_pwd.pwd, false);
    }
    return gs_wifi_status;
}

void disconn_wifi(JsonDocument& /*doc*/)
{
    disconn_wifi();
}

wl_status_t disconn_wifi()
{
    void end_mb_tcp_server();

    end_mb_tcp_server();

    gs_wifi_status = (wl_status_t)WiFi.disconnect();

    memset(&gs_curr_wifi_ssid_pwd, 0, sizeof(gs_curr_wifi_ssid_pwd));

    return gs_wifi_status;
}

#define WIFI_SECURITY_LIST(X) \
    X(SECURITY_OPEN,         "Open") \
    X(SECURITY_WEP_PSK,      "WEP") \
    X(SECURITY_WPA_TKIP_PSK, "WPA TKIP") \
    X(SECURITY_WPA_AES_PSK,  "WPA AES") \
    X(SECURITY_WPA2_AES_PSK, "WPA2 AES") \
    X(SECURITY_WPA2_TKIP_PSK, "WPA2 TKIP") \
    X(SECURITY_WPA2_MIXED_PSK, "WPA2 MIXED") \
    X(SECURITY_WPA_WPA2_MIXED|AES_ENABLED, "WPA/WPA2 AES") \
    X(SECURITY_WPA3_AES_PSK,  "WPA3 AES") \
    X(SECURITY_WPA2_WPA3_MIXED, "WPA2/WPA3")

const char* wifi_security_2_str(uint32_t t)
{
    switch (t) {
#define X(a,b) case a: return b;
        WIFI_SECURITY_LIST(X)
#undef X
        default: return "Unknown";
    }
}

#define WIFI_ENC_LIST(X) \
    X(ENC_TYPE_WEP, "WEP") \
    X(ENC_TYPE_WPA, "WPA") \
    X(ENC_TYPE_WPA3, "WPA3") \
    X(ENC_TYPE_WPA2, "WPA2") \
    X(ENC_TYPE_NONE, "NONE") \
    X(ENC_TYPE_AUTO, "AUTO")

const char* wifi_enc_2_str(uint8_t t)
{
    switch (t) {
#define X(a,b) case a: return b;
        WIFI_ENC_LIST(X)
#undef X
        default: return "Unknown";
    }
}

static void build_ap_list_json(String &json_msg_string)
{
    wdt.RefreshWatchdog();

    DBG_PRINTLN(LOG_DEBUG, F("** Scan Networks **"));
    int numSsid = WiFi.scanNetworks();
    if (numSsid < 0) {
        DBG_PRINTLN(LOG_ERROR, F("Couldn't get a wifi connection"));
        return;
    }

    // print the list of networks seen:
    DBG_PRINT(LOG_DEBUG, "number of available networks:"); DBG_PRINTLN(LOG_DEBUG, numSsid);

    JsonDocument doc;
    doc[JSON_KEY_JSON_TYPE] = JSON_VAL_TYPE_AP_LIST;
    JsonArray ap_list_json = doc[JSON_KEY_AP_LIST].to<JsonArray>();

    for (int thisNet = 0; thisNet < numSsid; thisNet++)
    {
        JsonObject ap_info_json = ap_list_json.add<JsonObject>();

        ap_info_json[JSON_KEY_SSID]     = WiFi.SSID(thisNet);

        uint32_t sec;
        sec = WiFi.encryptionTypeEx(thisNet);
        ap_info_json[JSON_KEY_SEC] = wifi_security_2_str(sec);

        ap_info_json[JSON_KEY_RSSI]     = WiFi.RSSI(thisNet);

    }

    serializeJson(doc, json_msg_string);

    wdt.RefreshWatchdog();
}

void scan_wifi_aps(JsonDocument& /*scan_json_doc*/)
{
    String scan_result;
    build_ap_list_json(scan_result);

    g_scrn_serial.print(scan_result);
}

void printMacAddress()
{
    // print your MAC address:
    uint8_t mac[6];
    static char mac_str[6*3] = {0};
    static bool getted = false;

    if(!getted)
    {
        char d;
        WiFi.macAddress(mac);
        for(int i = 0; i < 6; ++i)
        {
            d = (mac[i] >> 4);  mac_str[i * 3] = HEXD_2_CH_CAP(d);
            d = (mac[i] & 0xF); mac_str[i * 3 + 1] = HEXD_2_CH_CAP(d);
            mac_str[i * 3 + 2] = ':';
        }
        mac_str[5*3+2] = 0;
        getted = true;
    }

    g_dbg_serial.print("MAC: ");
    g_dbg_serial.println(mac_str);
}

void rpt_network_info_json()
{
    JsonDocument doc;
    doc[JSON_KEY_JSON_TYPE] = JSON_VAL_TYPE_NETWORK;

    doc[JSON_KEY_CONN_STATUS] = (WL_CONNECTED == curr_wifi_status()) ? 1 : 0;
    doc[JSON_KEY_IP_ADDR] = WiFi.localIP().get_address();
    doc[JSON_KEY_SUBNET_MASK] = WiFi.subnetMask().get_address();
    doc[JSON_KEY_GW] = WiFi.gatewayIP().get_address();
    doc[JSON_KEY_SSID] = WiFi.SSID();
    doc[JSON_KEY_RSSI] = WiFi.RSSI();

    String msg_str;
    serializeJson(doc, msg_str);
    g_scrn_serial.print(msg_str);
}

bool parseIP(const char* str, IPAddress& out)
{
    int a, b, c, d;

    int scan_ret = sscanf(str, "%d.%d.%d.%d", &a, &b, &c, &d);
    if (scan_ret != 4) return false;

    if (0>a || a>255 || 0>b || b>255 || 0>c || c>255 || 0>d || d>255)
        return false;

    out = IPAddress((uint8_t)a, (uint8_t)b, (uint8_t)c, (uint8_t)d);
    return true;
}

void json_config_network(JsonDocument& doc)
{
    const char *ip_str = doc[JSON_KEY_IP_ADDR].as<const char*>(),
               *gw_str = doc[JSON_KEY_GW].as<const char*>(),
               *mask_str = doc[JSON_KEY_SUBNET_MASK].as<const char*>();

    DBG_PRINTLN(LOG_DEBUG, F("ips in msg:"));
    DBG_PRINT(LOG_DEBUG, F("local ip: "));    DBG_PRINTLN(LOG_DEBUG, ip_str);
    DBG_PRINT(LOG_DEBUG, F("gw: "));          DBG_PRINTLN(LOG_DEBUG, gw_str);
    DBG_PRINT(LOG_DEBUG, F("subnet mask: ")); DBG_PRINTLN(LOG_DEBUG, mask_str);

    IPAddress local_ip, dns_server, gw_ip, subnet_mask;

    if(!parseIP(ip_str, local_ip))
    {
        DBG_PRINT(LOG_ERROR, F("ip parse error: ")); DBG_PRINTLN(LOG_ERROR, ip_str);
        return;
    }

    if(!parseIP(gw_str, gw_ip))
    {
        DBG_PRINT(LOG_ERROR, F("gw parse error: ")); DBG_PRINTLN(LOG_ERROR, gw_str);
        return;
    }

    if(!parseIP(mask_str, subnet_mask))
    {
        DBG_PRINT(LOG_ERROR, F("subnet_mask parse error: ")); DBG_PRINTLN(LOG_ERROR, mask_str);
        return;
    }

    DBG_PRINTLN(LOG_DEBUG, F("\n"));
    DBG_PRINTLN(LOG_DEBUG, F("ips after convert:"));
    DBG_PRINT(LOG_DEBUG, F("local ip: "));    DBG_PRINTLN(LOG_DEBUG, local_ip);
    DBG_PRINT(LOG_DEBUG, F("dns server: "));  DBG_PRINTLN(LOG_DEBUG, dns_server);
    DBG_PRINT(LOG_DEBUG, F("gw: "));          DBG_PRINTLN(LOG_DEBUG, gw_ip);
    DBG_PRINT(LOG_DEBUG, F("subnet mask: ")); DBG_PRINTLN(LOG_DEBUG, subnet_mask);

    WiFi.config(local_ip, dns_server, gw_ip, subnet_mask);
}
