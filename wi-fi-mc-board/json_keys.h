#ifndef __JSON_KEYS_H__
#define __JSON_KEYS_H__

#define JSON_KEY(var_name, val) extern const char* var_name;

#define JSON_KEY_LIST \
    JSON_KEY(JSON_KEY_JSON_TYPE, "json_type") \
    JSON_KEY(JSON_VAL_TYPE_AP_LIST, "wifi_ap_list") \
    JSON_KEY(JSON_VAL_TYPE_SCAN_WIFI, "scan_wifi") \
    JSON_KEY(JSON_VAL_TYPE_CONN_AP, "conn_wifi_ap") \
    JSON_KEY(JSON_VAL_TYPE_DISCONN_AP, "disconn_wifi") \
    JSON_KEY(JSON_VAL_TYPE_REG, "register") \
    JSON_KEY(JSON_VAL_TYPE_CMD, "cmd") \
    JSON_KEY(JSON_VAL_TYPE_DATA, "data") \
    JSON_KEY(JSON_KEY_AP_LIST, "ap_list") \
    JSON_KEY(JSON_KEY_SSID, "ssid") \
    JSON_KEY(JSON_KEY_SEC, "security") \
    JSON_KEY(JSON_KEY_RSSI, "rssi") \
    JSON_KEY(JSON_KEY_WIFI_PWD, "wifi_pwd") \
    JSON_KEY(JSON_KEY_DIST, "tof_distance") \
    JSON_KEY(JSON_KEY_COMMAND, "command") \
    JSON_KEY(JSON_VAL_COMMAND_READ_MB_REG, "read_mb_reg") \
    JSON_KEY(JSON_VAL_COMMAND_INQUIRE_NETWORK, "inquire_network")

JSON_KEY_LIST;

#endif
