#include <Arduino.h>

#include "common_defs.h"
#include "debug_ctrl.h"
#include "json_msg_proc.h"
#include "json_keys.h"

#include "modbus_ops.h"

#define JSON_RX_BUF_SIZE 1024

/*
 * NOTE:
 * This function is NOT thread-safe.
 *
 * It uses internal static buffers and state variables to maintain
 * incremental receive and parse state across calls.
 *
 * Therefore:
 *   - It MUST be called from a single thread or execution context only.
 *   - Concurrent calls, re-entrant calls, or calls from multiple threads (or ISRs) are strictly FORBIDDEN.
 *
 * Typical and intended usage:
 *   - Called repeatedly from the main loop() in Arduino-style
 *     single-threaded environments.
 */
const char* json_msg_recv_proc(Stream &sport, json_str_msg_dispatcher_t hdlr, size_t * msg_len_ptr)
{
#define RESTART_RECV \
    {\
        data_start = data_end = 0; \
        depth = 0; \
        curr_proc_start = 0; \
        in_string = false; \
        escape = false; \
        in_msg = false; \
    }

    static char   rx_buf[JSON_RX_BUF_SIZE + 1];
    static char   msg_buf[MAX_JSON_MSG_LEN + 1];

    static size_t data_start = 0;  // 未处理数据起点
    static size_t data_end   = 0;  // 已写入数据尾
    static size_t curr_proc_start = 0;

    static int    depth      = 0;
    static bool   in_msg = false;
    static bool   in_string  = false;
    static bool   escape     = false;

    const char* last_msg_ptr = nullptr;
    size_t      last_msg_len = 0;

    if (msg_len_ptr) { *msg_len_ptr = 0; }

    /* ---------- 读取串口数据 ---------- */
    while (sport.available() > 0)
    {
        // 尾部没空间，尝试压缩
        if (data_end >= JSON_RX_BUF_SIZE)
        {
            if (data_start > 0)
            {
                size_t remain = data_end - data_start;
                memmove(rx_buf, rx_buf + data_start, remain);
                data_end   = remain;
                data_start = 0;
                curr_proc_start = data_end;
            }
            else
            {
                // 缓冲区满且无法压缩，直接丢弃
                RESTART_RECV;

                DBG_PRINTLN(LOG_WARN, F("json recv buffer full. discard buffer data."));
            }
        }

        char c = (char)sport.read();

        // 未进入 JSON，丢弃 '{' 之前的字符
        if (!in_msg && c != '{') { continue; }

        // 遇到 '{'，开始记录
        if (!in_msg && c == '{')
        {
            in_msg = true;

            data_start = data_end;
            rx_buf[data_end++] = c;
            continue;
        }

        rx_buf[data_end++] = c;
    }

    /* ---------- 扫描并解析 JSON ---------- */
    size_t i = curr_proc_start;
    while (i < data_end)
    {
        char c = rx_buf[i];
        if(!in_msg && c != '{')
        {
            i++; data_start++;
            continue;
        }

        if (in_string)
        {
            if (escape) { escape = false; } 
            else if (c == '\\') { escape = true; } 
            else if (c == '"') { in_string = false; }
        }
        else
        {
            if (c == '"') { in_string = true; } 
            else if (c == '{') { depth++; in_msg = true;} 
            else if (c == '}')
            {
                depth--;
                if (depth == 0)
                {
                    // 完整 JSON 结束
                    size_t msg_len = i - data_start + 1;
                    if (msg_len <= MAX_JSON_MSG_LEN)
                    {
                        memcpy(msg_buf, rx_buf + data_start, msg_len);
                        msg_buf[msg_len] = '\0';

                        if (hdlr) { hdlr(msg_buf); }

                        last_msg_ptr = msg_buf;
                        last_msg_len = msg_len;
                    }
                    // 移动 data_start 到下一个字符
                    data_start = i + 1;
                    in_msg = false;

                    in_string = escape = false;
                }
            }
        }
        i++;
    }
    curr_proc_start = i;

    /* ---------- 更新返回信息 ---------- */
    if (last_msg_ptr && msg_len_ptr)
    {
        *msg_len_ptr = last_msg_len;
    }

    return last_msg_ptr;

#undef RESTART_RECV
}

typedef struct
{
    const char* msg_type;
    json_doc_hdlr hdlr;
}json_msg_type_hdlr_map_t;

static void json_cmd_read_mb_reg(JsonDocument& /*doc*/)
{//4~12, 14, 15  
#define ITEM_ELE(e) e
#define ITEM_LIST \
    {\
        ITEM_ELE(State), ITEM_ELE(VoltSet), ITEM_ELE(FilamentSet), ITEM_ELE(ExposureTime), ITEM_ELE(Voltmeter),\
        ITEM_ELE(Ammeter), ITEM_ELE(RangeIndicationStatus), ITEM_ELE(ExposureStatus), ITEM_ELE(RangeIndicationStart), \
        ITEM_ELE(BatteryLevel), ITEM_ELE(BatteryVoltmeter), \
    }
    static hv_mb_reg_e_t reg_rpt_list[] = ITEM_LIST;
#undef ITEM_ELE
#define ITEM_ELE(e) String(e)
    static String reg_rpt_str_list[] = ITEM_LIST;

    uint16_t reg_val_buf[MAX_HV_NORMAL_MB_REG_NUM];

    memset(reg_val_buf, 0, sizeof(reg_val_buf));

    bool ret = hv_controller_read_regs((uint16_t)HSV, reg_val_buf, MAX_HV_NORMAL_MB_REG_NUM);

    LOG_LEVEL log_lvl = ret ? LOG_DEBUG : LOG_ERROR;
    DBG_PRINT(log_lvl, F("json_cmd_read_mb_reg read hv ret:")); DBG_PRINTLN(log_lvl, ret);

    JsonDocument ret_doc;
    ret_doc[JSON_KEY_JSON_TYPE] = JSON_VAL_TYPE_REG;
    for(int idx = 0; idx < ARRAY_ITEM_CNT(reg_rpt_list); ++idx)
    {
        ret_doc[reg_rpt_str_list[idx]] = String(reg_val_buf[reg_rpt_list[idx]]);
    }
    String mb_reg_val_json_doc_str;
    serializeJson(ret_doc, mb_reg_val_json_doc_str);

    g_scrn_serial.print(mb_reg_val_json_doc_str);
}

static json_msg_type_hdlr_map_t gs_cmd_handler_map[] =
{
    {JSON_VAL_COMMAND_READ_MB_REG, json_cmd_read_mb_reg},
    {JSON_VAL_COMMAND_INQUIRE_NETWORK, nullptr},
    {nullptr, nullptr},
};

static void find_hdlr_and_exec(JsonDocument& doc, const char* key_str, json_msg_type_hdlr_map_t * hdlr_map)
{
    if(!key_str || !hdlr_map) return;

    bool handled = false;
    const char* msg_or_cmd_type = doc[key_str];

    for(int idx = 0; hdlr_map[idx].msg_type && hdlr_map[idx].hdlr; ++idx)
    {
        if(!strcmp(msg_or_cmd_type, hdlr_map[idx].msg_type))
        {
            hdlr_map[idx].hdlr(doc);
            handled = true;
            break;
        }
    }

    if(!handled)
    {
        DBG_PRINT(LOG_WARN, F("Unknown msg or cmd type:")); DBG_PRINTLN(LOG_WARN, msg_or_cmd_type);
    }
    else
    {
        DBG_PRINT(LOG_DEBUG, F("Msg handled: ")); DBG_PRINTLN(LOG_DEBUG, msg_or_cmd_type);
    }
}

static void cmd_hdlr(JsonDocument& doc)
{
    find_hdlr_and_exec(doc, JSON_KEY_COMMAND, gs_cmd_handler_map);
}

static void json_cmd_write_mb_reg(JsonDocument& doc)
{
#define DBG_WRITE_LOG \
    {\
        String dbg_str = String("json_cmd_write_mb_reg write hv, reg from ") + String(reg_seg_1st) \
                        + String(" to ") + String(reg_seg_last) + String(". ret: ") + String(ret); \
        LOG_LEVEL log_lvl = ret ? LOG_DEBUG : LOG_ERROR; \
        DBG_PRINTLN(log_lvl, dbg_str); \
    }

    bool new_reg_seg = true;
    uint16_t reg_seg_1st = 0, reg_seg_last = 0;
    uint16_t reg_val_buf[MAX_HV_NORMAL_MB_REG_NUM];
    int idx = 0;

    JsonObject obj = doc.as<JsonObject>();
    for (JsonPair kv : obj)
    {
        const char* key = kv.key().c_str();
        JsonVariant value = kv.value();
        if(!strcmp(key, JSON_KEY_JSON_TYPE)) continue;

        uint16_t reg_no = (uint16_t)(String(key).toInt());
        if((!new_reg_seg && (reg_seg_last + 1 != reg_no)) || (idx >= MAX_HV_NORMAL_MB_REG_NUM))
        {
            bool ret = hv_controller_write_mult_regs(reg_seg_1st, reg_val_buf, idx);
            new_reg_seg = true;

            DBG_WRITE_LOG;
        }

        if(new_reg_seg)
        {
            idx = 0;
            reg_seg_1st = reg_seg_last = reg_no;
            reg_val_buf[idx++] = (uint16_t)(value.as<unsigned int>());
            new_reg_seg = false;

        }
        else if(reg_seg_last + 1 == reg_no)
        {
            reg_seg_last = reg_no;
            reg_val_buf[idx++] = (uint16_t)(value.as<unsigned int>());
        }
    }

    if(idx > 0)
    {
        bool ret = hv_controller_write_mult_regs(reg_seg_1st, reg_val_buf, idx);
        DBG_WRITE_LOG;
    }
#undef DBG_WRITE_LOG
}

void scan_wifi_aps(JsonDocument& scan_json_doc);
void connect_wifi(JsonDocument& ssid_key);
void disconn_wifi(JsonDocument& msg);
static json_msg_type_hdlr_map_t gs_json_msg_handler_map[] =
{
    {JSON_VAL_TYPE_SCAN_WIFI, scan_wifi_aps},
    {JSON_VAL_TYPE_CONN_AP, connect_wifi},
    {JSON_VAL_TYPE_DISCONN_AP, disconn_wifi},
    {JSON_VAL_TYPE_CMD, cmd_hdlr},
    {JSON_VAL_TYPE_REG, json_cmd_write_mb_reg},
    {nullptr, nullptr},
};

void json_msg_handler_dispatcher(const char * msg)
{
    if(!msg)
    {
        DBG_PRINTLN(LOG_ERROR, F("json msg string is NULL."));
        return;
    }

    DBG_PRINTLN(LOG_DEBUG, F("a complete json msg:"));
    DBG_PRINTLN(LOG_DEBUG, msg);

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, msg);
    if(error)
    {
        DBG_PRINT(LOG_ERROR, F("deserialize json error:"));
        DBG_PRINTLN(LOG_ERROR, error.c_str());
        return;
    }

    find_hdlr_and_exec(doc, JSON_KEY_JSON_TYPE, gs_json_msg_handler_map);
}

