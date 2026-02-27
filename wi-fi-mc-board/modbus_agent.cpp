#include <WiFiServer.h>
#include <WiFiClient.h>

#include "common_defs.h"
#include "modbus_internal.h"
#include "modbus_ops.h"

#include "debug_ctrl.h"

#define MODBUS_TCP_PORT 502

#define RTU_BAUDRATE   9600

static WiFiServer mbServer(MODBUS_TCP_PORT, TCP_MODE, NON_BLOCKING_MODE);
#define MAX_TCP_CLIENTS 2
static WiFiClient tcpClients[MAX_TCP_CLIENTS];

typedef struct
{
  uint16_t transId;
  uint16_t protoId;
  uint16_t length;
  uint8_t  unitId;
}MBAP;

static const int gs_mb_max_pdu_len = 253;

typedef struct
{
  bool     busy;          // 该 client 是否在等待 RTU
  MBAP     mbap;
  uint8_t  pdu[gs_mb_max_pdu_len];
  uint8_t  pduLen;
}ClientCtx;
static ClientCtx ctx[MAX_TCP_CLIENTS];

static const int gs_mb_tcp_mbap_len = 7; //size of MBAP.
static const int gs_mb_max_tcp_adu_len = gs_mb_tcp_mbap_len + gs_mb_max_pdu_len;

static const int gs_mb_rtu_addr_len = 1;
static const int gs_mb_rtu_crc_len = 2;
static const int gs_mb_max_rtu_adu_len = gs_mb_rtu_addr_len + gs_mb_max_pdu_len + gs_mb_rtu_crc_len;

static const int gs_mb_excption_resp_pdu_len = 2; //1 byte err, 1 byte execption

//exception resp adu: 1 byte addr, 1 byte err, 1 byte execption, 2 bytes crc
static const int gs_mb_min_rtu_resp_adu_len = 5;

static const int gs_mb_func_code_len = 1;

static uint16_t modbus_crc16(const uint8_t* buf, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++)
  {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

typedef enum
{
    GW_IDLE,
    GW_WAIT_RTU
}GwState ;

static GwState gwState = GW_IDLE;


/*------------------------------*/
static uint8_t gs_mb_rtu_adu_buf[gs_mb_max_rtu_adu_len];
static unsigned long gs_rtu_send_time = 0;
static uint16_t gs_rtu_rx_len = 0;
static bool poll_rtu_response(bool *crc_ret = nullptr, uint16_t * pdu_len = nullptr)
{
    int incoming_byte_cnt = g_pdb_serial.available();
    if(incoming_byte_cnt <= 0) return false;
    DBG_PRINT(LOG_DEBUG, F("rtu available bytes ")); DBG_PRINTLN(LOG_DEBUG, incoming_byte_cnt);

    if(incoming_byte_cnt > gs_mb_max_rtu_adu_len)
    {
        DBG_PRINTLN(LOG_ERROR, F("too many incoming bytes, buffer can't contain!!!"));
        return false;
    }

    if(gs_rtu_rx_len + incoming_byte_cnt > gs_mb_max_rtu_adu_len)
    {
        gs_rtu_rx_len = 0;//buffer full. currently just discard received data. maybe do better in future.

    }
    size_t read_byte_cnt = g_pdb_serial.readBytes(&gs_mb_rtu_adu_buf[gs_rtu_rx_len], incoming_byte_cnt);
    DBG_PRINT(LOG_DEBUG, F("rtu read bytes ")); DBG_PRINTLN(LOG_DEBUG, read_byte_cnt);
    gs_rtu_rx_len += read_byte_cnt;

    bool complete_resp = false, crc_ok = false;
    if (gs_rtu_rx_len >= gs_mb_min_rtu_resp_adu_len)
    {
        uint16_t expect_len = 0;
        mb_func_code_e_t resp_func_code = (mb_func_code_e_t)gs_mb_rtu_adu_buf[1]; //the 1st byte is addr byte.
        if(resp_func_code >= MB_EXCP_FUNC_CODE_START) //exception
        {
            expect_len = 1/*addr*/ + 1/*exception-func_code*/ + 1/*exception code*/ + 2/*crc*/;
            complete_resp = true;
        }
        else
        {
            uint8_t byte_count;
            switch(resp_func_code)
            {
                case MB_FUNC_CODE_READ_HREG:
                    byte_count = gs_mb_rtu_adu_buf[2];
                    expect_len = 1/*addr*/ + 1/*func code*/ + 1/*byte count*/ + byte_count + 2/*crc*/;
                    break;

                case MB_FUNC_CODE_WRITE_SINGLE_REG:
                    expect_len = 1/*addr*/ + 1/*func code*/ + 2/*reg addr*/ + 2/*reg val*/ + 2/*crc*/;
                    break;

                default: //MB_FUNC_CODE_WRITE_MULI_REGS
                    expect_len = 1/*addr*/ + 1/*func code*/ + 2/*start reg addr*/ + 2/*reg count*/ + 2/*crc*/;
                    break;
            }
            complete_resp = (gs_rtu_rx_len >= expect_len);
        }

        if(complete_resp)
        {
            DBG_PRINTLN(LOG_DEBUG, F("one complete rtu response"));
            {
                for(int i = 0; i < gs_rtu_rx_len; ++i)
                {
                    DBG_PRINT(LOG_DEBUG, gs_mb_rtu_adu_buf[i], HEX);
                    DBG_PRINT(LOG_DEBUG, F(" "));
                }
                DBG_PRINTLN(LOG_DEBUG, F("\n"));
            }


            uint16_t crcCalc = modbus_crc16(gs_mb_rtu_adu_buf, expect_len - 2);
            uint16_t crcRecv = gs_mb_rtu_adu_buf[expect_len - 2] |
                               (gs_mb_rtu_adu_buf[expect_len - 1] << 8);

            crc_ok = (crcCalc == crcRecv); 
            gs_rtu_rx_len = 0;

            if(pdu_len) *pdu_len = expect_len - 3; //omit the 1st 1 addr byte and the last 2 crc bytes.


            DBG_PRINT(LOG_DEBUG, F("crc check: ")); DBG_PRINTLN(LOG_DEBUG, crc_ok);
        }
    }

    if(crc_ret) *crc_ret = crc_ok;
    return complete_resp;
}

bool send_mb_rtu_request(uint8_t * rtu_pdu, uint16_t pdu_len, uint8_t addr)
{
    uint16_t len = 0;

    if(!rtu_pdu || pdu_len > gs_mb_max_pdu_len)
    {
        return false;
    }
    gs_mb_rtu_adu_buf[len++] = addr;
    memcpy(&gs_mb_rtu_adu_buf[len], rtu_pdu, pdu_len);
    len += pdu_len;

    uint16_t crc = modbus_crc16(gs_mb_rtu_adu_buf, len);
    gs_mb_rtu_adu_buf[len++] = crc & 0xFF;
    gs_mb_rtu_adu_buf[len++] = crc >> 8;

    g_pdb_serial.write(gs_mb_rtu_adu_buf, len);

    gs_rtu_send_time = millis();
    return true;
}

bool read_rtu_response(uint8_t ** rtu_pdu, uint16_t *pdu_len_ptr, uint32_t timeout_ms)
{
    bool complet_msg = false, crc_ok = false, timeout = false;
    uint16_t pdu_len = 0;
    while(!(complet_msg = poll_rtu_response(&crc_ok, &pdu_len)))
    {
        uint32_t curr = millis();
        if(curr - gs_rtu_send_time >= timeout_ms)
        {
            timeout = true;
            DBG_PRINTLN(LOG_ERROR, F("read rtu response timeout!"));

            gs_rtu_rx_len = 0;
            break;
        }
    }
    if(rtu_pdu) *rtu_pdu = &gs_mb_rtu_adu_buf[1];
    if(pdu_len_ptr) *pdu_len_ptr = pdu_len;
    return (complet_msg && crc_ok && !timeout);
}

/*------------------------------*/
static int activeClient = -1;
static uint8_t gs_mb_tcp_adu_buf[gs_mb_max_tcp_adu_len];
void send_tcp_response(int idx, uint8_t * pdu_buf, uint16_t pdu_len)
{
    if(!pdu_buf || pdu_len + gs_mb_tcp_mbap_len > gs_mb_max_tcp_adu_len) return;

    WiFiClient& c = tcpClients[idx];
    ClientCtx&  x = ctx[idx];

    MBAP *mbap_part= (MBAP*)gs_mb_tcp_adu_buf;
    mbap_part->transId = UINT16_HTONS(x.mbap.transId);
    mbap_part->protoId = UINT16_HTONS(x.mbap.protoId);
    mbap_part->length  = UINT16_HTONS(pdu_len + 1);
    mbap_part->unitId = x.mbap.unitId;
    memcpy(&gs_mb_tcp_adu_buf[gs_mb_tcp_mbap_len], pdu_buf, pdu_len); 

    c.write(gs_mb_tcp_adu_buf, gs_mb_tcp_mbap_len + pdu_len);
}

void send_tcp_exception(int idx, uint8_t fc, uint8_t code)
{
    WiFiClient& c = tcpClients[idx];
    ClientCtx&  x = ctx[idx];

    MBAP *mbap_part= (MBAP*)gs_mb_tcp_adu_buf;
    mbap_part->transId = UINT16_HTONS(x.mbap.transId);
    mbap_part->protoId = UINT16_HTONS(x.mbap.protoId);
    mbap_part->length  = UINT16_HTONS(3);
    mbap_part->unitId = x.mbap.unitId;

    uint8_t *pdu = &gs_mb_tcp_adu_buf[gs_mb_tcp_mbap_len];
    pdu[0] = fc | 0x80;
    pdu[1] = code;

    c.write(gs_mb_tcp_adu_buf, gs_mb_tcp_mbap_len + gs_mb_excption_resp_pdu_len);
}

static void handle_tcp_response(int idx)
{
    uint8_t * pdu_buf;
    uint16_t pdu_len;
    bool ret = read_rtu_response(&pdu_buf, &pdu_len);

    if(ret)
    {
        send_tcp_response(idx, pdu_buf, pdu_len);
    }
    else
    {
        send_tcp_exception(idx, ctx[activeClient].pdu[0], MB_GW_TGT_DEV_FAILED_TO_RESP);
    }

    ctx[idx].busy = false;
    activeClient = -1;
    gwState = GW_IDLE;
}

bool read_tcp_req_adu(int idx, uint32_t timeout_ms)
{
    WiFiClient& c = tcpClients[idx];
    ClientCtx&  x = ctx[idx];

    bool ret = false;

    if(c.available() <= 0) return ret;

    int incoming_bytes = 0, expect_byte_cnt = gs_mb_tcp_mbap_len + gs_mb_func_code_len;
    bool mbap_read = false;
    unsigned long start_time = millis();
    while((incoming_bytes < expect_byte_cnt) && (millis() - start_time <= timeout_ms))
    {
        int read_ret = c.read(&gs_mb_tcp_adu_buf[incoming_bytes], gs_mb_max_tcp_adu_len - incoming_bytes);
        DBG_PRINT(LOG_DEBUG, F("mb tcp read return: ")); DBG_PRINTLN(LOG_DEBUG, read_ret);
        if(read_ret <= 0) continue;

        incoming_bytes += read_ret;

        if(incoming_bytes < expect_byte_cnt) continue;

        if(!mbap_read)
        {
            MBAP * mbap_part = (MBAP*)gs_mb_tcp_adu_buf;
            x.mbap.transId = UINT16_NSTOH(mbap_part->transId);
            x.mbap.protoId = UINT16_NSTOH(mbap_part->protoId);
            x.mbap.length  = UINT16_NSTOH(mbap_part->length);
            x.mbap.unitId = mbap_part->unitId;

            x.pduLen = x.mbap.length - 1;
            if (x.pduLen > sizeof(x.pdu)) return ret;

            expect_byte_cnt = gs_mb_tcp_mbap_len + x.pduLen;

            mbap_read = true;

            if(incoming_bytes >= expect_byte_cnt) ret = true;
        }
        else
        {
            ret = true;
            break;
        }
    }
    if(ret) memcpy(x.pdu, &gs_mb_tcp_adu_buf[gs_mb_tcp_mbap_len], x.pduLen);

    return ret;
}

static void try_handle_tcp_request(int idx)
{
    if(!read_tcp_req_adu(idx))
    {
        return;
    }

    ClientCtx&  x = ctx[idx];

    uint8_t fc = x.pdu[0];
    DBG_PRINT(LOG_DEBUG, F("mb tcp req func code is: ")); DBG_PRINTLN(LOG_DEBUG, fc);
    if(!IS_VALID_MB_FUNC_CODE(fc))
    {
        send_tcp_exception(idx, fc, MB_EXCP_ILLEGAL_FUNC);
        return;
    }

    DBG_PRINTLN(LOG_DEBUG, F("send mb rtu request"));
    send_mb_rtu_request(x.pdu, x.pduLen, x.mbap.unitId);

    x.busy = true;
    activeClient = idx;
    gwState = GW_WAIT_RTU;
}

static void cleanup_tcp_client(int idx)
{
    if (tcpClients[idx])
    {
        DBG_PRINT(LOG_DEBUG, F("client stop: ")); DBG_PRINTLN(LOG_DEBUG, idx);

        tcpClients[idx].stop();
    }
    ctx[idx].busy = false;

    if (activeClient == idx)
    {
        activeClient = -1;
        gwState = GW_IDLE;
    }
}

static void accept_new_clients()
{
    WiFiClient newClient = mbServer.available();
    if (!newClient) return;

    for (int i = 0; i < MAX_TCP_CLIENTS; i++)
    {
        if (!tcpClients[i] || !tcpClients[i].connected())
        {
            tcpClients[i] = newClient;
            ctx[i].busy = false;

            if(tcpClients[i].connected())
            {
                DBG_PRINT(LOG_DEBUG, F("accept, "));
                DBG_PRINT(LOG_DEBUG, i); DBG_PRINT(LOG_DEBUG, F(" conn state: "));
                DBG_PRINTLN(LOG_DEBUG, tcpClients[i].connected());
            }
            return;
        }
    }

    // 超过 MAX_TCP_CLIENTS 个，直接拒绝
    newClient.stop();
}

static bool gs_mb_server_started = false;
void end_mb_tcp_server()
{
    DBG_PRINTLN(LOG_INFO, "end server...");
    for (int i = 0; i < MAX_TCP_CLIENTS; i++)
    {
        cleanup_tcp_client(i);
    }

    activeClient = -1;
    gwState = GW_IDLE;

    mbServer.end();
    gs_mb_server_started = false;
}

void modbus_tcp_server(bool work)
{
    if(!gs_mb_server_started && work)
    {
        mbServer.begin();
        gs_mb_server_started = true;
    }
    else if(!work)
    {
        if(gs_mb_server_started)
        {
            end_mb_tcp_server();
        }
        return;
    }

    accept_new_clients();

    // 轮询每个 TCP client
    for (int i = 0; i < MAX_TCP_CLIENTS; i++)
    {
        if (!tcpClients[i] || !tcpClients[i].connected())
        {
            cleanup_tcp_client(i);
            continue;
        }

        if (!ctx[i].busy && gwState == GW_IDLE)
        {
            try_handle_tcp_request(i);
            if (gwState == GW_WAIT_RTU)
            {
                handle_tcp_response(i);
            }
        }
    }
}

/*------------------------------*/
bool hv_controller_write_single_reg(uint16_t reg_addr, uint16_t value)
{
    uint8_t pdu[5]; //1 byte func_code + 2 byte reg_addr + 2 byte value
    int pos = 0;
    bool ret;

    pdu[pos++] = MB_FUNC_CODE_WRITE_SINGLE_REG;
    pdu[pos++] = UINT16_HI_BYTE(reg_addr);
    pdu[pos++] = UINT16_LO_BYTE(reg_addr); //addr
    pdu[pos++] = UINT16_HI_BYTE(value);
    pdu[pos++] = UINT16_LO_BYTE(value); //value

    ret = send_mb_rtu_request(pdu, sizeof(pdu));
    if(ret) ret = read_rtu_response();
    return ret;
}

bool hv_controller_write_mult_regs(uint16_t reg_addr_start, uint16_t *buf, int reg_cnt)
{
    uint8_t pdu[gs_mb_max_pdu_len];
    int pdu_len = 0;
    bool ret = false;

     //6: 1 byte of func code; 2 bytes of start addr, 2 bytes of reg cnt, 1 byte of byte count
    if(!buf || (reg_cnt * 2 + 6 > gs_mb_max_pdu_len)) return false;

    pdu[pdu_len++] = MB_FUNC_CODE_WRITE_MULI_REGS;
    pdu[pdu_len++] = UINT16_HI_BYTE(reg_addr_start);
    pdu[pdu_len++] = UINT16_LO_BYTE(reg_addr_start); //start addr
    pdu[pdu_len++] = UINT16_HI_BYTE(reg_cnt);
    pdu[pdu_len++] = UINT16_LO_BYTE(reg_cnt); //reg_cnt
    pdu[pdu_len++] = 2 * reg_cnt;
    for(int idx = 0; idx < reg_cnt; ++idx)
    {
        pdu[pdu_len++] = UINT16_HI_BYTE(buf[idx]);
        pdu[pdu_len++] = UINT16_LO_BYTE(buf[idx]);
    }

    ret = send_mb_rtu_request(pdu, pdu_len);
    if(ret) ret = read_rtu_response();
    return ret;
}

bool hv_controller_read_regs(uint16_t reg_addr_start, uint16_t * buf, int reg_cnt)
{
    uint8_t req_pdu[5]; //1 byte func_code + 2 byte start addr + 2 byte reg cnt
    int pos = 0;
    bool ret;

    req_pdu[pos++] = MB_FUNC_CODE_READ_HREG;
    req_pdu[pos++] = UINT16_HI_BYTE(reg_addr_start);
    req_pdu[pos++] = UINT16_LO_BYTE(reg_addr_start); //start addr
    req_pdu[pos++] = UINT16_HI_BYTE(reg_cnt);
    req_pdu[pos++] = UINT16_LO_BYTE(reg_cnt); //reg_cnt

    ret = send_mb_rtu_request(req_pdu, sizeof(req_pdu));
    if(ret)
    {
        uint8_t * resp_pdu_ptr;
        uint16_t resp_pdu_len;
        ret = read_rtu_response(&resp_pdu_ptr, &resp_pdu_len);
        if(ret && buf)
        {
            /*resp pdu ptr: 1 byte of func code, 1 byte of byte count, then followed by data bytes.*/
            int byte_cnt_in_pdu = (int)(resp_pdu_ptr[1]);
            if(byte_cnt_in_pdu != reg_cnt * 2) return false;

            uint8_t *val_byte_ptr = &resp_pdu_ptr[2];
            for(int idx = 0; idx < reg_cnt; ++idx)
            {
                buf[idx] = (uint16_t)((val_byte_ptr[2*idx] << 8) | (val_byte_ptr[2*idx + 1]));
            }
        }
    }
    return ret;
}

const char* get_pdb_ver_str()
{
#define MAX_PDB_VER_STR_LEN 6
    static char ver_str[MAX_PDB_VER_STR_LEN + 1] = {0};
    static bool getted = false;

#define BYTE_2_3DGIT_STR \
    {\
        for(int j = 0; j < 3; ++j)\
        {\
            ver_str[idx--] = (pv % 10) + '0';\
            pv = pv / 10;\
        }\
    }
    if(!getted)
    {
        uint16_t ver = 0;
        if(hv_controller_read_regs(HSV, &ver, 1))
        {
            int idx = MAX_PDB_VER_STR_LEN - 1;
            uint8_t pv = ver & 0xFF;
            BYTE_2_3DGIT_STR;

            pv = ver >> 8;
            BYTE_2_3DGIT_STR;
 
            ver_str[MAX_PDB_VER_STR_LEN] = 0;
            getted = true;
        }
    }

    return getted ? ver_str : nullptr;

#undef BYTE_2_3DGIT_STR
#undef MAX_PDB_VER_STR_LEN
}
