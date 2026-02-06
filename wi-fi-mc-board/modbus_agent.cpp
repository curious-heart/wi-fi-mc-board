#include <WiFiServer.h>
#include <WiFiClient.h>

#include "common_defs.h"
#include "modbus_internal.h"

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

static const int gs_mb_excption_resp_pdu_len = 2;
static const int gs_mb_min_rtu_resp_adu_len = 5; //exception resp adu

static const int gs_mb_func_code_len = 1;

uint16_t modbus_crc16(const uint8_t* buf, uint16_t len)
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
static unsigned long rtuSendTime = 0;

static int activeClient = -1;

static uint8_t rtuRxBuf[gs_mb_max_rtu_adu_len];
static void send_tcp_response(int idx, uint16_t rtu_adu_len)
{
    WiFiClient& c = tcpClients[idx];
    ClientCtx&  x = ctx[idx];

    uint8_t pduLen = rtu_adu_len - 3; /*1 byte addr plus 2 bytes crc.*/

    MBAP mbap;
    mbap.transId = UINT16_HTONS(x.mbap.transId);
    mbap.protoId = 0;
    mbap.length  = UINT16_HTONS(pduLen + 1);
    mbap.unitId  = rtuRxBuf[0];

    c.write((uint8_t*)&mbap, gs_mb_tcp_mbap_len);
    c.write(&rtuRxBuf[1], pduLen);
}

static void send_tcp_exception(int idx, uint8_t fc, uint8_t code)
{
    WiFiClient& c = tcpClients[idx];
    ClientCtx&  x = ctx[idx];

    MBAP mbap;
    mbap.transId = UINT16_HTONS(x.mbap.transId);
    mbap.protoId = 0;
    mbap.length  = UINT16_HTONS(3);
    mbap.unitId  = x.mbap.unitId;

    uint8_t pdu[gs_mb_excption_resp_pdu_len];
    pdu[0] = fc | 0x80;
    pdu[1] = code;

    c.write((uint8_t*)&mbap, gs_mb_tcp_mbap_len);
    c.write(pdu, gs_mb_excption_resp_pdu_len);
}

static bool poll_rtu_response(bool *crc_ret = nullptr, uint16_t * pdu_len = nullptr)
{
    static uint16_t rtuRxLen = 0;

    bool finished = false;
    bool go_on_read = true;
    while(go_on_read)
    {
        while ((rtuRxLen < gs_mb_max_rtu_adu_len) && g_pdb_serial.available())
        {
            rtuRxBuf[rtuRxLen++] = g_pdb_serial.read();
        }
        if(rtuRxLen >= gs_mb_max_rtu_adu_len && g_pdb_serial.available())
        {
            rtuRxLen = 0; //buffer full. currently just discard received data. maybe do better in future.
        }
        else if(!g_pdb_serial.available())
        {
            go_on_read = false;
        }
    }

    bool complete_resp = false, crc_ok = false;
    if (rtuRxLen >= gs_mb_min_rtu_resp_adu_len)
    {
        uint16_t expect_len = 0;
        mb_func_code_e_t resp_func_code = (mb_func_code_e_t)rtuRxBuf[1]; //the 1st byte is addr byte.
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
                    byte_count = rtuRxBuf[2];
                    expect_len = 1/*addr*/ + 1/*func code*/ + 1/*byte count*/ + byte_count*2 + 2/*crc*/;
                    break;

                case MB_FUNC_CODE_WRITE_SINGLE_REG:
                    expect_len = 1/*addr*/ + 1/*func code*/ + 2/*reg addr*/ + 2/*reg val*/ + 2/*crc*/;
                    break;

                default: //MB_FUNC_CODE_WRITE_MULI_REGS
                    expect_len = 1/*addr*/ + 1/*func code*/ + 2/*start reg addr*/ + 2/*reg count*/ + 2/*crc*/;
                    break;
            }
            complete_resp = (rtuRxLen >= expect_len);
        }

        if(complete_resp)
        {
            uint16_t crcCalc = modbus_crc16(rtuRxBuf, expect_len - 2);
            uint16_t crcRecv = rtuRxBuf[expect_len - 2] |
                               (rtuRxBuf[expect_len - 1] << 8);

            crc_ok = (crcCalc == crcRecv); 
            rtuRxLen = 0;

            if(pdu_len) *pdu_len = expect_len - 3; //omitint the 1st 1 addr byte and the last 2 crc bytes.
        }
    }

    if(crc_ret) *crc_ret = crc_ok;
    return complete_resp;
}

bool send_mb_rtu_request(uint8_t * rtu_pdu, uint16_t pdu_len, uint8_t addr)
{
    uint8_t rtuTxBuf[gs_mb_rtu_addr_len + gs_mb_max_pdu_len + gs_mb_rtu_crc_len];
    uint16_t len = 0;

    if(!rtu_pdu || pdu_len > gs_mb_max_pdu_len)
    {
        return false;
    }
    rtuTxBuf[len++] = addr;
    memcpy(&rtuTxBuf[len], rtu_pdu, pdu_len);
    len += pdu_len;

    uint16_t crc = modbus_crc16(rtuTxBuf, len);
    rtuTxBuf[len++] = crc & 0xFF;
    rtuTxBuf[len++] = crc >> 8;

    g_pdb_serial.write(rtuTxBuf, len);

    rtuSendTime = millis();
    return true;
}

bool read_rtu_response(uint8_t ** rtu_pdu, uint16_t *pdu_len_ptr, uint32_t timeout_ms)
{
    bool complet_msg = false, crc_ok = false, timeout = false;
    uint16_t pdu_len = 0;
    while(!(complet_msg = poll_rtu_response(&crc_ok, &pdu_len)))
    {
        uint32_t curr = millis();
        if(curr - rtuSendTime >= timeout_ms)
        {
            timeout = true;
            break;
        }
    }
    if(rtu_pdu) *rtu_pdu = &rtuRxBuf[1];
    if(pdu_len_ptr) *pdu_len_ptr = pdu_len;
    return (complet_msg && crc_ok && !timeout);
}

static void handle_tcp_response()
{
    uint16_t pdu_len;
    bool ret = read_rtu_response(nullptr, &pdu_len);

    if(ret)
    {
        send_tcp_response(activeClient, pdu_len + 3); //pdu_len + 3 is adu_len. 3: 1 byte of addr, 2 bytes of crc.
    }
    else
    {
        send_tcp_exception(activeClient, ctx[activeClient].pdu[0], MB_GW_TGT_DEV_FAILED_TO_RESP);
    }

    ctx[activeClient].busy = false;
    activeClient = -1;
    gwState = GW_IDLE;
}

static void try_handle_tcp_request(int idx)
{
    WiFiClient& c = tcpClients[idx];
    ClientCtx&  x = ctx[idx];

    if (c.available() < (gs_mb_tcp_mbap_len + gs_mb_func_code_len)) return;

    c.read((uint8_t*)&x.mbap, gs_mb_tcp_mbap_len);

    x.mbap.transId = UINT16_NSTOH(x.mbap.transId);
    x.mbap.length  = UINT16_NSTOH(x.mbap.length);

    x.pduLen = x.mbap.length - 1;
    if (x.pduLen > sizeof(x.pdu)) return;

    c.read(x.pdu, x.pduLen);

    uint8_t fc = x.pdu[0];
    if(!IS_VALID_MB_FUNC_CODE(fc))
    {
        send_tcp_exception(idx, fc, MB_EXCP_ILLEGAL_FUNC);
        return;
    }

    send_mb_rtu_request(x.pdu, x.pduLen, x.mbap.unitId);

    x.busy = true;
    activeClient = idx;
    gwState = GW_WAIT_RTU;
}

static void cleanup_tcp_client(int idx)
{
    if (tcpClients[idx])
    {
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
        if (!tcpClients[i])
        {
            tcpClients[i] = newClient;
            ctx[i].busy = false;
            return;
        }
    }

    // 超过 2 个，直接拒绝
    newClient.stop();
}

static void end_server()
{
    for (int i = 0; i < MAX_TCP_CLIENTS; i++)
    {
        cleanup_tcp_client(i);
    }

    activeClient = -1;
    gwState = GW_IDLE;

    mbServer.end();
}

void modbus_tcp_server(bool work)
{
    static bool server_started = false;

    if(!server_started && work)
    {
        mbServer.begin();
        server_started = true;
    }
    else if(!work)
    {
        if(server_started)
        {
            end_server();
            server_started = false;
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
                handle_tcp_response();
            }
        }
    }
}

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
    if(!buf || (reg_cnt * 2 + 6 > sizeof(pdu))) return false;

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
