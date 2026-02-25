#ifndef _MODBUS_INTERNAL_H__
#define _MODBUS_INTERNAL_H__

typedef enum
{
    MB_FUNC_CODE_READ_HREG = 0x03,
    MB_FUNC_CODE_WRITE_SINGLE_REG = 0x06,
    MB_FUNC_CODE_WRITE_MULI_REGS = 0x10,

    MB_EXCP_FUNC_CODE_START = 0x80,
}mb_func_code_e_t;

#define IS_VALID_MB_FUNC_CODE(c) \
(((c) == MB_FUNC_CODE_READ_HREG) || ((c) == MB_FUNC_CODE_WRITE_SINGLE_REG) || ((c) == MB_FUNC_CODE_WRITE_MULI_REGS))

typedef enum
{
    MB_EXCP_ILLEGAL_FUNC = 0x01,
    MB_GW_TGT_DEV_FAILED_TO_RESP = 0x0B,
}mb_exception_code_e_t;

#define RTU_TIMEOUT_MS 800
#define MODBUS_RTU_SRV_ADDR 1
bool send_mb_rtu_request(uint8_t * rtu_pdu, uint16_t pdu_len, uint8_t addr = MODBUS_RTU_SRV_ADDR);

/*
 * rtu_pdu: OUT. buffer holding pdu read.
 * pdu_len: OUT. holding pdu len.
 * */
bool read_rtu_response(uint8_t ** rtu_pdu = nullptr, uint16_t *pdu_len = nullptr, uint32_t timeout_ms = RTU_TIMEOUT_MS);

#define TCP_ADU_READ_TO_MS 300
bool read_tcp_req_adu(int idx, uint32_t timeout_ms = TCP_ADU_READ_TO_MS);
void send_tcp_exception(int idx, uint8_t fc, uint8_t code);
void send_tcp_response(int idx, uint8_t * pdu_buf, uint16_t pdu_len);

#define UINT16_HI_BYTE(d) ((uint8_t)((d) >> 8))
#define UINT16_LO_BYTE(d) ((uint8_t)((d) & 0xFF))

#endif
