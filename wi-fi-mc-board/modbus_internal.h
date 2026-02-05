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

#define RTU_TIMEOUT_MS 200
#define MODBUS_RTU_SRV_ADDR 1
bool send_mb_rtu_request(uint8_t * rtu_pdu, uint16_t pdu_len, uint8_t addr = MODBUS_RTU_SRV_ADDR);
void read_rtu_response(unsigned long send_time, bool ack_tcp = true, uint32_t timout_ms = RTU_TIMEOUT_MS);

#endif
