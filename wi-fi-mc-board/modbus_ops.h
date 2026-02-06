#ifndef __MODBUS_OPS_H__
#define __MODBUS_OPS_H__

void modbus_tcp_server(bool work = true);

bool hv_controller_write_single_reg(uint16_t reg_addr, uint16_t value);
bool hv_controller_write_mult_regs(uint16_t reg_addr_start, uint16_t *buf, int len);
bool hv_controller_read_regs(uint16_t reg_addr_start, uint16_t * buf, int len);

#endif
