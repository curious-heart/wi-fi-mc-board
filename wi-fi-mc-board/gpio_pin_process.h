#ifndef __GPIO_PIN_PROCESS_H__
#define __GPIO_PIN_PROCESS_H__

void gpio_pin_init();

void process_hardware_key();
uint16_t get_chg_st_in_reg_form();

void set_shutdown_pin(uint32_t val);

#endif
