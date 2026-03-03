#ifndef _DAP_CALCULATION_H_
#define _DAP_CALCULATION_H_

#include "common_defs.h"

float calculate_DAP_value(uint16_t kV, uint32_t uA, uint16_t ms);
void set_float_DAP_to_mapping_reg(float DAP_v, uint16_t * lo_part, uint16_t * hi_part);

#endif

