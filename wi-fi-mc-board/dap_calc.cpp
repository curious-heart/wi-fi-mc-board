#include "dap_calc.h"

static const float gs_co_kV = 690.0026667, gs_co_mA = 34689.19811, gs_co_sec = 29106, gs_co_D = -67396.55975;
static const float gs_co_size_times = 3.9982002504686105; // (pi * 34.5 * 33.2 / (15*15*4))

float calculate_DAP_value(uint16_t kV, uint32_t uA, uint16_t ms)
{
    float mA = ((float)uA) / 1000, seconds = ((float)ms) / 1000, DAP_v;

    DAP_v = (gs_co_kV * kV + gs_co_mA * mA + gs_co_sec * seconds + gs_co_D) * gs_co_size_times / 100000;

    return DAP_v;
}

void set_float_DAP_to_mapping_reg(float DAP_v, uint16_t * lo_part, uint16_t * hi_part)
{
    uint8_t * reg_B_3, *float_B_0;
    int idx, float_size = 4;
    uint16_t dap_in_reg[2];

    reg_B_3 = (uint8_t*)&dap_in_reg[1] + 1;
    float_B_0 = (uint8_t*)&DAP_v;
    for(idx = 0; idx < float_size; ++idx)
    {
        *(reg_B_3 - idx) = *(float_B_0 + idx);
    }

    if(lo_part) *lo_part = dap_in_reg[1];
    if(hi_part) *hi_part = dap_in_reg[0];
}
