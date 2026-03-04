#ifndef __TOF_DISTANCE_MEASURE_H__
#define __TOF_DISTANCE_MEASURE_H__

bool tof_chip_init();
uint16_t calc_dis(bool req_ava = true);

extern bool g_tof_chip_working;

#endif
