#ifndef __COMMON_DEFS_H__
#define __COMMON_DEFS_H__

#include <Arduino.h>
#include <WDT.h>

extern Stream& g_scrn_serial;
extern Stream& g_pdb_serial;
extern Stream& g_dbg_serial;
extern WDT wdt;

#define HEXD_2_CH_CAP(x) ((0 <= (x) && (x) <= 9)) ? ((x) + '0') : ((x) - 10 + 'A')

#define UINT16_HTONS(d) (uint16_t)((((uint16_t)(d)) << 8) | (((uint16_t)(d)) >> 8))
#define UINT16_NSTOH(d) UINT16_HTONS(d)

#define ARRAY_ITEM_CNT(arr) (sizeof(arr)/sizeof(arr[0]))

#endif
