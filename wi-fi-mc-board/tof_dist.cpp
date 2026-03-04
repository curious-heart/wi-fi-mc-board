#include <Arduino.h>
#include <Adafruit_VL53L0X.h>

#include "common_defs.h"
#include "debug_ctrl.h"
#include "gpio_pin_process.h"

#include "modbus_ops.h"

bool g_tof_chip_working = false;

static const unsigned long gs_tof_pwr_on_off_gap_ms = 3000;

static Adafruit_VL53L0X lox = Adafruit_VL53L0X();

static VL53L0X_RangingMeasurementData_t measure;
#define numReadings 20
static uint16_t readings[numReadings] = {0};
static int readIndex;
static uint32_t total;

bool tof_chip_init()
{
    g_dbg_serial.println(F("init Adafruit VL53L0X......"));
    if (!lox.begin())
    {
        g_tof_chip_working = false;
        g_dbg_serial.println(F("Failed to boot VL53L0X, expo is disabled."));
        /*
        wdt.StopWatchdog();
        wdt.InitWatchdog(gs_wdt_reset_wait_ms);
        while(1);
        */
    }
    else
    {
        g_tof_chip_working = true;
        g_dbg_serial.println(F("VL53L0X started\n"));
    }

    return g_tof_chip_working;
}

uint16_t calc_dis(bool req_ava)
{
    static unsigned long ls_tof_pwr_off_time_ms; //when tof is power off
    static bool ls_tof_resetting = false;
    static const uint16_t ls_invalid_dis = (uint16_t)-1;

    uint16_t single_val;
    int complement_val = (int)(g_mb_ext_reg_val[EXT_TOF_DIST_COMP_MM - FIRST_EXT_REG_NO]);

#define COMPLEMENT_VAL(v) (uint16_t)((int)(v) + complement_val)

    //tof is powered off. wait some time.
    if(ls_tof_resetting && (millis() - ls_tof_pwr_off_time_ms < gs_tof_pwr_on_off_gap_ms))
    {
        return ls_invalid_dis;
    }

    //now power on tof.
    if(ls_tof_resetting)
    {
        set_shutdown_pin(HIGH);
        ls_tof_resetting = false;

        g_tof_chip_working = true;
    }

    lox.rangingTest(&measure, false);  // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    } else {
      return ls_invalid_dis;  //超限
    }

    total = total - readings[readIndex];
    // 读入当前旋转电位计的数值，
    // 并将其存储到数组的最后一位。
    single_val = measure.RangeMilliMeter;
    readings[readIndex] = single_val;
    // 将最新读入的数值加入到总值中
    total = total + readings[readIndex++];
    // 将数组指示索引值加1
    // 判断数组指示索引是否超出数组范围，
    // 如果是，将数组指示索引重置为0
    if (readIndex >= numReadings) {
      readIndex = 0;
    }

    uint16_t reg0 = readings[0];
    if((reg0 != 8191)&& (reg0 != 0))
    {
        int loop=0;
        for(; loop< numReadings; loop++)
        {
          if(readings[loop] != reg0)
            break;
        }

        if(loop==numReadings)
        {
            DBG_PRINT(LOG_WARN, F("reset tof because it the data is frozen at ")); DBG_PRINTLN(LOG_WARN, reg0);
            set_shutdown_pin(LOW);
            ls_tof_resetting = true;  ls_tof_pwr_off_time_ms = millis();

            g_tof_chip_working = false;
            return COMPLEMENT_VAL(reg0);
        }
    }
    // 计算平均值
    return COMPLEMENT_VAL(req_ava ? (total / numReadings) : single_val);
}
