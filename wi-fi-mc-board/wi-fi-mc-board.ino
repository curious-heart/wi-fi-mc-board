#include <Arduino.h>
#include <ArduinoJson.h>
#include <WDT.h>

#include <Adafruit_VL53L0X.h>

#include "wifi_ops.h"
#include "debug_ctrl.h"
#include "json_msg_proc.h"

#include "modbus_ops.h"

constexpr const char g_dev_maj_ver[] = "v1";
constexpr const char g_wifi_mc_ver_str[] = "d014";

bool g_tof_chip_working = false;
bool g_allow_force_exposure_ignoring_dist = false;
uint16_t g_min_dist_for_expo_mm = 200;

static constexpr long gs_scrn_serial_baud = 115200;
static constexpr long gs_pdb_serial_baud = 9600;
static const long gs_pdb_serial_ro_to_ms = 300; //this value should not exceeds RTU_TIMEOUT_MS(in logic).

Stream& g_scrn_serial = Serial;
Stream& g_pdb_serial = Serial1;
Stream& g_dbg_serial = Serial;

static constexpr uint32_t gs_wdt_dura_ms = 30000;
static constexpr uint32_t gs_wdt_reset_wait_ms = 10000;

static const unsigned long gs_tof_pwr_on_off_gap_ms = 3000;

static const unsigned long gs_maitain_nw_period_ms = 3000;
static const unsigned long gs_tof_report_period_ms = 2000;
static const unsigned long gs_mb_reg_rpt_period_ms = 4000;
static const unsigned long gs_dev_info_rpt_period_ms = 30000;

static const unsigned long gs_expo_key_held_to_ms = 5000;

WDT wdt;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const char* get_dev_ver_str()
{
#define MAX_DEV_STR_LEN 20
    static char ls_ver_str[MAX_DEV_STR_LEN + 1] = {0};
    static bool getted = false;

    const char invalid_pdb_ver_str[] = "0";

    if(!getted)
    {
        const char * pdb_ver = get_pdb_ver_str();
        getted = (pdb_ver != nullptr);

        if(!pdb_ver) pdb_ver = invalid_pdb_ver_str;
        snprintf(ls_ver_str, sizeof(ls_ver_str), "%s %s.%s.%s",
                                                 g_dev_maj_ver, g_dev_maj_ver, pdb_ver, g_wifi_mc_ver_str);

    }

    return ls_ver_str;
}

VL53L0X_RangingMeasurementData_t measure;
#define numReadings 20
uint16_t readings[numReadings] = {0};
int readIndex;
uint32_t total;
/*
"key_name":"add" 或"sub"

"key_val":"press"或"release"
*/
#define EXPO_KEY   PA27// PA12
#define RANGE_LIGH_KEY PA30 //PA27
#define XSHUTDOWN PA14
#define BLC       PB3

#define CHARGER_ST   PA12
#define CHARG_SC_PG   PA15

uint16_t calc_dis(bool req_ava = true)
{
    static unsigned long ls_tof_pwr_off_time_ms; //when tof is power off
    static bool ls_tof_resetting = false;
    static const uint16_t ls_invalid_dis = (uint16_t)-1;

    uint16_t single_val;

    //tof is powered off. wait some time.
    if(ls_tof_resetting && (millis() - ls_tof_pwr_off_time_ms < gs_tof_pwr_on_off_gap_ms))
    {
        return ls_invalid_dis;
    }

    //now power on tof.
    if(ls_tof_resetting)
    {
        digitalWrite(XSHUTDOWN, HIGH);
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
            digitalWrite(XSHUTDOWN, LOW);
            ls_tof_resetting = true;  ls_tof_pwr_off_time_ms = millis();

            g_tof_chip_working = false;
            return reg0;
        }
    }
    // 计算平均值
    return req_ava ? (total / numReadings) : single_val ;
}

bool expo_is_allowed()
{
    if(g_allow_force_exposure_ignoring_dist) return true;

    uint16_t curr_dist = calc_dis(false);
    DBG_PRINTLN(LOG_DEBUG, F("dist in expo_is_allowed: ")); DBG_PRINTLN(LOG_DEBUG, curr_dist);
    return (curr_dist >= g_min_dist_for_expo_mm);
}

volatile static uint32_t gs_key_press_dbg_cnt = 0, gs_key_release_dbg_cnt = 0;

volatile static bool gs_expo_key_pressed = false;
volatile static unsigned long gs_expo_key_pressed_time;
volatile static bool gs_expo_key_held_handled = true;
static void expo_key_switched(uint32_t /*id*/, uint32_t /*event*/)
{
    if(HIGH == digitalRead(EXPO_KEY))
    {//released
        gs_expo_key_pressed = false;
        gs_expo_key_held_handled = true;

        gs_key_release_dbg_cnt++;
    }
    else
    {
        gs_expo_key_pressed = true;
        gs_expo_key_pressed_time = millis();
        gs_expo_key_held_handled = false;

        gs_key_press_dbg_cnt++;
    }
}

volatile static bool gs_range_light_key_pressed = false;
volatile static bool gs_range_light_key_handled = true;
static void range_light_key_switched(uint32_t /*id*/, uint32_t /*event*/)
{
    if(HIGH == digitalRead(RANGE_LIGH_KEY))
    {//released
        gs_range_light_key_pressed = false;
        gs_range_light_key_handled = true;

    }
    else
    {
        gs_range_light_key_pressed = true;
        gs_range_light_key_handled = false;

    }
}

static void process_hardware_key()
{
    if(gs_expo_key_pressed)
    {
        /*
        DBG_PRINT(LOG_ERROR, F("press and release cnt: ")); DBG_PRINT(LOG_ERROR, gs_key_press_dbg_cnt);
        DBG_PRINT(LOG_ERROR, F(", ")); DBG_PRINTLN(LOG_ERROR, gs_key_release_dbg_cnt);
        */

        if(!gs_expo_key_held_handled && (millis() - gs_expo_key_pressed_time >= gs_expo_key_held_to_ms))
        {
            start_expo();
            gs_expo_key_held_handled = true;
        }
    }

    if(gs_range_light_key_pressed)
    {
        if(!gs_range_light_key_handled)
        {
            static bool turn_on = true;
            switch_range_light(turn_on); turn_on = !turn_on;
            gs_range_light_key_handled = true;
        }
    }
}

void setup(void)
{
    Serial.begin(gs_scrn_serial_baud);
    Serial1.begin(gs_pdb_serial_baud);

    //初始化看门狗
    wdt.InitWatchdog(gs_wdt_dura_ms);  // setup watchdog
    wdt.StartWatchdog();     // enable watchdog timer

    //加减档按钮
    pinMode(EXPO_KEY, INPUT_IRQ_CHANGE);  //
    digitalSetIrqHandler(EXPO_KEY, expo_key_switched);

    pinMode(RANGE_LIGH_KEY, INPUT_IRQ_CHANGE);  //
    digitalSetIrqHandler(RANGE_LIGH_KEY, range_light_key_switched);

    pinMode(XSHUTDOWN, OUTPUT); 
    digitalWrite(XSHUTDOWN, HIGH);

    //wait until serial port opens for native USB devices
    while (!Serial) { delay(1); }

    while (!Serial1) { delay(1);}
    Serial1.setTimeout(gs_pdb_serial_ro_to_ms);

    g_dbg_serial.print(F("\n\nstart to run: ")); g_dbg_serial.println(g_wifi_mc_ver_str);

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

    wdt.RefreshWatchdog();

    pinMode(BLC, OUTPUT);
    digitalWrite(BLC, LOW);

    wifi_init();
}

void loop(void)
{
    static bool start_up = true;
    static unsigned long ls_last_tof_rpt_time, ls_last_dev_inf_rpt_time, ls_last_reg_rpt_time,
                         ls_maitain_nw_rpt_time;
    static wl_status_t ls_last_rec_wifi_staus = WL_DISCONNECTED;
 
   /*
    //急停状态，只显示急停  别的不显示
    while (atoi(regValue[Addr4]) & 0x10) {
      //"系统急停，请确认状态后,长按开机键解除急停!"
    }
  */
 
    /* process hard-key */
    process_hardware_key();
 
    /* maitain network */
    if(start_up || ((millis() - ls_maitain_nw_rpt_time) >= gs_maitain_nw_period_ms))
    {
        if(WL_CONNECTED != curr_wifi_status())
        {
            DBG_PRINTLN(LOG_WARN, "Wi-Fi is disconnected. Now try connecting...");
            connect_wifi();
            DBG_PRINT(LOG_INFO, "Wi-Fi status :"); DBG_PRINTLN(LOG_INFO, curr_wifi_status());
        }

        wl_status_t curr_status = curr_wifi_status();
        if(start_up || (curr_status != ls_last_rec_wifi_staus))
        {
            rpt_network_info_json();
            ls_last_rec_wifi_staus = curr_status;
        }

        start_up = false;
        ls_maitain_nw_rpt_time = millis();
    }
 
    /* process (json) msg from screen */
    json_msg_recv_proc(g_scrn_serial);

    /* process msg from modbus client */
     modbus_tcp_server(WL_CONNECTED == curr_wifi_status());

    /* report tof distance */
    uint16_t average = calc_dis();
    if((millis() - ls_last_tof_rpt_time) >= gs_tof_report_period_ms)
    {
        rpt_tof_dis_json(average);
        ls_last_tof_rpt_time = millis();
    }

    /* other periodically ops*/
    if((millis() - ls_last_dev_inf_rpt_time) >= gs_dev_info_rpt_period_ms)
    {
        rpt_dev_info_json();
        ls_last_dev_inf_rpt_time = millis();
    }
    if((millis() - ls_last_reg_rpt_time) >= gs_mb_reg_rpt_period_ms)
    {
        rpt_mb_reg_json();
        ls_last_reg_rpt_time = millis();
    }

    wdt.RefreshWatchdog();
}
