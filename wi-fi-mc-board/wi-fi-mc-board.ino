#include <Arduino.h>
#include <ArduinoJson.h>
#include <WDT.h>

#include <Adafruit_VL53L0X.h>

#include "wifi_ops.h"
#include "debug_ctrl.h"
#include "json_msg_proc.h"

#include "modbus_ops.h"
#include "gpio_pin_process.h"
#include "tof_dist.h"

constexpr const char g_dev_maj_ver[] = "v1";
constexpr const char g_wifi_mc_ver_str[] = "d015-k";

bool gs_allow_force_exposure_ignoring_dist = false;
uint16_t g_min_dist_for_expo_mm = 200;

static constexpr long gs_scrn_serial_baud = 115200;
static constexpr long gs_pdb_serial_baud = 9600;
static const long gs_pdb_serial_ro_to_ms = 300; //this value should not exceeds RTU_TIMEOUT_MS(in logic).

Stream& g_scrn_serial = Serial;
Stream& g_pdb_serial = Serial1;
Stream& g_dbg_serial = Serial;

static constexpr uint32_t gs_wdt_dura_ms = 30000;
static constexpr uint32_t gs_wdt_reset_wait_ms = 10000;


static const unsigned long gs_maitain_nw_period_ms = 3000;
static const unsigned long gs_tof_report_period_ms = 2000;
static const unsigned long gs_mb_reg_rpt_period_ms = 10000;
static const unsigned long gs_dev_info_rpt_period_ms = 30000;


WDT wdt;

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

static bool gs_mb_reg_written = false, gs_just_rpt_mb_reg = false;
void set_mb_reg_written_flag(bool wr)
{
    gs_mb_reg_written = wr;
}
void set_just_rpt_mb_reg_flag(bool rpt)
{
    gs_just_rpt_mb_reg = rpt;
}

void allow_force_expo_ig_dist(bool flag)
{
    gs_allow_force_exposure_ignoring_dist = flag;
}
 
bool expo_is_allowed()
{
    if(gs_allow_force_exposure_ignoring_dist) return true;

    uint16_t curr_dist = calc_dis(false);
    DBG_PRINTLN(LOG_DEBUG, F("dist in expo_is_allowed: ")); DBG_PRINTLN(LOG_DEBUG, curr_dist);
    return (curr_dist >= g_min_dist_for_expo_mm);
}

void setup(void)
{
    Serial.begin(gs_scrn_serial_baud);
    Serial1.begin(gs_pdb_serial_baud);

    //初始化看门狗
    wdt.InitWatchdog(gs_wdt_dura_ms);  // setup watchdog
    wdt.StartWatchdog();     // enable watchdog timer

    gpio_pin_init();

    //wait until serial port opens for native USB devices
    while (!Serial) { delay(1); }

    while (!Serial1) { delay(1);}
    Serial1.setTimeout(gs_pdb_serial_ro_to_ms);

    g_dbg_serial.print(F("\n\nstart to run: ")); g_dbg_serial.println(g_wifi_mc_ver_str);

    tof_chip_init();

    wdt.RefreshWatchdog();

    wifi_init();
}

void loop(void)
{
    static bool start_up = true;
    static unsigned long ls_last_tof_rpt_time, ls_last_dev_inf_rpt_time, ls_last_reg_rpt_time,
                         ls_maitain_nw_rpt_time;
    static wl_status_t ls_last_rec_wifi_staus = WL_DISCONNECTED;
 
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

    if(gs_mb_reg_written || gs_just_rpt_mb_reg || ((millis() - ls_last_reg_rpt_time) >= gs_mb_reg_rpt_period_ms))
    {
        rpt_mb_reg_json();
        ls_last_reg_rpt_time = millis();

        gs_mb_reg_written = false;
        gs_just_rpt_mb_reg = false;
    }

    wdt.RefreshWatchdog();
}
