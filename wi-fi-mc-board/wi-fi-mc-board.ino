#include <Arduino.h>
#include <ArduinoJson.h>
#include <WDT.h>

#include <Adafruit_VL53L0X.h>

#include "wifi_ops.h"
#include "debug_ctrl.h"
#include "json_msg_proc.h"

#include "modbus_ops.h"

constexpr const char g_dev_maj_ver[] = "v1";
constexpr const char g_wifi_mc_ver_str[] = "d003";

static constexpr long gs_scrn_serial_baud = 115200;
static constexpr long gs_pdb_serial_baud = 9600;
static const long gs_pdb_serial_ro_to_ms = 300; //this value should not exceeds RTU_TIMEOUT_MS(in logic).

Stream& g_scrn_serial = Serial;
Stream& g_pdb_serial = Serial1;
Stream& g_dbg_serial = Serial;

static constexpr uint32_t gs_wdt_dura_ms = 30000;
static constexpr uint32_t gs_wdt_reset_wait_ms = 10000;

static const unsigned long gs_tof_pwr_on_off_gap_ms = 3000;
static const unsigned long gs_tof_report_period_ms = 2000;
static const unsigned long gs_dev_info_rpt_period_ms = 30000;
static const unsigned long gs_mb_reg_rpt_period_ms = 4000;

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
#define GEAR_UP   PA12
#define GEAR_DOWN PA27
#define XSHUTDOWN PA14
#define BLC       PB3

uint16_t calc_dis(void)
{
    static unsigned long ls_tof_pwr_off_time_ms; //when tof is power off
    static bool ls_tof_resetting = false;
    static const uint16_t ls_invalid_dis = (uint16_t)-1;

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
    }

    lox.rangingTest(&measure, false);  // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    } else {
      return ls_invalid_dis;  //超限
    }

    total = total - readings[readIndex];
    // 读入当前旋转电位计的数值，
    // 并将其存储到数组的最后一位。
    readings[readIndex] = measure.RangeMilliMeter;
    // 将最新读入的数值加入到总值中
    total = total + readings[readIndex++];
    // 将数组指示索引值加1
    // 判断数组指示索引是否超出数组范围，
    // 如果是，将数组指示索引重置为0
    if (readIndex >= numReadings) {
      readIndex = 0;
    }
    //调试
    uint16_t reg0 = readings[0];
    if((reg0==8191)||(reg0==0))
      return total / numReadings;;
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
        return reg0;
    }
    // 计算平均值
    return total / numReadings;
}

void GearUpChange(uint32_t id, uint32_t event) {

  if (digitalRead(GEAR_UP)) {
    Serial.println("gear up releaseed");
    JsonDocument doc;
    // Add values in the document
    doc["json_type"] = "data";
    doc["key_name"] = "add";
    doc["key_val"] = "released";
    // Generate the minified JSON and send it to the Serial port
    serializeJson(doc, Serial1);
  } else {
    Serial.println("gear up pressed");
    JsonDocument doc;
    // Add values in the document
    doc["json_type"] = "data";
    doc["key_name"] = "add";
    doc["key_val"] = "pressed";
    // Generate the minified JSON and send it to the Serial port
    serializeJson(doc, Serial1);
  }
}

void GearDownChange(uint32_t id, uint32_t event) {

  if (digitalRead(GEAR_DOWN)) {
    Serial.println("gear up released");
    JsonDocument doc;
    // Add values in the document
    doc["json_type"] = "data";
    doc["key_name"] = "sub";
    doc["key_val"] = "released";
    // Generate the minified JSON and send it to the Serial port
    serializeJson(doc, Serial1);
  } else {
    Serial.println("gear up pressed");
    JsonDocument doc;
    // Add values in the document
    doc["json_type"] = "data";
    doc["key_name"] = "sub";
    doc["key_val"] = "pressed";
    // Generate the minified JSON and send it to the Serial port
    serializeJson(doc, Serial1);
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
    pinMode(GEAR_UP, INPUT_IRQ_CHANGE);  //
    digitalSetIrqHandler(GEAR_UP, GearUpChange);

    pinMode(GEAR_DOWN, INPUT_IRQ_CHANGE);  //
    digitalSetIrqHandler(GEAR_DOWN, GearDownChange);

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
        g_dbg_serial.println(F("Failed to boot VL53L0X, reset device in several secondes..."));
        wdt.StopWatchdog();
        wdt.InitWatchdog(gs_wdt_reset_wait_ms);
        while(1);
    }
    wdt.RefreshWatchdog();

    g_dbg_serial.println(F("VL53L0X started\n"));

    pinMode(BLC, OUTPUT);
    digitalWrite(BLC, LOW);

    wifi_init();
}

static const uint32_t MAITAIN_NW_PERIOD = 5;
void loop(void)
{
    static uint32_t maitain_nw_cnt = 0;
    static unsigned long ls_last_tof_rpt_time, ls_last_dev_inf_rpt_time, ls_last_reg_rpt_time;
 
   /*
    //急停状态，只显示急停  别的不显示
    while (atoi(regValue[Addr4]) & 0x10) {
      //"系统急停，请确认状态后,长按开机键解除急停!"
    }
  */
 
    /*
    process hard-key
    */
 
    /* maitain network */
    if(((maitain_nw_cnt % MAITAIN_NW_PERIOD) == 0) && (WL_CONNECTED != curr_wifi_status()))
    {
        DBG_PRINTLN(LOG_WARN, "Wi-Fi is disconnected. Now try connecting...");
        connect_wifi();
        DBG_PRINT(LOG_INFO, "Wi-Fi status :"); DBG_PRINTLN(LOG_INFO, curr_wifi_status());
    }
    ++maitain_nw_cnt;
 
    /* process (json) msg from screen */
    json_msg_recv_proc(g_scrn_serial);

    /* process msg from modbus client */
     modbus_tcp_server(WL_CONNECTED == curr_wifi_status());

    /* report tof distance */
    uint16_t average = calc_dis();
    if ((millis() - ls_last_tof_rpt_time) >= gs_tof_report_period_ms)
    {
        rpt_tof_dis_json(average);
        ls_last_tof_rpt_time = millis();
    }

    /* other periodically ops*/
    if ((millis() - ls_last_dev_inf_rpt_time) >= gs_dev_info_rpt_period_ms)
    {
        rpt_dev_info_json();
        ls_last_dev_inf_rpt_time = millis();
    }
    if ((millis() - ls_last_reg_rpt_time) >= gs_mb_reg_rpt_period_ms)
    {
        rpt_mb_reg_json();
        ls_last_reg_rpt_time = millis();
    }

    wdt.RefreshWatchdog();
}
