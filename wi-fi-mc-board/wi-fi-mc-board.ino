#include <Arduino.h>
#include <ArduinoJson.h>
#include <WDT.h>

#include <Adafruit_VL53L0X.h>
//ble config wifi
// #include "BLEDevice.h"
// #include "BLEWifiConfigService.h"

#include "wifi_ops.h"
#include "debug_ctrl.h"
#include "json_msg_proc.h"

#include "modbus_ops.h"

constexpr const char gs_wifi_mc_ver_str[] = "wi-fi-mc-1.00x";

static constexpr long gs_scrn_serial_baud = 115200;
static constexpr long gs_pdb_serial_baud = 9600;
static const long gs_pdb_serial_ro_to_ms = 300; //this value should not exceeds RTU_TIMEOUT_MS(in logic).

Stream& g_scrn_serial = Serial;
Stream& g_pdb_serial = Serial1;
Stream& g_dbg_serial = Serial;

static constexpr uint32_t gs_wdt_dura_ms = 30000;

// BLEWifiConfigService configService;
////ble config wifi
WDT wdt;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

char macaddr[50];   //mac地址
char version[50];   //版本号
char jsontype[10];  //数据类型Addr4

enum regIndex {
  // 从 Addr0 到 Addr21
  Addr0 = 0,  //硬件版本|软件版本
  Addr1,      //ota版本
  Addr2,      //波特率
  Addr3,      //设备地址
  Addr4,      //状态字
  Addr5,      //管电压设置值
  Addr6,      //管电流设置
  Addr7,      //曝光时间
  Addr8,      //管电压
  Addr9,      //管电流
  Addr10,     //范围指示状态
  Addr11,     //曝光状态
  Addr12,     //范围指示启动
  Addr13,     //曝光启动申请
  Addr14,     //电池电量
  Addr15,     //电池电压
  Addr16,     //油盒温度
  Addr17,     //关机请求
  Addr18,     //校准位置
  Addr19,     //校准值
  Addr20,     //充能状态
  Addr21,     //曝光次数

  // 接下来续写 101 到 109
  Addr101,  //档位调整事件
  Addr102,  //充电
  Addr103,  //dap 高位
  Addr104,  //dap 低位
  Addr105,  //测距结果
  Addr106,  //wifi热点状态 0xFF 未开启，其余为接入点数量
  Addr107,  //蜂窝信号  高字节  信号0-5  0-无服务 1-3G 2-4G 3-5G
  Addr108,  //高字节 0-4 电池格数   低字节 0-4 wan侧WiFi信号
  Addr109,  //0位：0 充电器未连接  1 充电器链接
            //1位：0 电池未充满    1 电池充满
            //2位：0 wan侧WiFi未连接  1 wan侧WiFi已连接
            //3位：0 wan侧蜂窝网没有连接  1 wan侧蜂窝网已连接
            //4位：0 sim卡异常   1 sim卡正常
  Addr110,  //测距修正


};  //索引

char regValue[Addr110 + 1][6];  //寄存器
String cleanedString;

int parse_str() {
  // 从串口接收数据
  String receivedData = "";
  while (Serial1.available()) {
    char c = Serial1.read();
    receivedData += c;
  }

  if (receivedData.length() == 0) {
    return -2;  //长度错误
  }

  //收到json
  //Serial.print(receivedData);
  if (!(receivedData.startsWith("{")) || (!(receivedData.endsWith("}") || receivedData.endsWith("\n")))) {
    Serial.print("json error, return -3");
    if (!receivedData.startsWith("{")) {
      Serial.print("error \"{\"");
      Serial.println(receivedData[0]);
    }
    if (!receivedData.endsWith("}")) {
      Serial.print("error \"}\"");
      Serial.println(receivedData[receivedData.length() - 1]);
    }
    return -3;  // 非有效json
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, receivedData);

  // Test if parsing succeeds
  if (!error) {
    if (doc.containsKey("json_type"))
      Serial.println("json_type found in JSON");
  } else {
    Serial.print(F("deserializeJson() failed: "));
    Serial.print(F("return -1"));
    return -1;
  }

  strcpy(jsontype, doc["json_type"]);
  Serial.println(jsontype);
  if (!strcmp(jsontype, "info")) {

    if (doc.containsKey("mac_address")) {
      strcpy(macaddr, doc["mac_address"]);
      for (int i = 0; i < strlen(macaddr); i++) {
        if (macaddr[i] != ':') {
          cleanedString += macaddr[i];
        }
      }
      String ssidstr = "GKXG-DR-" + cleanedString.substring(cleanedString.length() - 6, cleanedString.length());
      Serial.println(ssidstr);
    }
    if (doc.containsKey("version")) {
      strcpy(version, doc["version"]);
      Serial.println(version);
    }
  }
  //{"json_type:":"cmd","command":"factory reset"}  路由系统重置
  if (!strcmp(jsontype, "cmd")) {
    wdt.RefreshWatchdog();
    Serial.println("json type cmd");

    if (doc.containsKey("command")) {
      Serial.println("json has cmd");
      if (!strcmp("factory reset", doc["command"])) {
        Serial.println("json type cmd");
        char str1[30];
        strcpy(str1, "系统重置，耗时较长！");
        char str2[30];
        strcpy(str2,  "请耐心等待。。。");
        wdt.RefreshWatchdog();
        delay(3000);
        wdt.RefreshWatchdog();
        delay(2000);
        while (1)
          ;
      }
    }
  }

  if (!strcmp(jsontype, "register")) {
    Serial.println("parse register");
    if (doc.containsKey("0")) {
      strcpy(regValue[Addr0], doc["0"]);
    }
    if (doc.containsKey("1")) {
      strcpy(regValue[Addr1], doc["1"]);
    }
    if (doc.containsKey("2")) {
      strcpy(regValue[Addr2], doc["2"]);
    }
    if (doc.containsKey("3")) {
      strcpy(regValue[Addr3], doc["3"]);
    }
    if (doc.containsKey("4")) {
      strcpy(regValue[Addr5], doc["4"]);
    }
    if (doc.containsKey("5")) {
      strcpy(regValue[Addr5], doc["5"]);
    }
    if (doc.containsKey("6"))
      strcpy(regValue[Addr6], doc["6"]);
    if (doc.containsKey("7"))
      strcpy(regValue[Addr7], doc["7"]);
    if (doc.containsKey("8"))
      strcpy(regValue[Addr8], doc["8"]);
    if (doc.containsKey("9"))
      strcpy(regValue[Addr9], doc["9"]);
    if (doc.containsKey("10"))
      strcpy(regValue[Addr10], doc["10"]);
    if (doc.containsKey("11"))
      strcpy(regValue[Addr11], doc["11"]);
    if (doc.containsKey("12"))
      strcpy(regValue[Addr12], doc["12"]);
    if (doc.containsKey("13"))
      strcpy(regValue[Addr13], doc["13"]);
    if (doc.containsKey("14"))
      strcpy(regValue[Addr14], doc["14"]);
    if (doc.containsKey("15"))
      strcpy(regValue[Addr15], doc["15"]);
    if (doc.containsKey("105"))
      strcpy(regValue[Addr105], doc["105"]);
    if (doc.containsKey("106"))
      strcpy(regValue[Addr106], doc["106"]);
    if (doc.containsKey("107"))
      strcpy(regValue[Addr107], doc["107"]);
    if (doc.containsKey("108"))
      strcpy(regValue[Addr108], doc["108"]);
    if (doc.containsKey("109"))
      strcpy(regValue[Addr109], doc["109"]);
    if (doc.containsKey("110"))
      strcpy(regValue[Addr110], doc["110"]);
  }
  Serial.println("rcv json");
  return 0;
}

VL53L0X_RangingMeasurementData_t measure;
#define numReadings 20
int readings[numReadings];
int readIndex;
int total;
bool isTofOn;
/*
"key_name":"add" 或"sub"

"key_val":"press"或"release"
*/
#define GEAR_UP   PA12
#define GEAR_DOWN PA27
#define XSHUTDOWN PA14
#define BLC       PB3

int calc_dis(void) {

  // Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false);  // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
  } else {
    return -1;  //超限
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
  int reg0 = readings[0];
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
    Serial.println("Adafruit VL53L0X sleep");
    Serial.println(loop);
    Serial.println(reg0);
    digitalWrite(XSHUTDOWN, LOW);
    delay(10000);
    digitalWrite(XSHUTDOWN, HIGH);
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


char cmd[20];


void setup(void) {

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
  while (!Serial) {
    delay(1);
  }

  while (!Serial1) {
    delay(1);
  }
  Serial1.setTimeout(gs_pdb_serial_ro_to_ms);

  g_dbg_serial.println(String("\n\nstart to run: ") + String(gs_wifi_mc_ver_str));

  String wifi_status_str = String("WiFi status:") + String(WiFi.status());
  g_dbg_serial.println(wifi_status_str);
  printMacAddress();

  g_dbg_serial.println(F("init Adafruit VL53L0X......"));
  //if (!lox.begin(VL53L0X_I2C_ADDR, true, &Wire)) {
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    delay(10000);
  }
  wdt.RefreshWatchdog();

  // power
  g_dbg_serial.println(F("VL53L0X started\n"));


  pinMode(BLC, OUTPUT);
  digitalWrite(BLC, LOW);

  wifi_init();
}

long lastsend;

static const uint32_t MAITAIN_NW_PERIOD = 5;
void loop(void)
{
  static uint32_t maitain_nw_cnt = 0;

 /*
  //急停状态，只显示急停  别的不显示
  while (atoi(regValue[Addr4]) & 0x10) {
    //"系统急停，请确认状态后,长按开机键解除急停!"
  }
*/

  /*
  maitain network
  process hard-key
  process (json) msg from screen
  process msg from modbus client
  read tof distance measurement
  timer (refresh screen)
  */

  /*
      maitain network
  */
  if(((maitain_nw_cnt % MAITAIN_NW_PERIOD) == 0) && (WL_CONNECTED != curr_wifi_status()))
  {
    DBG_PRINTLN(LOG_WARN, "Wi-Fi is disconnected. Now try connecting...");
    connect_wifi();
    DBG_PRINT(LOG_INFO, "Wi-Fi status :"); DBG_PRINTLN(LOG_INFO, curr_wifi_status());
  }
  ++maitain_nw_cnt;

  /*
      process (json) msg from screen
  */
  json_msg_recv_proc(g_scrn_serial);


  /*
    process msg from modbus client
  */
   modbus_tcp_server(WL_CONNECTED == curr_wifi_status());

  // 平均值
  //int average = calc_dis();
  //g_dbg_serial.print(F("distance:"));
  //g_dbg_serial.println(average);


/*
  if ((millis() - lastsend) > 2000) {

    lastsend = millis();
    JsonDocument doc;
    // Add values in the document
    doc["json_type"] = "data";
    doc["tof_distance"] = String(average);

    // Generate the minified JSON and send it to the Serial port
    serializeJson(doc, Serial1);
  }
*/
  wdt.RefreshWatchdog();
  //delay(2000);
}
