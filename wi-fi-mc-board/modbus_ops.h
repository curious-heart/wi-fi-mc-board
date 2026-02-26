#ifndef __MODBUS_OPS_H__
#define __MODBUS_OPS_H__

typedef enum
{
    HSV = 0,                            /*软硬件版本*/
    OTA = 1,                            /*OTA升级*/
    BaudRate = 2,                       /*波特率*/
    ServerAddress = 3,                  /*设备地址*/
    State = 4,                          /*状态*/
    VoltSet = 5,                        /*5管电压设置值*/
    FilamentSet = 6,                    /*6 管设置值电流 （决定灯丝电流决定管电流）*/
    ExposureTime = 7,                   /*曝光时间*/
    Voltmeter = 8,                      /*管电压读出值*/
    Ammeter = 9,                        /*管电流读出值*/
    RangeIndicationStatus = 10,         /*范围指示状态*/
    ExposureStatus = 11,                /*曝光状态*/
    RangeIndicationStart = 12,          /*范围指示启动*/
    ExposureStart = 13,                 /*曝光启动*/
    BatteryLevel = 14,                  /*电池电量*/
    BatteryVoltmeter = 15,
    OilBoxTemperature = 16,             /*电池电压高位*/
    Poweroff = 17,                      /*关机请求*/
    Fixpos = 18,                        /*校准定义*/
    Fixval = 19,                        /*校准值*/
    Workstatus = 20,                    /*充能状态*/
    exposureCount = 21,                 /*曝光次数*/

    MAX_HV_NORMAL_MB_REG_NUM, /*normal register end flag.*/

    EXT_MB_REG_START_FLAG = 100, /*extend register start flag.*/
    /*Below are extend register, that is, they are processed internally by server and not passed to hv controller.*/ 
    EXT_MB_REG_DOSE_ADJ = 101,                       /*+/- key event*/
    EXT_MB_REG_CHARGER = 102,                       /*charger plug in/pull out*/
    EXT_MB_REG_DAP_HP = 103,                       /*High part of a float of DAP(Dose Area Product), big endian.*/
    EXT_MB_REG_DAP_LP = 104,                       /*Low part of a float of DAP, big endian.*/
    EXT_MB_REG_DISTANCE = 105,                       /* uint16，测距结果。单位mm*/
    EXT_MB_REG_HOTSPOT_ST = 106,           /*uint16，本机Wi-Fi热点状态。*/
                                              /*0xFFFF表示Wi-Fi热点未开启；*/
                                              /*其它值表示热点开启，数值指示热点上连接的Client数量*/
    EXT_MB_REG_CELLUAR_ST = 107,              /*uint16，高字节表示蜂窝网的信号格数，有效值0~5；*/
                                                         /*低字节表示蜂窝网状态：0-无服务；1-3G；2-4G；3-5G*/
    EXT_MB_REG_WIFI_WAN_SIG_AND_BAT_LVL = 108, /*uint16，高字节指示电池电量(BatteryLevel)；*/
                                                           /*低字节指示WAN侧Wi-Fi信号格数，有效值0~4*/
    EXT_MB_REG_DEV_INFO_BITS = 109, /*uint16的每个bit指示一个设备的二值状态信息：*/
                                        /*第0位：0-没有连接充电器，1-连接充电器。*/
                                        /*第1位：0-电池电量未充满，1-电池电量已充满。这一位目前无法使用，因为路由板的GPIO没办法读取到正确的状态。*/
                                        /*第2位：0-WAN侧Wi-Fi没有连接；1-WAN侧Wi-Fi已连接。*/
                                        /*第3位：0-WAN侧蜂窝网没有连接；1-WAN侧蜂窝网已连接。*/
                                        /*第4位：0-SIM卡异常（未插卡，或者卡锁定、卡不识别）；1-SIM卡正常识别。*/

    EXT_TOF_DIST_COMP_MM = 110, /*compensation for TOF distance. unit is mm.*/ 
    HV_MB_REG_END_FLAG, /*register end flag.*/
}hv_mb_reg_e_t;

void modbus_tcp_server(bool work = true);
void end_mb_tcp_server();

bool hv_controller_write_single_reg(uint16_t reg_addr, uint16_t value);
bool hv_controller_write_mult_regs(uint16_t reg_addr_start, uint16_t *buf, int reg_cnt);
bool hv_controller_read_regs(uint16_t reg_addr_start, uint16_t * buf, int reg_cnt);

#endif
