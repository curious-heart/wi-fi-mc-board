#include "common_defs.h"

#include "debug_ctrl.h"
#include "gpio_pin_process.h"
#include "modbus_ops.h"
#include "json_msg_proc.h"

#define EXPO_KEY   PA27// PA12
#define RANGE_LIGH_KEY PA30 //PA27

#define CHARGER_ST   PA12
#define CHARG_FULL_SC_PG   PA15

#define XSHUTDOWN PA14
#define BLC       PB3

void set_just_rpt_mb_reg_flag(bool rpt);
void set_mb_rtu_operated_flag(bool op);

static const unsigned long gs_expo_key_held_to_ms = 5000;

/*begin: these vars are for debug*/
volatile static bool gs_key_dbg_flag = false;
volatile static uint32_t gs_key_press_dbg_cnt = 0, gs_key_release_dbg_cnt = 0;
volatile static uint32_t gs_irq_id = 0, gs_irq_event = 0;
/*end*/

/*----------------------------------------*/
volatile static bool gs_expo_key_pressed = false;
volatile static unsigned long gs_expo_key_pressed_time;
volatile static bool gs_expo_key_held_handled = true;
static void expo_key_switched(uint32_t id, uint32_t event)
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

    gs_key_dbg_flag = true;
    gs_irq_id = id; gs_irq_event = event;
}

/*----------------------------------------*/
volatile static bool gs_range_light_key_pressed = false;
volatile static bool gs_range_light_key_handled = true;
static void range_light_key_switched(uint32_t id, uint32_t event)
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

    gs_key_dbg_flag = true;
    gs_irq_id = id; gs_irq_event = event;
}

/*----------------------------------------*/
volatile static bool gs_charger_on_st = false, gs_last_charger_on_st = false;
static void charger_plugged(uint32_t id, uint32_t event)
{
    if(HIGH == digitalRead(CHARGER_ST))
    {//plugged
        gs_charger_on_st = true;

    }
    else
    {
        gs_charger_on_st = false;
    }

    gs_key_dbg_flag = true;
    gs_irq_id = id; gs_irq_event = event;
}

/*----------------------------------------*/
volatile static bool gs_charge_full_st = false, gs_last_charge_full_st = false;
static void charge_full_irq(uint32_t id, uint32_t event)
{
    if(HIGH == digitalRead(CHARG_FULL_SC_PG))
    {//full
        gs_charge_full_st = true;
    }
    else
    {
        gs_charge_full_st = false;
    }

    gs_key_dbg_flag = true;
    gs_irq_id = id; gs_irq_event = event;
}

/*----------------------------------------*/
uint16_t get_chg_st_in_reg_form()
{
    uint16_t reg_val = 0;

    if(gs_charger_on_st) reg_val |= MB_REG_DEV_INFO_BITS_CHG_CONN;
    if(gs_charge_full_st) reg_val |= MB_REG_DEV_INFO_BITS_BAT_FULL;

    return reg_val;
}

void process_hardware_key()
{
    bool rtu_oped = false;

    if(gs_key_dbg_flag)
    {
        DBG_PRINT(LOG_ERROR, F("irq id: ")); DBG_PRINTLN(LOG_ERROR, gs_irq_id);
        DBG_PRINT(LOG_ERROR, F("irq event: ")); DBG_PRINTLN(LOG_ERROR, gs_irq_event);

        gs_key_dbg_flag = false;
    }

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

            rtu_oped = true;
        }
    }

    if(gs_range_light_key_pressed)
    {
        if(!gs_range_light_key_handled)
        {
            static bool turn_on = true;
            switch_range_light(turn_on); turn_on = !turn_on;
            gs_range_light_key_handled = true;

            rtu_oped = true;
        }
    }

    bool rpt_chg = ((gs_charger_on_st != gs_last_charger_on_st) || (gs_charge_full_st != gs_last_charge_full_st));
    if(rpt_chg)
    {
        //there is somewhat difficulty for screen to process json msg that does not contain all regs. 
        //so we report all regs here, instead of report only chg st.
        rpt_mb_reg_json(); //rpt_dev_info_bits_json();

        set_just_rpt_mb_reg_flag(true);
    }

    gs_last_charger_on_st = gs_charger_on_st; gs_last_charge_full_st = gs_charge_full_st;

    set_mb_rtu_operated_flag(rtu_oped);
}

void gpio_pin_init()
{
    pinMode(EXPO_KEY, INPUT_IRQ_CHANGE);  //
    digitalSetIrqHandler(EXPO_KEY, expo_key_switched);

    pinMode(RANGE_LIGH_KEY, INPUT_IRQ_CHANGE);  //
    digitalSetIrqHandler(RANGE_LIGH_KEY, range_light_key_switched);

    pinMode(CHARGER_ST, INPUT_IRQ_CHANGE);  //
    digitalSetIrqHandler(CHARGER_ST, charger_plugged);

    pinMode(CHARG_FULL_SC_PG, INPUT_IRQ_CHANGE);  //
    digitalSetIrqHandler(CHARG_FULL_SC_PG, charge_full_irq);

    pinMode(XSHUTDOWN, OUTPUT); 
    digitalWrite(XSHUTDOWN, HIGH);

    pinMode(BLC, OUTPUT);
    digitalWrite(BLC, LOW);
}

void set_shutdown_pin(uint32_t val)
{
    digitalWrite(XSHUTDOWN, val);
}
