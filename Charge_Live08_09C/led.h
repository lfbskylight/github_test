
#ifndef __LED_H
#define __LED_H
#include "battery.h"

#define LED_ROLL_INTERVAL  30 
#define LED_FLASH_INTERVAL  62  // 1S flash one time //25    //define led flash frequency  that is 25*15 = 400ms flash once time.
#define BATTERY_LEVEL_3S_TIMEOUT       188

#define BATTERY_CHARGE_LED_TIME       3750  //18750   // Time 1Min check charging led 
#define CHECK_PHONE_FULL_CHARGE_TIME  3750    //time 1Min check if phone full charge  3750*16 = 60000
#define CHECK_BATTERY_DISCHARGE_TIME  3750   // time 1Min check if  battery level low than 10% to notify DSP shut down
#define KEY_SET_TIMEOUT             62   // keep  key hign  1S for H state.
#define CHECK_FASTCHARGE_TIME       62   // check if switch fast charge with 1S interval time

#define CHECK_BATTEMP_TIMEOUT      18750  // time 5Min check battery temperature
typedef enum _LED_MGR_STA{
    //LED_MGR_INVALID = -1,
    //LED1_FLASH,
    //LED2_FLASH,
    //LED3_FLASH,
    LED4_FLASH,
    LED4_ON_LED3_FLASH,
    LED4_LED3_ON_LED2_FLASH,
    LED4_LED3_LED2_ON_LED1_FLASH,
  //  LED1_ON_LED2_FLASH,
  //  LED1_LED2_ON_LED3_FLASH,
  //  LED1_LED2_LED3_ON_LED4_FLASH,
  //  LED1_ON,
 //   LED2_ON,
 //   LED3_ON,
 //   LED4_ON,
 //   LED1_LED2_ON,
 //   LED1_LED2_LED3_ON,
    LED_ALL_ON,
  //  LED_ALL_OFF,
  //  LED_NOT_ALL_OFF,
}LED_MGR_STA;

typedef enum{
 STATE_IDLE = 0,
 BAT_CHARGING_INDICATE,
 BAT_CHARGDONE_INDICATE,
// LOW_BAT_INDICATE,
 BAT_LEVEL_INDICATE,
 BAT_ROLL_INDICATE
}LED_indication_t;
#if 0
typedef enum _CHARGE_LED_FLASH_WAY{
    LED_FLASH_INVALID = -1,
    USB_DETECT_CHARGE,
    PRESS_KEY,
}LED_FLASH_WAY;

typedef enum _LED_ACTION_MGR{
    LED_IDEL,
    LED_SEARCH_POWER,
    LED_3S_TURN_OFF_ALL,
    //LED_BAT_FULL_3S_TURN_OFF_ALL,
    LED_3S_FLASH_CHARGING,
    LED_BAT_FULL_TURN_OFF_ALL,
    LED_VR_INSERT_LED1_ON,
    LED_VR_INSERT_LED2_ON,
    LED_VR_INSERT_LED3_ON,
    LED_VR_INSERT_LED4_ON,
    LED_VR_INSERT_LED1_LED2_LED3_ON,
    LED_VR_INSERT_LED1_LED2_ON_LED3_READY,
    LED_VR_INSERT_LED1_LED2_LED3_ON_LED4_READY,
    LED_INVILID = -1
}LED_ACTION_MGR;
#endif
typedef struct _LED_MGR_OBJ_T{
    unsigned char  led_3s_off_time;
    unsigned char  led_flash_time; 
    LED_indication_t led_indicate_state;
    LED_MGR_STA  led_flash_state;
    unsigned char  led_roll_state;
}LED_MGR_OBJ;

void Battery_Level_Led_Indicator(BATTERY_LEVEL level);
void Set_Led_Indicator_State(LED_indication_t state);
void Set_Led_Flash_State(BATTERY_LEVEL level);
void Check_Led_State (void);
void Led_ALL_OFF(void);
void Led_ALL_ON(void);
void Check_Charge_Led_Flash_State(void);
#endif
