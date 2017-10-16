#ifndef __CHARGE_H
#define __CHARGE_H

#include "button.h"
#include "led.h"
#include "usart.h"
#include "comm.h"
//#define  PRO_LINE_TEST
#ifdef PRO_LINE_TEST
#include "product_test.h"
#endif

//#define FAST_I2C_MODE

#ifdef FAST_I2C_MODE
#define I2C_SPEED 400000    //300000
#else
#define I2C_SPEED 40000  //100000
#endif
#define CHARGING_FULL_CURRENT      100   // CASE charging current less than 100ms that is full.
#define LOW_BATTERY_PROTECT_VOL          20     // battery less than 3.5V will notify A12 shutdown
#define CASE_CHARING_THRESHOLD           (LOW_BATTERY_PROTECT_VOL+ 20)  //25
//#define WIRLESS_S1S2_DELAY_TIMEOUT  63
#define OTG_ON_DELAY_TIMEOUT    94
#define OTG_OFF_DELAY_TIMEOUT   32 //64 //10  // 5  53 times fail   // 3  delay 48ms test 45 times failed charging PHONE.
#define ENTER_SLEEP_TIMEOUT         250
#define SWITCH_STATE_TIMEOUT        0  //20 //62    // switch state after 1s when detect state change

#define FAST_CHARGE_ADC_VALUE           2730  //3400          // >2.0V  judge it that is fast charge  adapter
#define SLOW_CHARGE_ADC_VALUE           2460         // <1.8V  judge it that is slow charge adapter or PC usb charge  

#define NORMAL_ADAPTER_ADC_MIN_VALUE      82         // >0.06V
#define NORMAL_ADAPTER_ADC_MAX_VALUE      136        // <0.1V

#define PC_USB_ADC_MIN_VALUE               27          //>0.02V
#define PC_USB_ADC_MAX_VALUE               68         //<0.05V

#define PHONE_CHARGE_FULL_ADC_MAX_VALUE    55 //0.04V //41 // 0.03 //34 // 0.025 //27         //<0.02V


#define BAT_SCK                     GPIOC
#define BAT_SCK_PINS                GPIO_Pin_1

#define BAT_SDA                     GPIOC
#define BAT_SDA_PINS                GPIO_Pin_0

#define LED1_PORT                   GPIOD      //GPIOD      //GPIOB    //
#define LED1_PINS                   GPIO_Pin_5 // old GPIO_Pin_7 //GPIO_Pin_1

#define LED2_PORT                   GPIOD     //
#define LED2_PINS                   GPIO_Pin_7

#define LED3_PORT                   GPIOD   //
#define LED3_PINS                   GPIO_Pin_6

#define LED4_PORT                   GPIOC  //
#define LED4_PINS                   GPIO_Pin_5

#define S1_PORT                     GPIOD  //GPIOC 
#define S1_PINS                     GPIO_Pin_2 //GPIO_Pin_1

#define S2_PORT                     GPIOA  //
#define S2_PINS                     GPIO_Pin_4   //GPIO_Pin_4 old board

#define S3_PORT                     GPIOC   //
#define S3_PINS                     GPIO_Pin_4

#define S4_PORT                     GPIOC        //GPIOA              
#define S4_PINS                     GPIO_Pin_6  //GPIO_Pin_5

#define S5_PORT                     GPIOB      //GPIOC
#define S5_PINS                     GPIO_Pin_5 //GPIO_Pin_0

#define S6_PORT                     GPIOB      //GPIOA
#define S6_PINS                     GPIO_Pin_6

#define S7_PORT                     GPIOC      
#define S7_PINS                     GPIO_Pin_3

#define S8_PORT                     GPIOC      
#define S8_PINS                     GPIO_Pin_2

#define S8_OTG_PORT                 GPIOD     //
#define S8_OTG__PINS                GPIO_Pin_4

#define USB_DETECT_PORT             GPIOB     //
#define USB_DETECT_PINS             GPIO_Pin_2      // PB2 usb detect need to interrupt detect

#define PHONE_DETECT_PORT           GPIOA        //GPIOD
#define PHONE_DETECT_PINS           GPIO_Pin_6  //GPIO_Pin_0   //  PD0 phone detect need to interrupt detect

#define MCU_DSP_SING_PORT           GPIOB //GPIOA     //
#define MCU_DSP_SING_PINS           GPIO_Pin_0 //GPIO_Pin_4  //GPIO_Pin_6   // PC6  camera on detect need to interup detect 

//#define MCU_DSP_SING_PORT           GPIOA  //GPIOA     //
//#define MCU_DSP_SING_PINS           GPIO_Pin_4 //GPIO_Pin_4  //GPIO_Pin_6   // PC6  camera on detect need to interup detect 


#define WIRLESS_DETECT_PORT        GPIOB   // GPIOD      //GPIOB                // w_chg
#define WIRLESS_DETECT_PINS        GPIO_Pin_1 // GPIO_Pin_5 //GPIO_Pin_0  // PD0  wirelsse on detect need to interup detect 

#define  PRESS_KEY_PORT             GPIOB    //
#define  PRESS_KEY_PORT_PINS        GPIO_Pin_4

#define PHONE_CHARGE_DETECT_PORT    GPIOD    //
#define PHONE_CHARGE_DETECT_PINS    GPIO_Pin_3


#define WAKE_UP_PORT                GPIOA      //GPIOD
#define WAKE_UP_PINS                GPIO_Pin_5

#define KEY_PORT                    GPIOB    //
#define KEY_PINS                    GPIO_Pin_3

#define TEMP_CHARGE_PORT            GPIOD       //GPIOB
#define TEMP_CHARGE_PINS            GPIO_Pin_1 //GPIO_Pin_5

//#define USART_EN_PORT               GPIOD
//#define USART_EN_PORT_PINS          GPIO_Pin_0

#define USART_TX_PORT               GPIOA      //
#define USART_TX_PINS               GPIO_Pin_2

#define USART_RX_PORT               GPIOA     //
#define USART_RX_PINS               GPIO_Pin_3


// ADC port define
//#define BATTERY_ADC_PORT            GPIOB
//#define BATTERY_ADC_PINS            GPIO_Pin_6 

#define USB_V_DETECT_ADC_PORT       GPIOD                 //      ADC1 AIN2
#define USB_V_DETECT_ADC_PINS       GPIO_Pin_0  //GPIO_Pin_2
 
#define Current_detect_ADC_PORT     GPIOB         //GPIOA      //      ADC1 AIN11
#define Current_detect_ADC_PINS     GPIO_Pin_7  //GPIO_Pin_6

//#define MCU_NTC_ADC_PORT            GPIOB     //   ADC1 AIN18          
//#define MCU_NTC_ADC_PINS            GPIO_Pin_0
//end


/*
typedef struct _CHARGE_INFO_OBJ_T{
    unsigned char version_len;
    unsigned char version[32];
    unsigned char sn_len;
    unsigned char sn[32];
}CHARGE_INFO_OBJ;
*/
typedef enum {             
    CHARGE_STATE_IDLE,   //0        
    CHARGE_STATE_DEFAULT,// 1   
    CHARGE_STATE_A_E,      // 2     //phone: 0 ,camera: 0, usb in: 0, Wirelsess Base :1    
   // CHARGE_STATE_E,      // 3     //phone: 0 ,camera: 1, usb in: 0, Wirelsess Base :1   
    CHARGE_STATE_D,      // 4     //phone: 0 ,camera: 1, usb in: 0, Wirelsess Base :0
    CHARGE_STATE_H,      // 5     //phone: 1 ,camera: 0, usb in: 0, Wirelsess Base :0
    CHARGE_STATE_I,      // 6     //phone: 1 ,camera: 0, usb in: 0, Wirelsess Base :1
    CHARGE_STATE_L,      // 7     //phone: 1 ,camera: 1, usb in: 0, Wirelsess Base :0
    CHARGE_STATE_M,      // 8     //phone: 1 ,camera: 1, usb in: 0, Wirelsess Base :1
    CHARGE_STATE_N,      // 9     //phone: 1 ,camera: 1, usb in: 1, Wirelsess Base :0
   // CHARGE_STATE_O,      // 10     //phone: 1 ,camera: 1, usb in: 1, Wirelsess Base :1
    CHARGE_STATE_B_F,      // 11     //phone: 0 ,camera: 0, usb in: 1, Wirelsess Base :0
   // CHARGE_STATE_C,      // 12     //phone: 0 ,camera: 0, usb in: 1, Wirelsess Base :1
  //  CHARGE_STATE_F,      // 13     //phone: 0 ,camera: 1, usb in: 1, Wirelsess Base :0
   // CHARGE_STATE_G,      // 14     //phone: 0 ,camera: 1, usb in: 1, Wirelsess Base :1
    CHARGE_STATE_J1,     // 15     //phone: 1 ,camera: 0, usb in: 1, Wirelsess Base :X
    CHARGE_STATE_J2,     // 16     //phone: 1 ,camera: 0, usb in: 1, Wirelsess Base :X
  //  CHARGE_STATE_J3,     // 17      //phone: 1 ,camera: 0, usb in: 1, Wirelsess Base :X
    CHARGE_STATE_H_Y,    // 18     ////phone: 1 ,camera: 0, usb in: 0, Wirelsess Base :0 but short press key charging phone
    CHARGE_STATE_I_Y,    // 19      //phone: 1 ,camera: 0, usb in: 0, Wirelsess Base :1  // long press key switch stop charging phone or no
    CHARGE_STATE_J1_Y,   // 20       //phone: 1 ,camera: 0, usb in: 1, Wirelsess Base :X// long press key switch charging phone or CASE
    CHARGE_STATE_J2_Y,   // 21       //phone: 1 ,camera: 0, usb in: 1, Wirelsess Base :X // long press key switch charging phone or CASE
}ChargeCtlState_e;

/*
typedef enum{
 CHARGE_PHONE,
 CHARGE_CASE,
 CHARGE_PHONE_CASE,
}ChargeTarget_e;
*/
typedef enum
{
 OTG_IDLE,
 OTG_DELAY_START,
 OTG_DELAY_END
}OTGDelay_State_e;

typedef struct{
    bool force_check_state_flag;
    bool state_switch_flag;                  // 1: indicate control state happen changed.
    bool camera_on_off_flag;                 // 1: camera power on   0: camera shut down
    bool camera_uart_on_flag;                // 1: camera send power on cmd  0: 
    uint8_t  KEY_timeout;                   // control key port set high 1 S timeout for H state.
    uint8_t  H_state_key_cnt;               // to check if stop or continue charging phonewhen battery level <25% for H state.
    uint8_t  I2C_timeout;
    uint8_t  check_fastcharge_time;  
    uint8_t  check_switch_state_time;
   // uint8_t  check_Wirless_S1S2_Delay_time;
    uint8_t   IO_First_State;
    uint8_t   IO_Cur_State;
    uint8_t   IO_Prev_State;
    uint16_t usb_det_adc;
    uint16_t current_det_adc;
    uint16_t check_phone_fullcharge_time;
    uint16_t check_OTG_Delay_TimeOut;
    uint16_t check_BatTemp_TimeOut;
  //  ChargeTarget_e    ChargeTarget;
    ChargeCtlState_e  ChargeCtlState;
    ChargeCtlState_e  Prev_ChargeCtlState;
    
    
    LED_MGR_OBJ       led_obj;
    //bat 
    BATTERY_MGR_OBJ   bat_obj;
     //usart
    USART_OBJ         usart_obj;
     //ver info
   // CHARGE_INFO_OBJ   charge_info_obj;
}Charge_Obj_Data_t;

extern Charge_Obj_Data_t  Charge_Obj_Data;
extern const  char  MCU_version[];
extern unsigned char lowbattery_cnt;
extern OTGDelay_State_e OTG_delay_State;

void Charge_Init_Clock(void);
void Charge_Init_ADC(void);
void Charge_Init_Timer(void);
void Charge_Init_GPIO(void);
void Charge_Init_Interrupt(void);
void Check_ChargeControl_State(void);
void Charge_State_Contrl_Handler(void);
void Charge_Init_User(void);
void Charge_Init_iWatchdog(void);
void Charge_Init_USART(void);
void Charge_Init_Flash(void);   
void Charge_Init_Led(void);
void Delay(__IO uint16_t nCount);
//void Charge_StateA_ControlHandler(void);
void Charge_StateAE_ControlHandler(void);
void Charge_StateE_ControlHandler(void);
void Charge_StateD_ControlHandler(void);
void Charge_StateH_Y_ControlHandler(void);
void Charge_StateH_ControlHandler(void);
void Charge_StateB_F_ControlHandler(void);
//void Charge_StateF_ControlHandler(void);
//BATTERY_LEVEL Charge_Set_Bat_Threshold(unsigned char val);
void Charge_StateJ1_ControlHandler(void);
void Charge_StateJ1_BatteryChargeControlHandler(void);
void Charge_StateJ2_ControlHandler(void);
void Charge_StateJ3_ControlHandler(void);
void Set_Start_Charging_indicator(void);
void Check_Battery_Discharge_Level(void);
void Charge_StateI_ControlHandler(void);
void Charge_StateN_ControlHandler(void);
void Charge_StateM_ControlHandler(void);
void Charge_StateL_ControlHandler(void);
 void Judge_J_State(void);
 void Product_Test_Led_Handler(void);
 //void Charge_Default_ControlHandler(void);
#endif

