#include "stm8l15x.h"
#include "charge.h"
#include "battery.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
//#include "led.h"
#include "time.h"
//#include "button.h"
#include "Keyproc.h"
//NDEBUG
const char  MCU_version[] ="Apollo(0005)-mcu_v3.0.26";
Charge_Obj_Data_t  Charge_Obj_Data;
OTGDelay_State_e OTG_delay_State;

unsigned char lowbattery_cnt = 0;
bool led_test_flash_flag = FALSE;
//static void Judge_J_State(void);

extern void Enter_halt_mode_init(void);
extern void Exit_halt_mode_reinit(void);
static void Check_Battery_Temp(void);

void Delay(__IO uint16_t nCount)
{
    /* Decrement nCount value */
    while (nCount > 1)
    {
        nCount--;
    }
}

/**
  * @brief  init clock .
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_Init_Clock(void)
{
    CLK_HSICmd(ENABLE); 
    CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI); 
    while (CLK_GetSYSCLKSource() != CLK_SYSCLKSource_HSI)
    {}
   //  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1); 
    CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_8);   // 2Mhz
   // CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_2); 
   CLK_HaltConfig(CLK_Halt_FastWakeup,ENABLE);  // fbli add to fast wake up from Halt mode

   CLK_PeripheralClockConfig(CLK_Peripheral_BOOTROM, DISABLE); //fbli add to disable bootrom clock to save power

   PWR_UltraLowPowerCmd(ENABLE);   // disable internal voltage reference to save power when enter HALT mode
   PWR_FastWakeUpCmd(ENABLE);
}

/**
  * @brief  init all I/O  control.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_Init_GPIO(void)
{
    //Charge_DeInit_GPIO();
    //I2C
    GPIO_Init(BAT_SCK,BAT_SCK_PINS,GPIO_Mode_Out_PP_High_Fast);  //GPIO_Mode_Out_PP_High_Slow//GPIO_Mode_Out_PP_High_Fast
    GPIO_Init(BAT_SDA,BAT_SDA_PINS,GPIO_Mode_Out_PP_High_Fast);  //GPIO_Mode_Out_PP_High_Slow
    //end
   // led io init
    GPIO_Init(LED1_PORT,LED1_PINS,GPIO_Mode_Out_PP_Low_Slow); //GPIO_Mode_Out_PP_Low_Slow
    GPIO_Init(LED2_PORT,LED2_PINS,GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(LED3_PORT,LED3_PINS,GPIO_Mode_Out_PP_Low_Slow); //GPIO_Mode_Out_PP_High_Slow
    GPIO_Init(LED4_PORT,LED4_PINS,GPIO_Mode_Out_PP_Low_Slow);
  // init S1~S6, S8_OTG  output  io
    GPIO_Init(S1_PORT,S1_PINS,GPIO_Mode_Out_PP_Low_Slow); 
    GPIO_Init(GPIOA,GPIO_Pin_0,GPIO_Mode_In_FL_No_IT); 
    GPIO_Init(S2_PORT,S2_PINS,GPIO_Mode_Out_PP_Low_Slow);   // not init it or cause dowload failed when only  two wire SWIM and Gnd
    GPIO_Init(S3_PORT,S3_PINS,GPIO_Mode_Out_PP_Low_Slow); 
    GPIO_Init(S4_PORT,S4_PINS,GPIO_Mode_Out_PP_High_Slow); 
    GPIO_Init(S5_PORT,S5_PINS,GPIO_Mode_Out_PP_Low_Slow); 
    GPIO_Init(S6_PORT,S6_PINS,GPIO_Mode_Out_PP_Low_Slow); 
    GPIO_Init(S7_PORT,S7_PINS,GPIO_Mode_Out_PP_Low_Slow); 
    GPIO_Init(S8_PORT,S8_PINS,GPIO_Mode_Out_PP_High_Slow); 
    GPIO_Init(S8_OTG_PORT,S8_OTG__PINS,GPIO_Mode_Out_PP_High_Slow); 

    GPIO_Init(KEY_PORT,KEY_PINS,GPIO_Mode_Out_PP_Low_Slow);

    GPIO_Init(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS,GPIO_Mode_Out_PP_Low_Slow);
   
    //GPIO_Init(USART_EN_PORT,USART_EN_PORT_PINS,GPIO_Mode_Out_PP_Low_Slow);
    
 // init input IO
   //use plug  detect 
 // GPIO_Init(USB_DETECT_PORT, USB_DETECT_PINS, GPIO_Mode_In_FL_No_IT);
  //  GPIO_Init(USB_DETECT_PORT, USB_DETECT_PINS, GPIO_Mode_In_PU_IT);  // OK
  GPIO_Init(USB_DETECT_PORT, USB_DETECT_PINS, GPIO_Mode_In_FL_IT);
   
   // phone plug detect
  // GPIO_Init(PHONE_DETECT_PORT, PHONE_DETECT_PINS, GPIO_Mode_In_FL_No_IT);
  // GPIO_Init(PHONE_DETECT_PORT, PHONE_DETECT_PINS, GPIO_Mode_In_PU_IT); // OK
   GPIO_Init(PHONE_DETECT_PORT, PHONE_DETECT_PINS, GPIO_Mode_In_FL_IT);

  // press key port
   GPIO_Init(PRESS_KEY_PORT, PRESS_KEY_PORT_PINS, GPIO_Mode_In_FL_No_IT);
   //GPIO_Init(PRESS_KEY_PORT, PRESS_KEY_PORT_PINS, GPIO_Mode_In_PU_IT);

   //MCU_DSP_SING
   // GPIO_Init(MCU_DSP_SING_PORT, MCU_DSP_SING_PINS, GPIO_Mode_In_FL_No_IT); //GPIO_Mode_In_FL_IT
   GPIO_Init(MCU_DSP_SING_PORT, MCU_DSP_SING_PINS, GPIO_Mode_In_FL_IT);
   // GPIO_Init(MCU_DSP_SING_PORT, MCU_DSP_SING_PINS, GPIO_Mode_In_PU_IT);
  // GPIO_Init(GPIOA, GPIO_Pin_4, GPIO_Mode_In_FL_No_IT);

   // wireless charge detect
  // GPIO_Init(WIRLESS_DETECT_PORT, WIRLESS_DETECT_PINS, GPIO_Mode_In_FL_No_IT);
 //   GPIO_Init(WIRLESS_DETECT_PORT, WIRLESS_DETECT_PINS, GPIO_Mode_In_PU_No_IT); // 
 //   GPIO_Init(WIRLESS_DETECT_PORT, WIRLESS_DETECT_PINS, GPIO_Mode_In_PU_IT); // OK 
   GPIO_Init(WIRLESS_DETECT_PORT, WIRLESS_DETECT_PINS, GPIO_Mode_In_FL_IT); // OK 
   
   // Wake up
    GPIO_Init(WAKE_UP_PORT, WAKE_UP_PINS, GPIO_Mode_In_FL_No_IT);  //GPIO_Mode_In_PU_No_IT
   //GPIO_Init(WAKE_UP_PORT, WAKE_UP_PINS, GPIO_Mode_In_PU_IT); // OK 
   //GPIO_Init(WAKE_UP_PORT, WAKE_UP_PINS, GPIO_Mode_In_FL_IT);

   //PHONE_CHARGE_DETECT
   GPIO_Init(PHONE_CHARGE_DETECT_PORT, PHONE_CHARGE_DETECT_PINS, GPIO_Mode_In_FL_No_IT);

   // GPIO_Init(ACCESSORY_DETECT_PORT, ACCESSORY_DETECT_PINS, GPIO_Mode_In_FL_IT); 
    
 // init  ADC port
   //BATTERY_ADC
    //GPIO_Init(BATTERY_ADC_PORT, BATTERY_ADC_PINS, GPIO_Mode_In_FL_No_IT);   //use ADC channel 12

   // USB v detect ADC 
    GPIO_Init(USB_V_DETECT_ADC_PORT, USB_V_DETECT_ADC_PINS, GPIO_Mode_In_FL_No_IT); // use ADC channel 2

   // phone charge  Current detect ADC 
    GPIO_Init(Current_detect_ADC_PORT, Current_detect_ADC_PINS, GPIO_Mode_In_FL_No_IT);  // use ADC channel 11

   //  charge  temperature detect ADC 
    //GPIO_Init(MCU_NTC_ADC_PORT, MCU_NTC_ADC_PINS, GPIO_Mode_In_FL_No_IT);   // use ADC channel 18

   #if 1
   //USART_TX
 //   GPIO_Init(USART_TX_PORT, USART_TX_PINS, GPIO_Mode_Out_PP_Low_Slow);   // not init TX and RX with UP, it will cause reset if not connnect uart

    //USART_RX
   // GPIO_Init(USART_RX_PORT, USART_RX_PINS, GPIO_Mode_In_PU_No_IT);
   #else  //that is ok
     /* Configure USART Tx as alternate function push-pull  (software pull up)*/
    GPIO_ExternalPullUpConfig(USART_TX_PORT, USART_TX_PINS, ENABLE);

    /* Configure USART Rx as alternate function push-pull  (software pull up)*/
    GPIO_ExternalPullUpConfig(USART_RX_PORT, USART_RX_PINS, ENABLE);
   #endif
}

/**
  * @brief  init flashl.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_Init_Flash(void)
{
    FLASH_DeInit();
    FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard);   //FLASH_ProgramMode_Standard
    
}
/**
  * @brief  init external interrupt control.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_Init_Interrupt(void)
{
    EXTI_DeInit ();  
   #if 1
   EXTI_SetPinSensitivity(EXTI_Pin_1,EXTI_Trigger_Rising_Falling);  // wireless interrrupt
   EXTI_SetPinSensitivity(EXTI_Pin_2,EXTI_Trigger_Rising_Falling); //usb detect interrupt  PB2
   EXTI_SetPinSensitivity(EXTI_Pin_0,EXTI_Trigger_Rising_Falling);        // Camera on interrupt
   EXTI_SetPinSensitivity(EXTI_Pin_4,EXTI_Trigger_Falling);        // press key detect interrupt  
   EXTI_SetPinSensitivity(EXTI_Pin_5,EXTI_Trigger_Rising_Falling);  // wake up  interrupt  PA5
   EXTI_SetPinSensitivity(EXTI_Pin_6,EXTI_Trigger_Rising_Falling); //phone detect interrupt  PA6

   EXTI_SetPinSensitivity(EXTI_Pin_3,EXTI_Trigger_Rising);   // uart rx as gpio  interrrupt
   #else
    EXTI_SetPinSensitivity(EXTI_Pin_0,EXTI_Trigger_Rising);   
    EXTI_SetPinSensitivity(EXTI_Pin_5,EXTI_Trigger_Falling); 

   #endif
}


/**
  * @brief  init usart.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_Init_USART(void)
{
    USART_DeInit(USART1);
    #if 1  //ndef SKY_DEBUG
    SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortA,ENABLE);
    CLK_PeripheralClockConfig (CLK_Peripheral_USART1,ENABLE);
    USART_Init(USART1,9600,USART_WordLength_8b,USART_StopBits_1,USART_Parity_No,(USART_Mode_Tx|USART_Mode_Rx));//设置USART参数9600，8N1，接收/发送
    USART_ITConfig (USART1,USART_IT_RXNE,ENABLE); 
    USART_Cmd (USART1,ENABLE); 
    #else
   // SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortA,ENABLE);
    CLK_PeripheralClockConfig (CLK_Peripheral_USART1,ENABLE);
    USART_Init(USART1,9600,USART_WordLength_8b,USART_StopBits_1,USART_Parity_No,USART_Mode_Tx|USART_Mode_Rx);//设置USART参数9600，8N1，接收/发送
    USART_ITConfig (USART1,USART_IT_RXNE,ENABLE); 
    USART_Cmd (USART1,ENABLE); 
    #endif
}

/**
  * @brief  first display led when power up.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_Init_Led(void)
{
   // __IO uint8_t i = 0;
     uint8_t i = 0;

    while(1)
    {
        if(( i== 0)&& (17 == Charge_Obj_Data.led_obj.led_flash_time))
        {
           GPIO_WriteBit(S8_OTG_PORT, S8_OTG__PINS, RESET);
        }
         
        if (30 == Charge_Obj_Data.led_obj.led_flash_time)
        {
            Charge_Obj_Data.led_obj.led_flash_time = 0;
            i++;
            GPIO_ToggleBits(LED1_PORT,LED1_PINS);
            GPIO_ToggleBits(LED2_PORT,LED2_PINS);
            GPIO_ToggleBits(LED3_PORT,LED3_PINS);
            GPIO_ToggleBits(LED4_PORT,LED4_PINS);
            if (i>5)
            {
                
                break;
            }
        }
         #ifdef WDOG_EN
      //IWDG_ReloadCounter();
       WWDG_SetCounter(0x7f);
       #endif
    }
}

/**
  * @brief  init ADC .
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_Init_ADC(void)
{
    
    CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, ENABLE); 
    Delay(4000);
    ADC_DeInit(ADC1); 
    #if 0
    ADC_SchmittTriggerConfig(ADC1, ADC_Channel_2, DISABLE);   //  USB_V_detect adc channel
    //ADC_SchmittTriggerConfig(ADC1, ADC_Channel_12, DISABLE);   // Battery ADC channel
    ADC_SchmittTriggerConfig(ADC1, ADC_Channel_18, DISABLE);   //  MCU_NTC temperature adc channel
    ADC_SchmittTriggerConfig(ADC1, ADC_Channel_11, DISABLE);    // current  ADC channel
    #endif
    ADC_Init(ADC1, ADC_ConversionMode_Single,ADC_Resolution_12Bit, ADC_Prescaler_2); 
    //ADC_SamplingTimeConfig(ADC1, ADC_Group_SlowChannels,ADC_SamplingTime_4Cycles ); //is ok
    ADC_SamplingTimeConfig(ADC1, ADC_Group_SlowChannels, ADC_SamplingTime_384Cycles ); 
    ADC_Cmd(ADC1, ENABLE); 
    Delay(1000);
  /*
    ADC_ChannelCmd(ADC1, ADC_Channel_0, ENABLE);
    ADC_ChannelCmd(ADC1, ADC_Channel_11, ENABLE);
    ADC_ChannelCmd(ADC1, ADC_Channel_12, ENABLE);
    ADC_ChannelCmd(ADC1, ADC_Channel_20, ENABLE);
    */
    ADC_DMACmd(ADC1,DISABLE);  //fbli add that is must disable DMA for single mode
    Delay(1000);
}


/**
  * @brief  check charge control state if change or not .
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */

void Check_ChargeControl_State(void)
{
    __IO uint8_t usb_detect_status;             //0: usb plug in                 1: usb plug out
    __IO uint8_t phone_detect_status;           // 0: phone exist               1: phone no exist
    __IO uint8_t camera_detect_status;          // 0: camera off                1: camera on       
    __IO uint8_t wireless_detect_status;        // 0: wireless off              1: wireless on

    
  //if(Charge_Obj_Data.state_switch_flag == TRUE)
  if((Charge_Obj_Data.state_switch_flag == TRUE)
      &&(Charge_Obj_Data.check_switch_state_time == 0)
     )
   {
        Charge_Obj_Data.state_switch_flag       = FALSE;
        Charge_Obj_Data.check_switch_state_time  = ENTER_SLEEP_TIMEOUT;   // delay 4 S to enter sleep when switch state
       
        Charge_Obj_Data.IO_Cur_State  = ((GPIO_ReadInputDataBit(PHONE_DETECT_PORT,PHONE_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(USB_DETECT_PORT,USB_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(WIRLESS_DETECT_PORT,WIRLESS_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(MCU_DSP_SING_PORT, MCU_DSP_SING_PINS))
                                          );

        if( (( Charge_Obj_Data.IO_Cur_State == Charge_Obj_Data.IO_First_State)
              &&(Charge_Obj_Data.IO_Cur_State != Charge_Obj_Data.IO_Prev_State)
             )
          ||(Charge_Obj_Data.force_check_state_flag == TRUE)
          )
          {

                #if 1
            
                switch ( Charge_Obj_Data.IO_Cur_State)
                {
                   case 0x46:
                   case 0x47:
                    // State A   phone: 0 ,camera: 0, usb in: 0, Wirelsess Base :1    
                     //State E  phone: 0 ,camera: 1, usb in: 0, Wirelsess Base :1   
                    Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_A_E;
                           Charge_Obj_Data.bat_obj.bat_det_time = 0;
                           Set_Start_Charging_indicator();
                           Charge_StateAE_ControlHandler();
                           Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                          #ifdef SKY_DEBUG  
                              printf("A_E\r\n");
                          #endif
                    break;
                   case 0x45:
                     //State D  phone: 0 ,camera: 1, usb in: 0, Wirelsess Base :0 
                        if(Charge_Obj_Data.camera_on_off_flag == TRUE)
                            {
                                Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_D;
                                Get_Battery_Percent();
                                lowbattery_cnt = 0;
                               // Charge_Obj_Data.bat_obj.bat_discharge_time  = 0;
                                Charge_Obj_Data.bat_obj.bat_discharge_time = (CHECK_BATTERY_DISCHARGE_TIME-188);  // delay 3S 
                               // Charge_StateD_ControlHandler();
                                if(Charge_Obj_Data.camera_uart_on_flag == TRUE)   // CAM on led dicator
                                {
                                    Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                    //COR_AP_ShorKey_Common_Handler();
                                    Charge_Obj_Data.bat_obj.bat_level =  Battery_Get_Vol_Level(Charge_Obj_Data.bat_obj.bat_remain_percent);
                                    if(Charge_Obj_Data.led_obj.led_indicate_state  != BAT_ROLL_INDICATE)
                                        Set_Led_Indicator_State(BAT_ROLL_INDICATE);
                                 }
                                else
                                {
                                    Set_Led_Indicator_State(STATE_IDLE);
                                }
                                #ifdef SKY_DEBUG  
                                    printf("D\r\n");
                                #endif
                            }
                          else
                            {
                               Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_DEFAULT;
                               Set_Led_Indicator_State(STATE_IDLE);
                                #ifdef SKY_DEBUG  
                                    printf("X1\r\n");
                                #endif
                            }
                    break;
                   case 0x04:
                     //State H  phone: 1 ,camera: 0, usb in: 0, Wirelsess Base :0
                         if( Charge_Obj_Data.ChargeCtlState != CHARGE_STATE_H_Y)
                          {
                              Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_H;
                              Charge_StateH_ControlHandler();
                              Charge_Obj_Data.H_state_key_cnt = 0;
                              Set_Led_Indicator_State(STATE_IDLE);
                              #ifdef SKY_DEBUG  
                                  printf("H\r\n");
                              #endif
                          }
                    break;
                  case 0x06:
                     //State I  phone: 1 ,camera: 0, usb in: 0, Wirelsess Base :1
                         if( Charge_Obj_Data.ChargeCtlState != CHARGE_STATE_I)
                         {
                              Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_I;
                              Charge_Obj_Data.bat_obj.bat_det_time = 0;
                              Charge_Obj_Data.check_phone_fullcharge_time = 0;
                              OTG_delay_State = OTG_DELAY_START;
                              GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);  //fbli temp mask
                              Charge_Obj_Data.check_OTG_Delay_TimeOut = OTG_OFF_DELAY_TIMEOUT;
                             // Charge_Obj_Data.check_Wirless_S1S2_Delay_time = WIRLESS_S1S2_DELAY_TIMEOUT;
                             // Charge_StateI_ControlHandler();
                              Set_Start_Charging_indicator();
                            // Set_Led_Indicator_State(STATE_IDLE); //fbli temp add
                         }
                          #ifdef SKY_DEBUG  
                              printf("I\r\n");
                          #endif
                   break;

                 case 0x05:
                       //State L  phone: 1 ,camera: 1, usb in: 0, Wirelsess Base :0
                          if(Charge_Obj_Data.camera_on_off_flag == TRUE)
                            {
                                Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_L;
                                Get_Battery_Percent();
                                 lowbattery_cnt = 0;
                               // Charge_Obj_Data.bat_obj.bat_discharge_time  = 0;
                                Charge_Obj_Data.bat_obj.bat_discharge_time = (CHECK_BATTERY_DISCHARGE_TIME-188);  // delay 3S 
                                if(Charge_Obj_Data.camera_uart_on_flag == TRUE)   // CAM on led dicator
                                {
                                    Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                   // COR_AP_ShorKey_Common_Handler();
                                    Charge_Obj_Data.bat_obj.bat_level =  Battery_Get_Vol_Level(Charge_Obj_Data.bat_obj.bat_remain_percent);
                                   if(Charge_Obj_Data.led_obj.led_indicate_state  != BAT_ROLL_INDICATE)
                                        Set_Led_Indicator_State(BAT_ROLL_INDICATE);
                                 }
                                else
                                {
                                    Set_Led_Indicator_State(STATE_IDLE);
                                }
                                 #ifdef SKY_DEBUG  
                                    printf("L\r\n");
                                 #endif
                            }
                          else
                            {
                                lowbattery_cnt = 0;
                               Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_H;
                               Charge_Obj_Data.H_state_key_cnt = 0;
                             //  Charge_Obj_Data.check_switch_state_time = 248;
                               Charge_StateH_ControlHandler();
                               Set_Led_Indicator_State(STATE_IDLE);
                                #ifdef SKY_DEBUG  
                                    printf("H 2\r\n");
                                #endif
                            }
                 break;
                 case 0x07:
                      //State M  phone: 1 ,camera: 1, usb in: 0, Wirelsess Base :1
                         if(Charge_Obj_Data.camera_on_off_flag == TRUE)
                         {
                               Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_M;
                               Charge_Obj_Data.bat_obj.bat_det_time = 0;
                               Charge_StateM_ControlHandler();
                               Charge_Obj_Data.check_OTG_Delay_TimeOut = OTG_ON_DELAY_TIMEOUT;
                               Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                             /*
                                                  if(Charge_Obj_Data.camera_uart_on_flag == TRUE)   // CAM on led dicator
                                                  {
                                                       Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                                       COR_AP_ShorKey_Common_Handler();
                                                   }
                                                   else
                                                   */
                               {
                                   Set_Start_Charging_indicator();
                               }
                          #ifdef SKY_DEBUG  
                              printf("M\r\n");
                          #endif
                         }
                          else
                          {
                                Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_I;
                                Charge_Obj_Data.bat_obj.bat_det_time = 0;
                                Charge_Obj_Data.check_phone_fullcharge_time = 0;
                                GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
                                Charge_Obj_Data.check_OTG_Delay_TimeOut = OTG_OFF_DELAY_TIMEOUT;
                                OTG_delay_State = OTG_DELAY_START;
                               // Charge_StateI_ControlHandler();
                                Set_Start_Charging_indicator();
                                 #ifdef SKY_DEBUG  
                                    printf("I\r\n");
                                #endif
                          }
                  break;
                 case 0x01:
                 case 0x03:
                    // State N   phone: 1 ,camera: 1, usb in: 1, Wirelsess Base :0
                    // State O   phone: 1 ,camera: 1, usb in: 1, Wirelsess Base :1
                          Charge_Obj_Data.bat_obj.bat_det_time = 0;
                          if(Charge_Obj_Data.camera_on_off_flag == TRUE)
                            {
                                Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_N;
                                Charge_StateN_ControlHandler();
                                #if 0 //def SKY_DEBUG  
                                GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, SET);
                                #endif
                                
                                Charge_Obj_Data.check_OTG_Delay_TimeOut = OTG_ON_DELAY_TIMEOUT;
                                Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                //COR_AP_ShorKey_Common_Handler();
                                /*
                                                           if(Charge_Obj_Data.camera_uart_on_flag == TRUE)   // CAM on led dicator
                                                            {
                                                                Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                                                COR_AP_ShorKey_Common_Handler();
                                                            }
                                                            else
                                                        */
                                {
                                    Set_Start_Charging_indicator();
                                }
                                #ifdef SKY_DEBUG  
                                    printf("N\r\n");
                                #endif
                            }
                          else
                            {
                                Charge_Obj_Data.usb_det_adc = USB_v_Detect_Get_Vol();
                          
                                if( Charge_Obj_Data.usb_det_adc <SLOW_CHARGE_ADC_VALUE)// slow charge adapter
                                {
                                    Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_J2;  //;CHARGE_STATE_J1
                                    Charge_Obj_Data.check_phone_fullcharge_time = 0;
                                    Charge_Obj_Data.check_fastcharge_time = 0;
                                    GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
                                    GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
                                    GPIO_WriteBit(S1_PORT, S1_PINS, RESET);
                                    GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
                                    Charge_Obj_Data.check_OTG_Delay_TimeOut = OTG_OFF_DELAY_TIMEOUT;
                                    OTG_delay_State  = OTG_DELAY_START;
                                    Set_Start_Charging_indicator();
                                    #ifdef SKY_DEBUG  
                                        printf("J2\r\n");
                                    #endif
                                 }
                                /*
                                                        else if(Charge_Obj_Data.usb_det_adc >FAST_CHARGE_ADC_VALUE) // fast charge adapter
                                                        {
                                                         Charge_Obj_Data.check_phone_fullcharge_time = 0;
                                                             Charge_Obj_Data.ChargeCtlState     = CHARGE_STATE_J1;
                                                             Charge_Obj_Data.ChargeTarget     = CHARGE_PHONE;
                                                             Charge_StateJ1_ControlHandler();
                                                             Set_Led_Indicator_State(STATE_IDLE);
                                                             #ifdef SKY_DEBUG  
                                                               printf("J1_1\r\n");
                                                             #endif
                                                           }
                                                         */
                            }
                          break;
                  case 0x40:   // B
                  case 0x41:   // F
                  case 0x42:   // C
                  case 0x43:   //G
                            #ifdef SKY_DEBUG  
                                printf("cur: %d \r\n",Charge_Obj_Data.IO_Cur_State);
                            #endif
                          Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_B_F;
                          Charge_Obj_Data.bat_obj.bat_det_time = 0;
                          Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                          Set_Start_Charging_indicator();
                           Charge_StateB_F_ControlHandler();
                          #ifdef SKY_DEBUG  
                              printf("B_F\r\n");
                          #endif
                    break;
                  case 0x00:
                  case 0x02:
                       //State J  phone: 1 ,camera: 0, usb in: 1, Wirelsess Base :X
                        #ifdef SKY_DEBUG  
                            printf("cur: %d \r\n",Charge_Obj_Data.IO_Cur_State);
                        #endif
                          Charge_Obj_Data.usb_det_adc       =       USB_v_Detect_Get_Vol();
                          
                         if( Charge_Obj_Data.usb_det_adc <SLOW_CHARGE_ADC_VALUE)// slow charge adapter
                         {
                               Charge_Obj_Data.bat_obj.bat_det_time = 0;
                               Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_J2;  //;CHARGE_STATE_J1
                               OTG_delay_State  = OTG_DELAY_START;
                               Charge_Obj_Data.check_phone_fullcharge_time = 0;
                               Charge_Obj_Data.check_fastcharge_time = 0;
                               GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
                               GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
                               GPIO_WriteBit(S1_PORT, S1_PINS, RESET);
                               GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
                               Charge_Obj_Data.check_OTG_Delay_TimeOut = OTG_OFF_DELAY_TIMEOUT;
                               Set_Start_Charging_indicator();
                               #ifdef SKY_DEBUG  
                                  printf("J2_2\r\n");
                               #endif
                          }
                         else if(Charge_Obj_Data.usb_det_adc >FAST_CHARGE_ADC_VALUE) // fast charge adapter
                          {
                                 Charge_Obj_Data.check_phone_fullcharge_time = 0;
                                 Charge_Obj_Data.ChargeCtlState     = CHARGE_STATE_J1;
                               //  Charge_Obj_Data.ChargeTarget     = CHARGE_PHONE;
                                 Charge_StateJ1_ControlHandler();
                                 Set_Led_Indicator_State(STATE_IDLE);
                                 #ifdef SKY_DEBUG  
                                   printf("J1_2\r\n");
                                 #endif
                           }
                    break;
                 default:
                    //default state
                         Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_DEFAULT;
                         Set_Led_Indicator_State(STATE_IDLE);
                         #ifdef SKY_DEBUG  
                              printf("X2\r\n");
                         #endif
                    break;
               }
            

                #else
                usb_detect_status       = GPIO_ReadInputDataBit(USB_DETECT_PORT,USB_DETECT_PINS);
                phone_detect_status     = GPIO_ReadInputDataBit(PHONE_DETECT_PORT,PHONE_DETECT_PINS);
                wireless_detect_status  = GPIO_ReadInputDataBit(WIRLESS_DETECT_PORT,WIRLESS_DETECT_PINS);
                camera_detect_status    = GPIO_ReadInputDataBit(MCU_DSP_SING_PORT, MCU_DSP_SING_PINS);
                 Charge_Obj_Data.IO_Prev_State = Charge_Obj_Data.IO_Cur_State;
                 Charge_Obj_Data.force_check_state_flag =FALSE;

              // camera_detect_status  = !RESET;
            //  Charge_Obj_Data.camera_on_off_flag = TRUE;
              #if 0  //def SKY_DEBUG  
               // usb_detect_status   = RESET;
                
                // phone_detect_status = RESET;
                 camera_detect_status = !RESET;
                // wireless_detect_status = !RESET;
               #endif
                
                if(usb_detect_status != RESET) // usb not plug or plug out
               // if(usb_detect_status == SET) // not use SET to judge I/O state
                {
                  #if 0
                   if((RESET != phone_detect_status)&&
                      (RESET == camera_detect_status)&&
                    //  (RESET == wireless_detect_status)
                      (RESET != wireless_detect_status)
                       )         // State A   phone: 0 ,camera: 0, usb in: 0, Wirelsess Base :1    
                    {
                           Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_A;
                           Charge_Obj_Data.bat_obj.bat_det_time = 0;
                           Set_Start_Charging_indicator();
                           Charge_StateAE_ControlHandler();
                          #ifdef SKY_DEBUG  
                              printf("A\r\n");
                          #endif
                    }
                   else if((RESET != phone_detect_status)&&
                           (RESET != camera_detect_status)&&
                          // (RESET== wireless_detect_status)
                           (RESET!= wireless_detect_status)
                          )     //State E  phone: 0 ,camera: 1, usb in: 0, Wirelsess Base :1   
                    {
                          
                           if(Charge_Obj_Data.camera_on_off_flag == TRUE)
                            {
                                Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_E;
                                Charge_Obj_Data.bat_obj.bat_det_time = 0;
                                Charge_StateAE_ControlHandler();
                                Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                /*
                                            if(Charge_Obj_Data.camera_uart_on_flag == TRUE)   // CAM on led dicator
                                        {
                                                Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                            COR_AP_ShorKey_Common_Handler();
                                        }
                                         else
                                                         */
                                {
                                    Set_Start_Charging_indicator();
                                }
                                #ifdef SKY_DEBUG  
                                 printf("E\r\n");
                                #endif
                            }
                          else
                            {
                               Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_A;
                               Charge_Obj_Data.bat_obj.bat_det_time = 0;
                               Set_Start_Charging_indicator();
                               Charge_StateAE_ControlHandler();
                                #ifdef SKY_DEBUG  
                                    printf("A\r\n");
                                #endif
                            }
                    }
                   #endif
                   if((RESET != phone_detect_status)&&
                      (RESET != wireless_detect_status)
                       )         // State A   phone: 0 ,camera: 0, usb in: 0, Wirelsess Base :1    
                                 //State E  phone: 0 ,camera: 1, usb in: 0, Wirelsess Base :1   
                    {
                           Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_A_E;
                           Charge_Obj_Data.bat_obj.bat_det_time = 0;
                           Set_Start_Charging_indicator();
                           Charge_StateAE_ControlHandler();
                           Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                          #ifdef SKY_DEBUG  
                              printf("A_E\r\n");
                          #endif
                    }
                   else if((RESET != phone_detect_status)&&
                           (RESET != camera_detect_status)&&
                           //(RESET != wireless_detect_status)
                           (RESET == wireless_detect_status)
                          )     //State D  phone: 0 ,camera: 1, usb in: 0, Wirelsess Base :0 
                    {
                          if(Charge_Obj_Data.camera_on_off_flag == TRUE)
                            {
                                Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_D;
                                Get_Battery_Percent();
                                lowbattery_cnt = 0;
                               // Charge_Obj_Data.bat_obj.bat_discharge_time  = 0;
                                Charge_Obj_Data.bat_obj.bat_discharge_time = (CHECK_BATTERY_DISCHARGE_TIME-188);  // delay 3S 
                               // Charge_StateD_ControlHandler();
                                if(Charge_Obj_Data.camera_uart_on_flag == TRUE)   // CAM on led dicator
                                {
                                    Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                    //COR_AP_ShorKey_Common_Handler();
                                    Charge_Obj_Data.bat_obj.bat_level =  Battery_Get_Vol_Level(Charge_Obj_Data.bat_obj.bat_remain_percent);
                                    Set_Led_Indicator_State(BAT_ROLL_INDICATE);
                                 }
                                else
                                {
                                    Set_Led_Indicator_State(STATE_IDLE);
                                }
                                #ifdef SKY_DEBUG  
                                    printf("D\r\n");
                                #endif
                            }
                          else
                            {
                               Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_DEFAULT;
                               Set_Led_Indicator_State(STATE_IDLE);
                                #ifdef SKY_DEBUG  
                                    printf("X1\r\n");
                                #endif
                            }
                    }
                   else if((RESET == phone_detect_status)&&
                           (RESET == camera_detect_status)&&
                           //(RESET != wireless_detect_status)
                           (RESET == wireless_detect_status)
                          )     //State H  phone: 1 ,camera: 0, usb in: 0, Wirelsess Base :0
                    {
                         if( Charge_Obj_Data.ChargeCtlState != CHARGE_STATE_H_Y)
                          {
                              Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_H;
                              Charge_StateH_ControlHandler();
                              Charge_Obj_Data.H_state_key_cnt = 0;
                              Set_Led_Indicator_State(STATE_IDLE);
                              #ifdef SKY_DEBUG  
                                  printf("H\r\n");
                              #endif
                          }
                    }
                   else if((RESET == phone_detect_status)&&
                           (RESET == camera_detect_status)&&
                          // (RESET == wireless_detect_status)
                           (RESET != wireless_detect_status)
                          )     //State I  phone: 1 ,camera: 0, usb in: 0, Wirelsess Base :1
                    {
                         if( Charge_Obj_Data.ChargeCtlState != CHARGE_STATE_I)
                         {
                              Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_I;
                              Charge_Obj_Data.bat_obj.bat_det_time = 0;
                              Charge_Obj_Data.check_phone_fullcharge_time = 0;
                              OTG_delay_State = OTG_DELAY_START;
                              GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);  //fbli temp mask
                              Charge_Obj_Data.check_OTG_Delay_TimeOut = OTG_OFF_DELAY_TIMEOUT;
                             // Charge_Obj_Data.check_Wirless_S1S2_Delay_time = WIRLESS_S1S2_DELAY_TIMEOUT;
                             // Charge_StateI_ControlHandler();
                              Set_Start_Charging_indicator();
                            // Set_Led_Indicator_State(STATE_IDLE); //fbli temp add
                         }
                          #ifdef SKY_DEBUG  
                              printf("I\r\n");
                          #endif
                    }
                   else if((RESET == phone_detect_status)&&
                           (RESET != camera_detect_status)&&
                           //(RESET != wireless_detect_status)
                           (RESET == wireless_detect_status)
                          )     //State L  phone: 1 ,camera: 1, usb in: 0, Wirelsess Base :0
                    {
                        
                          #if 0 //def SKY_DEBUG  
                            Charge_Obj_Data.camera_on_off_flag  = TRUE;
                             Charge_StateL_ControlHandler();
                          #endif
                          
                          if(Charge_Obj_Data.camera_on_off_flag == TRUE)
                            {
                                Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_L;
                                Get_Battery_Percent();
                                 lowbattery_cnt = 0;
                               // Charge_Obj_Data.bat_obj.bat_discharge_time  = 0;
                                Charge_Obj_Data.bat_obj.bat_discharge_time = (CHECK_BATTERY_DISCHARGE_TIME-188);  // delay 3S 
                                if(Charge_Obj_Data.camera_uart_on_flag == TRUE)   // CAM on led dicator
                                {
                                    Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                   // COR_AP_ShorKey_Common_Handler();
                                    Charge_Obj_Data.bat_obj.bat_level =  Battery_Get_Vol_Level(Charge_Obj_Data.bat_obj.bat_remain_percent);

                                //    Charge_Obj_Data.bat_obj.bat_level = 0;
                                    Set_Led_Indicator_State(BAT_ROLL_INDICATE);
                                 }
                                else
                                {
                                    Set_Led_Indicator_State(STATE_IDLE);
                                }
                                 #ifdef SKY_DEBUG  
                                    printf("L\r\n");
                                 #endif
                            }
                          else
                            {
                                lowbattery_cnt = 0;
                               Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_H;
                               Charge_Obj_Data.H_state_key_cnt = 0;
                             //  Charge_Obj_Data.check_switch_state_time = 248;
                               Charge_StateH_ControlHandler();
                               Set_Led_Indicator_State(STATE_IDLE);
                                #ifdef SKY_DEBUG  
                                    printf("H 2\r\n");
                                #endif
                            }
                    }
                   else if((RESET  == phone_detect_status)&&
                           (RESET != camera_detect_status)&&
                           //(RESET  == wireless_detect_status)
                           (RESET  != wireless_detect_status)
                          )     //State M  phone: 1 ,camera: 1, usb in: 0, Wirelsess Base :1
                    {
                         if(Charge_Obj_Data.camera_on_off_flag == TRUE)
                         {
                               Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_M;
                               Charge_Obj_Data.bat_obj.bat_det_time = 0;
                               Charge_StateM_ControlHandler();
                               Charge_Obj_Data.check_OTG_Delay_TimeOut = OTG_ON_DELAY_TIMEOUT;
                               Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                             /*
                              if(Charge_Obj_Data.camera_uart_on_flag == TRUE)   // CAM on led dicator
                              {
                                   Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                   COR_AP_ShorKey_Common_Handler();
                               }
                               else
                               */
                               {
                                   Set_Start_Charging_indicator();
                               }
                          #ifdef SKY_DEBUG  
                              printf("M\r\n");
                          #endif
                         }
                          else
                          {
                                Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_I;
                                Charge_Obj_Data.bat_obj.bat_det_time = 0;
                                Charge_Obj_Data.check_phone_fullcharge_time = 0;
                                GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
                                Charge_Obj_Data.check_OTG_Delay_TimeOut = OTG_OFF_DELAY_TIMEOUT;
                                OTG_delay_State = OTG_DELAY_START;
                               // Charge_StateI_ControlHandler();
                                Set_Start_Charging_indicator();
                                 #ifdef SKY_DEBUG  
                                    printf("I\r\n");
                                #endif
                          }
                    }
                    else
                    {
                         Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_DEFAULT;
                         Set_Led_Indicator_State(STATE_IDLE);
                         #ifdef SKY_DEBUG  
                              printf("X2\r\n");
                          #endif
                    }
                   
                }
                else  // usb plug in 
                {
                    if((RESET== phone_detect_status)&&
                       (RESET != camera_detect_status)
                    //   &&(RESET != wireless_detect_status)
                       )         // State N   phone: 1 ,camera: 1, usb in: 1, Wirelsess Base :0
                    {
                           #if 0 //def SKY_DEBUG  
                            Charge_Obj_Data.camera_on_off_flag  = TRUE;
                          #endif
                          Charge_Obj_Data.bat_obj.bat_det_time = 0;
                          if(Charge_Obj_Data.camera_on_off_flag == TRUE)
                            {
                                Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_N;
                                Charge_StateN_ControlHandler();
                                #if 0 //def SKY_DEBUG  
                                GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, SET);
                                #endif
                                
                                Charge_Obj_Data.check_OTG_Delay_TimeOut = OTG_ON_DELAY_TIMEOUT;
                                Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                //COR_AP_ShorKey_Common_Handler();
                                /*
                               if(Charge_Obj_Data.camera_uart_on_flag == TRUE)   // CAM on led dicator
                                {
                                    Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                    COR_AP_ShorKey_Common_Handler();
                                }
                                else
                                */
                                {
                                    Set_Start_Charging_indicator();
                                }
                                #ifdef SKY_DEBUG  
                                    printf("N\r\n");
                                #endif
                            }
                          else
                            {
                                Charge_Obj_Data.usb_det_adc = USB_v_Detect_Get_Vol();
                          
                                if( Charge_Obj_Data.usb_det_adc <SLOW_CHARGE_ADC_VALUE)// slow charge adapter
                                {
                                    Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_J2;  //;CHARGE_STATE_J1
                                    Charge_Obj_Data.check_phone_fullcharge_time = 0;
                                    Charge_Obj_Data.check_fastcharge_time = 0;
                                    GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
                                    GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
                                    GPIO_WriteBit(S1_PORT, S1_PINS, RESET);
                                    GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
                                    Charge_Obj_Data.check_OTG_Delay_TimeOut = OTG_OFF_DELAY_TIMEOUT;
                                    OTG_delay_State  = OTG_DELAY_START;
                                    Set_Start_Charging_indicator();
                                    #ifdef SKY_DEBUG  
                                        printf("J2\r\n");
                                    #endif
                                 }
                                /*
                                else if(Charge_Obj_Data.usb_det_adc >FAST_CHARGE_ADC_VALUE) // fast charge adapter
                                {
                                 Charge_Obj_Data.check_phone_fullcharge_time = 0;
                                     Charge_Obj_Data.ChargeCtlState     = CHARGE_STATE_J1;
                                     Charge_Obj_Data.ChargeTarget     = CHARGE_PHONE;
                                     Charge_StateJ1_ControlHandler();
                                     Set_Led_Indicator_State(STATE_IDLE);
                                     #ifdef SKY_DEBUG  
                                       printf("J1_1\r\n");
                                     #endif
                                   }
                                   */
                         }
                    }
                    /*
                   else if((RESET  == phone_detect_status)&&
                           (RESET != camera_detect_status)&&
                           (RESET== wireless_detect_status)
                          )     //State O  phone: 1 ,camera: 1, usb in: 1, Wirelsess Base :1
                    {
                          if(Charge_Obj_Data.camera_on_off_flag == TRUE)
                            {
                                Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_O;
                                Set_Start_Charging_indicator();
                                #ifdef SKY_DEBUG  
                                    printf("\r\n O State \r\n");
                                #endif
                            }
                          else
                            {
                               Judge_J_State();
                            }
                    }
                    */
                  else if((RESET != phone_detect_status)
                        // &&(RESET  == camera_detect_status)
                         // && (RESET != wireless_detect_status)
                          )     //State B  phone: 0 ,camera: 0, usb in: 1, Wirelsess Base :0
                    {
                          Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_B_F;
                          Charge_Obj_Data.bat_obj.bat_det_time = 0;
                          Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                          Set_Start_Charging_indicator();
                           Charge_StateB_F_ControlHandler();
                          #ifdef SKY_DEBUG  
                              printf("B_F\r\n");
                          #endif
                    }
                  #if 0
                   else if((RESET != phone_detect_status)&&
                           (RESET  == camera_detect_status)
                         // && (RESET != wireless_detect_status)
                          )     //State B  phone: 0 ,camera: 0, usb in: 1, Wirelsess Base :0
                    {
                          Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_B;
                          Charge_Obj_Data.bat_obj.bat_det_time = 0;
                          Set_Start_Charging_indicator();
                           Charge_StateB_ControlHandler();
                          #ifdef SKY_DEBUG  
                              printf("B\r\n");
                          #endif
                    }
                   /*
                   else if((RESET != phone_detect_status)&&
                           (RESET  == camera_detect_status)&&
                           (RESET== wireless_detect_status)
                          )     //State C  phone: 0 ,camera: 0, usb in: 1, Wirelsess Base :1
                    {
                          Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_C;
                          Set_Start_Charging_indicator();
                          #ifdef SKY_DEBUG  
                              printf("\r\n C State \r\n");
                          #endif
                    }
                    */
                   else if((RESET != phone_detect_status)&&
                           (RESET != camera_detect_status)
                         //  &&(RESET != wireless_detect_status)
                          )     //State F  phone: 0 ,camera: 1, usb in: 1, Wirelsess Base :0
                    {
                            if(Charge_Obj_Data.camera_on_off_flag == TRUE)
                            {
                                 Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_F;
                                 Charge_Obj_Data.bat_obj.bat_det_time = 0;
                                 Charge_StateF_ControlHandler();
                                 Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                 /*
                                  if(Charge_Obj_Data.camera_uart_on_flag == TRUE)   // CAM on led dicator
                                    {
                                         Charge_Obj_Data.camera_uart_on_flag  = FALSE;
                                        COR_AP_ShorKey_Common_Handler();
                                    }
                                    else
                                    */
                                    {
                                        Set_Start_Charging_indicator();
                                     }
                                #ifdef SKY_DEBUG  
                                    printf("F\r\n");
                                #endif
                            }
                          else
                            {
                               Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_B;
                               Charge_Obj_Data.bat_obj.bat_det_time = 0;
                               Set_Start_Charging_indicator();
                                Charge_StateB_ControlHandler();
                                #ifdef SKY_DEBUG  
                                    printf("B\r\n");
                                #endif
                            }
                    }
                   #endif
                   /*
                   else if((RESET != phone_detect_status)&&
                           (RESET != camera_detect_status)&&
                           (RESET== wireless_detect_status)
                          )     //State G  phone: 0 ,camera: 1, usb in: 1, Wirelsess Base :1
                    {
                          
                           if(Charge_Obj_Data.camera_on_off_flag == TRUE)
                            {
                                 Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_G;
                                 Set_Start_Charging_indicator();
                                 #ifdef SKY_DEBUG  
                                    printf("\r\n G State \r\n");
                                 #endif
                            }
                          else
                            {
                               Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_C;
                               Set_Start_Charging_indicator();
                                #ifdef SKY_DEBUG  
                                    printf("\r\n C State \r\n");
                                #endif
                            }
                    }
                    */
                    else if((RESET  == phone_detect_status)&&
                           (RESET  == camera_detect_status)
                           //&&(RESET== wireless_detect_status)
                          )     //State J  phone: 1 ,camera: 0, usb in: 1, Wirelsess Base :X
                    {

                          Charge_Obj_Data.usb_det_adc       =       USB_v_Detect_Get_Vol();
                          
                         if( Charge_Obj_Data.usb_det_adc <SLOW_CHARGE_ADC_VALUE)// slow charge adapter
                         {
                               Charge_Obj_Data.bat_obj.bat_det_time = 0;
                               Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_J2;  //;CHARGE_STATE_J1
                               OTG_delay_State  = OTG_DELAY_START;
                               Charge_Obj_Data.check_phone_fullcharge_time = 0;
                               Charge_Obj_Data.check_fastcharge_time = 0;
                               GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
                               GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
                               GPIO_WriteBit(S1_PORT, S1_PINS, RESET);
                               GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
                               Charge_Obj_Data.check_OTG_Delay_TimeOut = OTG_OFF_DELAY_TIMEOUT;
                               Set_Start_Charging_indicator();
                               #ifdef SKY_DEBUG  
                                  printf("J2_2\r\n");
                               #endif
                          }
                         else if(Charge_Obj_Data.usb_det_adc >FAST_CHARGE_ADC_VALUE) // fast charge adapter
                          {
                                 Charge_Obj_Data.check_phone_fullcharge_time = 0;
                                 Charge_Obj_Data.ChargeCtlState     = CHARGE_STATE_J1;
                                 Charge_Obj_Data.ChargeTarget     = CHARGE_PHONE;
                                 Charge_StateJ1_ControlHandler();
                                 Set_Led_Indicator_State(STATE_IDLE);
                                 #ifdef SKY_DEBUG  
                                   printf("J1_2\r\n");
                                 #endif
                           }
                          //Judge_J_State();
                       //   Charge_StateJ2_ControlHandler();
                    }
                }
              /*
               if( Charge_Obj_Data.Prev_ChargeCtlState != Charge_Obj_Data.ChargeCtlState)
                {
                    Charge_Obj_Data.Prev_ChargeCtlState = Charge_Obj_Data.ChargeCtlState;
                }
                */
         #endif
         }
        
  }
}


/**
  * @brief  judge J1 or J2 or J3 state
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */

 void Judge_J_State(void)
{
       static uint8_t fast_charge_cnt = 0;
       
        Charge_Obj_Data.usb_det_adc       =       USB_v_Detect_Get_Vol();
         if(Charge_Obj_Data.usb_det_adc >FAST_CHARGE_ADC_VALUE) // fast charge adapter
        {
             Charge_Obj_Data.check_phone_fullcharge_time = 0;
             fast_charge_cnt ++;
             if( fast_charge_cnt >= 5)  //3  // 3  //10
             {
     //            fast_charge_cnt  = 0; // fbli add 20170728
                 Charge_Obj_Data.ChargeCtlState     = CHARGE_STATE_J1;
               //  Charge_Obj_Data.ChargeTarget     = CHARGE_PHONE;
                 Charge_StateJ1_ControlHandler();
                 Set_Led_Indicator_State(STATE_IDLE);
                 #ifdef SKY_DEBUG  
                   printf("J1 %d\r\n",Charge_Obj_Data.usb_det_adc);
                 #endif
             }
        }
       #if 0  //fbli temp mask
        else if( Charge_Obj_Data.usb_det_adc <SLOW_CHARGE_ADC_VALUE)// slow charge adapter
        {
           // if(Charge_Obj_Data.ChargeCtlState != CHARGE_STATE_J2)
             {
                fast_charge_cnt  = 0;
                Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_J2;  //;CHARGE_STATE_J1
                Charge_Obj_Data.check_phone_fullcharge_time = 0;
                Charge_Obj_Data.check_fastcharge_time = 0;
             Charge_StateJ2_ControlHandler();
                Set_Start_Charging_indicator();
                #ifdef SKY_DEBUG  
                    printf("J2 %d \r\n",Charge_Obj_Data.usb_det_adc);
                #endif
             }
         }
         
         else
        {
           //  Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_DEFAULT;
           //  Set_Led_Indicator_State(STATE_IDLE);
             #if 0 //def SKY_DEBUG  
                printf("Default State %d \r\n",Charge_Obj_Data.usb_det_adc);
             #endif
        }
        #endif
       

}

/**
  * @brief  State A control handler.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateAE_ControlHandler(void)
{
  // State A
 //         S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //    1   0   0    0    0    0      0      0        0          1
    
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, RESET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);
   GPIO_WriteBit(S6_PORT, S6_PINS, RESET);
   GPIO_WriteBit(S7_PORT, S7_PINS, RESET);
   GPIO_WriteBit(S8_PORT, S8_PINS, RESET);
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}

#if 0
/**
  * @brief  State E control handler.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateE_ControlHandler(void)
{
  // State E
   //     S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //    1   0   0    0    0    0      0      0        0          1
    
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, RESET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);
   GPIO_WriteBit(S6_PORT, S6_PINS, RESET);
   GPIO_WriteBit(S7_PORT, S7_PINS, RESET);
   GPIO_WriteBit(S8_PORT, S8_PINS, RESET);
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}
#endif
/**
  * @brief  State D control handler.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateD_ControlHandler(void)
{
  // State D
   //     S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //    0   0   0    1    0    0      0      1        0          0
    
   GPIO_WriteBit(S1_PORT, S1_PINS, RESET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, SET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);
   GPIO_WriteBit(S6_PORT, S6_PINS, RESET);
   GPIO_WriteBit(S7_PORT, S7_PINS, RESET);
   GPIO_WriteBit(S8_PORT, S8_PINS, SET);
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, RESET);
}

/**
  * @brief  State H control handler.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateH_ControlHandler(void)
{
  Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_H;
  // State H
  //     S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //    0   0   0    0    0    0      0   0        0          1
    
   GPIO_WriteBit(S1_PORT, S1_PINS, RESET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, RESET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);
   GPIO_WriteBit(S6_PORT, S6_PINS, RESET);
   GPIO_WriteBit(S7_PORT, S7_PINS, RESET);
   GPIO_WriteBit(S8_PORT, S8_PINS, RESET);
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}

/**
  * @brief  State H control handler.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateH_Y_ControlHandler(void)
{
  // State H_Y
 
 //         S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //    0   0   1    1    1    0      0      1        0          0
    
  // GPIO_WriteBit(S1_PORT, S1_PINS, RESET);
  // GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, SET);
   GPIO_WriteBit(S4_PORT, S4_PINS, SET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, SET);
 //  GPIO_WriteBit(S6_PORT, S6_PINS, RESET);
 //  GPIO_WriteBit(S7_PORT, S7_PINS, RESET);
   GPIO_WriteBit(S8_PORT, S8_PINS, SET);
 //  GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, RESET);
   
   GPIO_WriteBit(KEY_PORT, KEY_PINS, SET);
   Charge_Obj_Data.KEY_timeout = 0;
   Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_H_Y;
   Charge_Obj_Data.bat_obj.bat_discharge_time  = 0;

   //mask to match new PCB
   /*
   OTG_delay_State = OTG_DELAY_START;
   Charge_Obj_Data.check_OTG_Delay_TimeOut = 6;  //6
   */
}
/**
  * @brief  State I control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateI_ControlHandler(void)
{
  // State I
  //         S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //      1   1   0    0    1    0      1     0        0          0
    

  // GPIO_WriteBit(S1_PORT, S1_PINS, SET);
  // GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
  // GPIO_WriteBit(S3_PORT, S3_PINS, SET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, SET);
   GPIO_WriteBit(S4_PORT, S4_PINS, RESET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, SET);
   GPIO_WriteBit(S6_PORT, S6_PINS, RESET);
   GPIO_WriteBit(S7_PORT, S7_PINS, RESET);
   GPIO_WriteBit(S8_PORT, S8_PINS, RESET);
//   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, RESET);
}


void Charge_StateI_Battery_ControlHandler(void)
{
  // State I
  //         S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //      1   1   0    0    1    0      1     0        0          0
    

   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, SET);
   GPIO_WriteBit(S4_PORT, S4_PINS, RESET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, SET);
   GPIO_WriteBit(S6_PORT, S6_PINS, RESET);
   GPIO_WriteBit(S7_PORT, S7_PINS, RESET);
   GPIO_WriteBit(S8_PORT, S8_PINS, RESET);
//   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}

/**
  * @brief  State L control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateL_ControlHandler(void)
{
  // State L
  //         S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //      0   0   0    1    0    1     0      1        1         0
    
   GPIO_WriteBit(S1_PORT, S1_PINS, RESET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, SET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);
   GPIO_WriteBit(S6_PORT, S6_PINS, SET);
   GPIO_WriteBit(S7_PORT, S7_PINS, RESET);
   GPIO_WriteBit(S8_PORT, S8_PINS, SET);
   
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, SET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, RESET);
}

/**
  * @brief  State M control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateM_ControlHandler(void)
{
  // State M
  
    //     S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //      1   0   0    1    0    1     0      0        1         1
    
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, SET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);
   GPIO_WriteBit(S6_PORT, S6_PINS, SET);
   GPIO_WriteBit(S7_PORT, S7_PINS, RESET);
   GPIO_WriteBit(S8_PORT, S8_PINS, RESET);
   //GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, SET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, RESET);
//   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}

/**
  * @brief  State N control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateN_ControlHandler(void)
{
  // State N
 //           S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //      1   0   0    1    0    1     1      1        1         1
    
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, SET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);
   GPIO_WriteBit(S6_PORT, S6_PINS, SET);
   GPIO_WriteBit(S7_PORT, S7_PINS, SET);
   GPIO_WriteBit(S8_PORT, S8_PINS, SET);
   
   //GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, SET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}

#if 0
/**
  * @brief  State O control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateO_ControlHandler(void)
{
  // State O
   //   S1 S2 S3 S4 S5 S6 S8_OTG, I_set
  //     1   0   0    0   1   0      1           0
#ifdef S2_OUT_EN
  GPIO_Init(S2_PORT,S2_PINS,GPIO_Mode_Out_PP_Low_Slow);
  #endif   
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, RESET);
   GPIO_WriteBit(S5_PORT, S5_PINS, SET);
   GPIO_WriteBit(S6_PORT, S6_PINS, RESET);
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, SET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, RESET);
}
#endif
/**
  * @brief  State B control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateB_F_ControlHandler(void)
{
  // State B
  //        S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //      1   0   0    1    0    0     1      1        0         1
    
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, SET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);
   GPIO_WriteBit(S6_PORT, S6_PINS, RESET);
   GPIO_WriteBit(S7_PORT, S7_PINS, SET);
   GPIO_WriteBit(S8_PORT, S8_PINS, SET);
   
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}

#if 0
/**
  * @brief  State C control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateC_ControlHandler(void)
{
  // State C
   //   S1 S2 S3 S4 S5 S6 S8_OTG, I_set
  //     1   0   0    0   1   1      0           1
#ifdef S2_OUT_EN
  GPIO_Init(S2_PORT,S2_PINS,GPIO_Mode_Out_PP_Low_Slow);
  #endif   
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, RESET);
   GPIO_WriteBit(S5_PORT, S5_PINS, SET);
   GPIO_WriteBit(S6_PORT, S6_PINS, SET);
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}

/**
  * @brief  State F control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateF_ControlHandler(void)
{
  // State F
 //          S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //      1   0   0    1    0    0     1      1        0         1
    
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, SET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);
   GPIO_WriteBit(S6_PORT, S6_PINS, RESET);
   GPIO_WriteBit(S7_PORT, S7_PINS, SET);
   GPIO_WriteBit(S8_PORT, S8_PINS, SET);
   
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}
#endif
#if 0
/**
  * @brief  State G control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateG_ControlHandler(void)
{
  // State G
  //   S1 S2 S3 S4 S5 S6 S8_OTG, I_set
  //     1   0   0    0   1   1      0           1
#ifdef S2_OUT_EN
  GPIO_Init(S2_PORT,S2_PINS,GPIO_Mode_Out_PP_Low_Slow);
  #endif   
  GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, RESET);
   GPIO_WriteBit(S5_PORT, S5_PINS, SET);
   GPIO_WriteBit(S6_PORT, S6_PINS, SET);
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}
#endif
/**
  * @brief  State J1 control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateJ1_ControlHandler(void)
{
  // State J1
 //          S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //      0   1   0    1    1    1     1      1        0         0
    
   GPIO_WriteBit(S1_PORT, S1_PINS, RESET);
   GPIO_WriteBit(S2_PORT, S2_PINS, SET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, SET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, SET);
   GPIO_WriteBit(S6_PORT, S6_PINS, SET);
   GPIO_WriteBit(S7_PORT, S7_PINS, SET);
   GPIO_WriteBit(S8_PORT, S8_PINS, SET);
   
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, RESET);
   
}

/**
  * @brief  State J1 control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateJ1_BatteryChargeControlHandler(void)
{
  // State J1
//            S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //      1   0   0    1    1    1     1      1        0         1
    
   uint16_t i;
   GPIO_WriteBit(S2_PORT, S2_PINS, SET);  //RESET
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, SET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, SET);
   GPIO_WriteBit(S6_PORT, S6_PINS, RESET);
   GPIO_WriteBit(S7_PORT, S7_PINS, RESET);
   GPIO_WriteBit(S8_PORT, S8_PINS, SET);
   for(i= 0; i<1000; i++)
    {
        ;
    }
   GPIO_WriteBit(S7_PORT, S7_PINS, SET);
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}
/**
  * @brief  State J2 control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateJ2_ControlHandler(void)
{
  // State J2
 //          S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //      1   1   0    1    1    1     1      1        0         0
 
    GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
 //  uint16_t i = 0;
  /*
  for(i= 0; i<60000; i++)
        {
          ;
        }  
      */
     // GPIO_WriteBit(S5_PORT, S5_PINS, RESET);
      //GPIO_WriteBit(S6_PORT, S6_PINS, SET);
      
      GPIO_WriteBit(S1_PORT, S1_PINS, SET);
      GPIO_WriteBit(S7_PORT, S7_PINS, SET);
      GPIO_WriteBit(S8_PORT, S8_PINS, SET);
      
      GPIO_WriteBit(S2_PORT, S2_PINS, SET);
      GPIO_WriteBit(S4_PORT, S4_PINS, SET);

     /*
      for(i= 0; i<1000; i++)
        {
          ;
        }  
      */
      GPIO_WriteBit(S5_PORT, S5_PINS, SET);
      GPIO_WriteBit(S6_PORT, S6_PINS, SET);
  

      
    GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
      GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, RESET);
}


/**
  * @brief  State J1 control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateJ2_BatteryChargeControlHandler(void)
{
  // State J2
  //        S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //      1   1   0    1    1    1     1      1        0         1
    
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, SET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, SET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, SET);
   GPIO_WriteBit(S6_PORT, S6_PINS, SET);
   GPIO_WriteBit(S7_PORT, S7_PINS, SET);
   GPIO_WriteBit(S8_PORT, S8_PINS, SET);
   
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}

#if 0
/**
  * @brief  State J3 control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateJ3_ControlHandler(void)
{
  // State J3
    //   S1 S2 S3 S4 S5 S6 S8_OTG, I_set
  //     0   1   0    1   0   1      0           1
#ifdef S2_OUT_EN
  GPIO_Init(S2_PORT,S2_PINS,GPIO_Mode_Out_PP_Low_Slow);
  #endif   
  GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S1_PORT, S1_PINS, RESET);
   GPIO_WriteBit(S2_PORT, S2_PINS, SET);
   
   GPIO_WriteBit(S4_PORT, S4_PINS, SET);
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);
   GPIO_WriteBit(S6_PORT, S6_PINS, SET);
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}
/**
  * @brief  State J1 control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_StateJ3_BatteryChargeControlHandler(void)
{
  // State J1
   //   S1 S2 S3 S4 S5 S6 S8_OTG, I_set
  //     1   0   0    0   0   1      0           1
#ifdef S2_OUT_EN
  GPIO_Init(S2_PORT,S2_PINS,GPIO_Mode_Out_PP_Low_Slow);
  #endif   
  GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   
   GPIO_WriteBit(S4_PORT, S4_PINS, RESET);
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);
   GPIO_WriteBit(S6_PORT, S6_PINS, SET);
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}
#endif
/**
  * @brief  Default State  control handler.
  * @param  None 
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
#if 0
void Charge_Default_ControlHandler(void)
{
  // Default State 
    //   S1 S2 S3 S4   S5  S6   S7,  S8,  S8_OTG, I_set
    //    0   0   0    0   0    0     0      0        0          1
  
   GPIO_WriteBit(S1_PORT, S1_PINS, RESET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, RESET);
   
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);
   GPIO_WriteBit(S6_PORT, S6_PINS, RESET);
   GPIO_WriteBit(S7_PORT, S7_PINS, RESET);
   GPIO_WriteBit(S8_PORT, S8_PINS, RESET);
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
}
#endif

/**
  * @brief  control I/O according to Sate .
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */

void Charge_State_Contrl_Handler(void)
{

  switch(Charge_Obj_Data.ChargeCtlState )
  {
    case CHARGE_STATE_A_E:
           // Charge_StateA_ControlHandler();
            Check_Charge_Led_Flash_State();
            Check_Battery_Temp();
            
    break;
    /*
    case CHARGE_STATE_E:
          //  Charge_StateE_ControlHandler();
            Check_Charge_Led_Flash_State();
            Check_Battery_Temp();
    break;
    */
    case CHARGE_STATE_D:
            Check_Battery_Discharge_Level();
           Charge_StateD_ControlHandler();
    break;
    /*
    case CHARGE_STATE_H:
           // Charge_StateH_ControlHandler();
           Check_Battery_Discharge_Level();
           if(Charge_Obj_Data.KEY_timeout == KEY_SET_TIMEOUT)  // keep Key high 1  timeout
            {
               GPIO_WriteBit(KEY_PORT, KEY_PINS, RESET);
            }
    break;
    */
    case CHARGE_STATE_H_Y:
           // Charge_StateH_ControlHandler();
           Check_Battery_Discharge_Level();
           if(Charge_Obj_Data.KEY_timeout == KEY_SET_TIMEOUT)  // keep Key high 1S  timeout
            {
               GPIO_WriteBit(KEY_PORT, KEY_PINS, RESET);
            }
       // mask to match new PCB
           /*
           if((Charge_Obj_Data.check_OTG_Delay_TimeOut == 0)
              && (OTG_DELAY_START == OTG_delay_State)
             )
            {
              OTG_delay_State = OTG_DELAY_END;
              GPIO_WriteBit(S6_PORT, S6_PINS, SET);
            }
            */
    break;
    case CHARGE_STATE_I:
           if((Charge_Obj_Data.check_OTG_Delay_TimeOut == 0)
              && (OTG_DELAY_START == OTG_delay_State)
             )
            {
              OTG_delay_State = OTG_DELAY_END;
              Charge_StateI_ControlHandler();
              // GPIO_WriteBit(S1_PORT, S1_PINS, SET);
              //  GPIO_WriteBit(S2_PORT, S2_PINS, SET);
               // GPIO_WriteBit(S3_PORT, S3_PINS, SET);
            }
           #if 0
           if(Charge_Obj_Data.check_Wirless_S1S2_Delay_time  == 0)
            {
                GPIO_WriteBit(S1_PORT, S1_PINS, SET);
                GPIO_WriteBit(S2_PORT, S2_PINS, SET);
               // GPIO_WriteBit(S3_PORT, S3_PINS, SET);
            }
           #endif
           #if 1
            if(Charge_Obj_Data.check_phone_fullcharge_time >= CHECK_PHONE_FULL_CHARGE_TIME) // time 1Min check if phone full charge
            {
                Charge_Obj_Data.check_phone_fullcharge_time  = 0;

                Charge_Obj_Data.current_det_adc   =  Current_Detect_Get_Vol();
                #ifdef SKY_DEBUG  
                 printf("\r\n I phone charge: %d \r\n",Charge_Obj_Data.current_det_adc);
             #endif
                if(Charge_Obj_Data.current_det_adc <PHONE_CHARGE_FULL_ADC_MAX_VALUE)
                {
                    Charge_StateI_Battery_ControlHandler();
                }
             }
          #endif

           Check_Charge_Led_Flash_State();
           //Check_Battery_Temp();  //fbli mask 20170728
    break;
    case CHARGE_STATE_I_Y:
            Check_Charge_Led_Flash_State();
            Check_Battery_Temp();  
     break;
     
    case CHARGE_STATE_L:
            Check_Battery_Discharge_Level();
            if(Charge_Obj_Data.bat_obj.bat_remain_percent > LOW_BATTERY_PROTECT_VOL)
            {
                Charge_StateL_ControlHandler();
            }
    break;
    case CHARGE_STATE_M:
             if(Charge_Obj_Data.check_OTG_Delay_TimeOut == 0)
              GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, SET);
            Check_Charge_Led_Flash_State();
           // Check_Battery_Temp();  //fbli mask 20170728
    break;
    case CHARGE_STATE_N:
           if(Charge_Obj_Data.check_OTG_Delay_TimeOut == 0)
              GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, SET);
            Check_Charge_Led_Flash_State();
            Check_Battery_Temp();
    break;
    /*
    case CHARGE_STATE_O:
            Charge_StateO_ControlHandler();
            Check_Charge_Led_Flash_State();
    break;
    */
    case CHARGE_STATE_B_F:
           // Charge_StateB_ControlHandler();
            Check_Charge_Led_Flash_State();
            Check_Battery_Temp();
    break;
    /*
    case CHARGE_STATE_C:
            Charge_StateC_ControlHandler();
            Check_Charge_Led_Flash_State();
    break;
    */
    /*
    case CHARGE_STATE_F:
          //  Charge_StateF_ControlHandler();
            Check_Charge_Led_Flash_State();
            Check_Battery_Temp();
    break;
    */
    /*
    case CHARGE_STATE_G:
            Charge_StateG_ControlHandler();
            Check_Charge_Led_Flash_State();
        */
    case CHARGE_STATE_J1:
        #if 0 //def SKY_DEBUG  
         if(Charge_Obj_Data.check_fastcharge_time >= CHECK_FASTCHARGE_TIME)
            {
                 Charge_Obj_Data.check_fastcharge_time  = 0;
                 Judge_J_State();
            }
        #endif

        /*
        switch(Charge_Obj_Data.ChargeTarget)
        {
           case CHARGE_PHONE:
            if(Charge_Obj_Data.check_phone_fullcharge_time >= CHECK_PHONE_FULL_CHARGE_TIME) // time 1Min check if phone full charge
            {
                Charge_Obj_Data.check_phone_fullcharge_time  = 0;

                Charge_Obj_Data.current_det_adc   =  Current_Detect_Get_Vol();
                #ifdef SKY_DEBUG  
                   printf("\r\n J1 phone charge: %d \r\n",Charge_Obj_Data.current_det_adc);
                 #endif
                if(Charge_Obj_Data.current_det_adc <PHONE_CHARGE_FULL_ADC_MAX_VALUE)
                 {
                    if(Charge_Obj_Data.led_obj.led_indicate_state != BAT_CHARGING_INDICATE)
                    {
                        Set_Start_Charging_indicator();
                    }
                    Charge_StateJ1_BatteryChargeControlHandler();
                    Charge_Obj_Data.ChargeTarget     = CHARGE_PHONE_CASE;
                   // Charge_Obj_Data.check_fastcharge_time  = 0;
                    #if 0 //def SKY_DEBUG  
                   printf("phone full charge Case\r\n");
                   #endif
                 }
            }
            break;
          #if 0
           case  CHARGE_CASE:
                 if(Charge_Obj_Data.check_fastcharge_time >= CHECK_FASTCHARGE_TIME)
                 {
                    Charge_Obj_Data.check_fastcharge_time  = 0;
                   // GPIO_WriteBit(S3_PORT, S3_PINS, SET);    // keep charge PHONE and CASE at the same time
                    Charge_Obj_Data.ChargeTarget     = CHARGE_PHONE_CASE;
                    #ifdef SKY_DEBUG  
                   printf("charge Case and Phone \r\n");
                   #endif
                 }
            break;

           case CHARGE_PHONE_CASE:
                    ;
            break;
          #endif
        }
        */
        if(Charge_Obj_Data.check_phone_fullcharge_time >= CHECK_PHONE_FULL_CHARGE_TIME) // time 1Min check if phone full charge
        {
                Charge_Obj_Data.check_phone_fullcharge_time  = 0;

                Charge_Obj_Data.current_det_adc   =  Current_Detect_Get_Vol();
                #ifdef SKY_DEBUG  
                   printf("\r\n J1 phone charge: %d \r\n",Charge_Obj_Data.current_det_adc);
                 #endif
                if(Charge_Obj_Data.current_det_adc <PHONE_CHARGE_FULL_ADC_MAX_VALUE)
                 {
                    if(Charge_Obj_Data.led_obj.led_indicate_state != BAT_CHARGING_INDICATE)
                    {
                        Set_Start_Charging_indicator();
                        Charge_StateJ1_BatteryChargeControlHandler();
                    }
                    
                  //  Charge_Obj_Data.ChargeTarget     = CHARGE_PHONE_CASE;
                   // Charge_Obj_Data.check_fastcharge_time  = 0;
                    #if 0 //def SKY_DEBUG  
                   printf("phone full charge Case\r\n");
                   #endif
                 }
       }
        if(Charge_Obj_Data.led_obj.led_indicate_state == BAT_CHARGING_INDICATE)
        {
            Check_Charge_Led_Flash_State();
            Check_Battery_Temp();
        }
    break;
   case CHARGE_STATE_J1_Y:
         if((Charge_Obj_Data.check_OTG_Delay_TimeOut == 0)
                && (OTG_DELAY_START == OTG_delay_State)
               )
             {
                 GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
                 GPIO_WriteBit(S7_PORT, S7_PINS, SET);
                 GPIO_WriteBit(S1_PORT, S1_PINS, SET);
                 GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
                 OTG_delay_State  = OTG_DELAY_END;
             }
        Check_Charge_Led_Flash_State();
        Check_Battery_Temp();
    break;
    case CHARGE_STATE_J2:
             if((Charge_Obj_Data.check_OTG_Delay_TimeOut == 0)
                && (OTG_DELAY_START == OTG_delay_State)
               )
             {
                  #ifdef SKY_DEBUG  
                    printf("J2 Chg\r\n");
                #endif
                  Charge_StateJ2_ControlHandler();
                  OTG_delay_State  = OTG_DELAY_END;
             }
             
            if(Charge_Obj_Data.check_fastcharge_time >= CHECK_FASTCHARGE_TIME)
            {
                 Charge_Obj_Data.check_fastcharge_time  = 0;
                 Judge_J_State();
               //  Charge_StateJ2_ControlHandler();
            }
            if(Charge_Obj_Data.check_phone_fullcharge_time >= CHECK_PHONE_FULL_CHARGE_TIME) // time 1Min check if phone full charge
            {
                Charge_Obj_Data.check_phone_fullcharge_time  = 0;

                Charge_Obj_Data.current_det_adc   =  Current_Detect_Get_Vol();
                #ifdef SKY_DEBUG  
               printf("\r\n J2 phone charge: %d \r\n",Charge_Obj_Data.current_det_adc);
             #endif
                if(Charge_Obj_Data.current_det_adc <PHONE_CHARGE_FULL_ADC_MAX_VALUE)
                {
                    Charge_StateJ2_BatteryChargeControlHandler();
                }
             }
            Check_Charge_Led_Flash_State();
            Check_Battery_Temp();
    break;

    case CHARGE_STATE_J2_Y:
        if((Charge_Obj_Data.check_OTG_Delay_TimeOut == 0)
                && (OTG_DELAY_START == OTG_delay_State)
           )
          {
                 #ifdef SKY_DEBUG  
                    printf("J2_Y Sp OTG \r\n");
                 #endif
                 GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
                 OTG_delay_State  = OTG_DELAY_END;
          }
        Check_Charge_Led_Flash_State();
        Check_Battery_Temp();
    break;
    #if 0
    case CHARGE_STATE_J3:
            if(Charge_Obj_Data.check_phone_fullcharge_time == CHECK_PHONE_FULL_CHARGE_TIME) // time 1Min check if phone full charge
           {
                Charge_Obj_Data.check_phone_fullcharge_time  = 0;

                Charge_Obj_Data.current_det_adc   =  Current_Detect_Get_Vol();
                if(Charge_Obj_Data.current_det_adc <PHONE_CHARGE_FULL_ADC_MAX_VALUE)
                {
                    if(Charge_Obj_Data.led_obj.led_indicate_state != BAT_CHARGING_INDICATE)
                    {
                   // Set_Led_Indicator_State(BAT_CHARGING_INDICATE);
                    Set_Start_Charging_indicator();
                    }
                    Charge_StateJ3_BatteryChargeControlHandler();
                }
                else
                {
                    Charge_StateJ3_ControlHandler();
                    Set_Led_Indicator_State(STATE_IDLE);
                    // Led_ALL_OFF();
                }
            }
            if(Charge_Obj_Data.led_obj.led_indicate_state == BAT_CHARGING_INDICATE)
                Check_Charge_Led_Flash_State();
    break;
    #endif
    case CHARGE_STATE_H:
        break;
    default:
      Charge_StateH_ControlHandler();
       Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_DEFAULT;
       
    break;
  }

}

/**
  * @brief  inite timer4 and timer 16ms .
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */

void Charge_Init_Timer(void)
{
    TIM1_DeInit();
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM1,DISABLE);  
    
    CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE); 
    Charge_Time4_Start();
}


/**
  * @brief  inite user variable.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_Init_User(void)
{
//  uint16_t i = 0;
  //led
    Charge_Obj_Data.led_obj.led_flash_time = 0;
    Charge_Obj_Data.led_obj.led_3s_off_time = 0;
    Charge_Obj_Data.led_obj.led_indicate_state = STATE_IDLE;
    
    Charge_Obj_Data.H_state_key_cnt = 0;
    //usart
    //Charge_Obj_Data.usart_obj.usart_rx_time = 0;
    Charge_Obj_Data.usart_obj.usart_rx_flg  = 0;
    Charge_Obj_Data.usart_obj.buf_index = 0;
    Charge_Obj_Data.usart_obj.rx_read   = 0;
    Charge_Obj_Data.usart_obj.rx_write   = 0;
    memset(Charge_Obj_Data.usart_obj.usart_rx_buf, 0x00, sizeof(Charge_Obj_Data.usart_obj.usart_rx_buf));
    Charge_Obj_Data.bat_obj.bat_threshold = CASE_CHARING_THRESHOLD;
    //ver info
    #if 0
    Charge_Obj_Data.charge_info_obj.version_len = 21; //just test 
    memset(Charge_Obj_Data.charge_info_obj.version, 0x00, sizeof(Charge_Obj_Data.charge_info_obj.version));
    strcpy(Charge_Obj_Data.charge_info_obj.version, "Apollo-mcui7-v1.01.01");//just test
    #endif
   // Charge_StateH_ControlHandler();
    Charge_Obj_Data.state_switch_flag       = TRUE;
    Charge_Obj_Data.force_check_state_flag  = TRUE;
    Charge_Obj_Data.check_switch_state_time = 0;
    OTG_delay_State  = OTG_IDLE;
    Charge_Obj_Data.check_BatTemp_TimeOut = CHECK_BATTEMP_TIMEOUT;
}

#if 0  //def WDOG_EN
/**
  * @brief  inite iwatch dog to timeout 1.684s to reset .
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_Init_iWatchdog(void)
{
   IWDG_Enable();
   /* Enable write access to IWDG_PR and IWDG_RLR registers */
   IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
   /* Set IWDG timeout */
   //IWDG_SetPrescaler(IWDG_Prescaler_32); OK
   IWDG_SetPrescaler(IWDG_Prescaler_256);  // (256*(250+1))/38k Hz = 1.684 S
   IWDG_SetReload(250);
   /* Refresh IWDG */
   IWDG_ReloadCounter();
}
#endif

#if 0
BATTERY_LEVEL Charge_Set_Bat_Threshold(unsigned char val)
{
    if (0 == val)
    {
        return BATTERY_LEVEL_LOW_PROTECTION;
    }
    else if (25 == val)
    {
        return BATTERY_LEVEL_PERCENT_25;
    }
    else if (50 == val)
    {
        return BATTERY_LEVEL_2;
    }
    else if (75 == val)
    {
        return BATTERY_LEVEL_3;
    }
    else if (100 <= val)
    {
        return BATTERY_LEVEL_FULL;
    }
    
    return BATTERY_LEVEL_PERCENT_25;
}
#endif


/**
  * @brief set start charging led indicator according to read battery  percent
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Set_Start_Charging_indicator(void)
{
    Get_Battery_Percent();
    Charge_Obj_Data.bat_obj.bat_level =  Battery_Get_Vol_Level(Charge_Obj_Data.bat_obj.bat_remain_percent);
    Battery_Level_Led_Indicator(Charge_Obj_Data.bat_obj.bat_level);
    if(BATTERY_LEVEL_FULL == Charge_Obj_Data.bat_obj.bat_level)
        Set_Led_Indicator_State(BAT_LEVEL_INDICATE);
    else
        Set_Led_Indicator_State(BAT_CHARGING_INDICATE);
}


/**
  * @brief Check_Battery_Discharge_Level
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-06-03
  */
void Check_Battery_Discharge_Level(void)
{
   if(Charge_Obj_Data.bat_obj.bat_discharge_time >= CHECK_BATTERY_DISCHARGE_TIME)
    {
        #if 0 //def SKY_DEBUG  
            printf("time 1Min discharge \r\n");
        #endif
        Charge_Obj_Data.bat_obj.bat_discharge_time  = 0;
        Get_Battery_Percent();
        if(( CHARGE_STATE_H_Y == Charge_Obj_Data.ChargeCtlState)&&(Charge_Obj_Data.H_state_key_cnt == 0))
        {
            if(Charge_Obj_Data.bat_obj.bat_remain_percent < Charge_Obj_Data.bat_obj.bat_threshold)
            {
                Charge_Obj_Data.H_state_key_cnt ++;
                Charge_StateH_ControlHandler();

                 #if 0 //def SKY_DEBUG  
                 printf("reach limit \r\n");
                #endif
            }
        }
        if(Charge_Obj_Data.bat_obj.bat_remain_percent <= LOW_BATTERY_PROTECT_VOL)  // low battery notify CAM off
        {
           if( Charge_Obj_Data.ChargeCtlState != CHARGE_STATE_H_Y)
            {
                Charge_Obj_Data.bat_obj.bat_discharge_time  = (CHECK_BATTERY_DISCHARGE_TIME -10);
                if(lowbattery_cnt<3)
                {
                    Send_MCU_Shutdown_cmd();
                    lowbattery_cnt++;
                }
                else
                {
                 Charge_Obj_Data.camera_on_off_flag = FALSE;
                 lowbattery_cnt = 0;
                  Enter_halt_mode_init();
                  halt();
                  Exit_halt_mode_reinit();
                }
            }
            else  // CASE stop charging PHONE when Vat <20%
            {
                    Charge_StateH_ControlHandler();

            }
        }
        else
        {
               lowbattery_cnt = 0;
        }
    }

}

/**
  * @brief Check_Battery_Discharge_Level
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-06-30
  */

static void Check_Battery_Temp(void)
{
  if(Charge_Obj_Data.check_BatTemp_TimeOut == 0)
    {

       Charge_Obj_Data.check_BatTemp_TimeOut = CHECK_BATTEMP_TIMEOUT;
       Get_Battery_Temp();
       if(Charge_Obj_Data.bat_obj.bat_temperature < 2732)
          GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, RESET);
        else
          GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
    }
}

void Product_Test_Led_Handler(void)
{
   //Charge_Obj_Data.bat_obj.bat_remain_percent = 44; //fbli debug
   if((product_test_flag == 1) &&(product_testBat_flag == 1))
    {
             if(Charge_Obj_Data.bat_obj.bat_remain_percent<39)   // 23 
             {
                if(63 == Charge_Obj_Data.led_obj.led_flash_time)   // 1S flash
                {
                  Charge_Obj_Data.led_obj.led_flash_time = 0;
                  led_test_flash_flag = !led_test_flash_flag;
                  if(led_test_flash_flag == TRUE)
                  {
                        Led_ALL_ON();
                  }
                  else
                  {
                        Led_ALL_OFF();
                  }
                }

             }
             else if(Charge_Obj_Data.bat_obj.bat_remain_percent>44)  //28
             {
                if(19 == Charge_Obj_Data.led_obj.led_flash_time)  // 
                {
                  Charge_Obj_Data.led_obj.led_flash_time = 0;
                  led_test_flash_flag = !led_test_flash_flag;
                  if(led_test_flash_flag == TRUE)
                  {
                        Led_ALL_ON();
                  }
                  else
                  {
                        Led_ALL_OFF();
                  }
                }
             }
             else   // (39~44)
             {
                  Led_ALL_ON();
             }
       if(Charge_Obj_Data.led_obj.led_indicate_state == STATE_IDLE)
           Check_Battery_Discharge_Level();
                
     }

}
