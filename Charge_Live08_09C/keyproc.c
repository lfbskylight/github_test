
#include "stdio.h"
#include "stm8l15x.h"
#include "charge.h"
#include "battery.h"
//#include "led.h"
#include "time.h"
#include<string.h>
//#include "button.h"
#include "Keyproc.h"
#include "led.h"


KeyScanData_t  KeyScandData;

void COR_AP_Key_Function(void);
//void COR_AP_StateA_LongKey1S_Handler(void);
void COR_AP_ShorKey_Common_Handler(void);
void COR_AP_StateH_LongKey3S_Handler(void);
static void COR_AP_StateI_LongKey3S_Handler(void);
static void COR_AP_StateI_Y_LongKey3S_Handler(void);
static void COR_AP_StateJ1_LongKey3S_Handler(void);
static void COR_AP_StateJ1_Y_LongKey3S_Handler(void);
static void COR_AP_StateJ2_LongKey3S_Handler(void);
static void COR_AP_StateJ2_Y_LongKey3S_Handler(void);
static void COR_AP_StateH_Y_LongKey3S_Handler(void);
static void COR_AP_StateH_Y_ShortKey_Handler(void);

const Menu_ID_to_Key_Func_Table Menu_ID_to_KeyFunction[] =
{
 //*****************************************************************************
 // Normal                                                                     *
 //*****************************************************************************
 //{CHARGE_STATE_H,                   n_KEY_SHORT,                COR_AP_ShorKey_Common_Handler      },
 {CHARGE_STATE_H,                   n_KEY_LONGPRESS_3S,         COR_AP_StateH_LongKey3S_Handler    },
 {CHARGE_STATE_H_Y,                 n_KEY_LONGPRESS_3S,         COR_AP_StateH_Y_LongKey3S_Handler    },
 {CHARGE_STATE_H_Y,                 n_KEY_SHORT,                COR_AP_StateH_Y_ShortKey_Handler    },
 {CHARGE_STATE_I,                     n_KEY_LONGPRESS_3S,         COR_AP_StateI_LongKey3S_Handler    },
 {CHARGE_STATE_I_Y,                   n_KEY_LONGPRESS_3S,         COR_AP_StateI_Y_LongKey3S_Handler  },
 {CHARGE_STATE_J1,                    n_KEY_LONGPRESS_3S,         COR_AP_StateJ1_LongKey3S_Handler    },
 {CHARGE_STATE_J1_Y,                  n_KEY_LONGPRESS_3S,         COR_AP_StateJ1_Y_LongKey3S_Handler  },
 {CHARGE_STATE_J2,                    n_KEY_LONGPRESS_3S,         COR_AP_StateJ2_LongKey3S_Handler    },
 {CHARGE_STATE_J2_Y,                  n_KEY_LONGPRESS_3S,         COR_AP_StateJ2_Y_LongKey3S_Handler  },
};



void COR_AP_ShorKey_Common_Handler(void)
{
   // IC read battery level
   Get_Battery_Percent();
  // Charge_Obj_Data.bat_obj.bat_remain_percent = 59;
   Charge_Obj_Data.bat_obj.bat_level =  Battery_Get_Vol_Level(Charge_Obj_Data.bat_obj.bat_remain_percent);
  #ifdef SKY_DEBUG
 // Send_MCU_Shutdown_cmd();
 // Get_Battery_Temp();
 /*
   Get_Battery_avgCurrent();
   printf("p:%d l:%d\r\n",Charge_Obj_Data.bat_obj.bat_remain_percent,Charge_Obj_Data.bat_obj.bat_current);
   */
 //  if(Charge_Obj_Data.bat_obj.bat_current <CHARGING_FULL_CURRENT)
 //   {
 //     printf("full charge\r\n");
 //   }
 // Charge_Obj_Data.bat_obj.bat_voltage = BQ27541_ReadReg(BQ27541CMD_VOLT_LSB,BQ27541CMD_VOLT_MSB);
 // printf(" bat vol:%d \r\n",Charge_Obj_Data.bat_obj.bat_voltage);
  //printf(" state:%d cnt: %d \r\n",Charge_Obj_Data.ChargeCtlState,Charge_Obj_Data.H_state_key_cnt);
   printf("cal:%d\r\n",Charge_Obj_Data.bat_obj.bat_remain_percent);
  #endif
  //Send_MCU_Shutdown_cmd();
  Set_Led_Indicator_State(BAT_LEVEL_INDICATE);
}

/*
void COR_AP_StateA_LongKey1S_Handler(void)
{

  //Charge_Obj_Data.usb_det_adc       =       USB_v_Detect_Get_Vol();
                  
 // while(1);
  // Charge_Obj_Data.bat_obj.bat_level = BATTERY_LEVEL_2;
 //  Set_Led_Indicator_State(BAT_LEVEL_INDICATE);
   

}
*/

void COR_AP_StateH_LongKey3S_Handler(void)
{
 #if 0 //def SKY_DEBUG  
            printf("St chg phone\r\n");
  #endif
     Get_Battery_Percent();
     if(Charge_Obj_Data.bat_obj.bat_remain_percent > LOW_BATTERY_PROTECT_VOL)
     {
         
          if(Charge_Obj_Data.bat_obj.bat_remain_percent < Charge_Obj_Data.bat_obj.bat_threshold)
           {
                Charge_Obj_Data.H_state_key_cnt ++;
           }
           
           Charge_StateH_Y_ControlHandler();
     }
  
}
void COR_AP_StateH_Y_LongKey3S_Handler(void)
{
 #if 0 //def SKY_DEBUG  
            printf("Sp chg phone\r\n");
  #endif
  Charge_StateH_ControlHandler();
  Set_Led_Indicator_State(STATE_IDLE);
  Charge_Obj_Data.H_state_key_cnt = 0;
 // Led_ALL_OFF();
}

static void COR_AP_StateH_Y_ShortKey_Handler(void)
{
   
   #if 0  //def SKY_DEBUG  
            printf("Sp\r\n");
   #endif
   #if 0
   
    Charge_StateH_ControlHandler();
    Set_Led_Indicator_State(STATE_IDLE);
    Charge_Obj_Data.H_state_key_cnt = 0;
   #else
    if((Charge_Obj_Data.bat_obj.bat_remain_percent < Charge_Obj_Data.bat_obj.bat_threshold)&&(Charge_Obj_Data.H_state_key_cnt == 0))
    {
        Charge_Obj_Data.H_state_key_cnt++;
        Charge_StateH_ControlHandler();
    }
    else if(Charge_Obj_Data.bat_obj.bat_remain_percent <= LOW_BATTERY_PROTECT_VOL)  // low battery notify CAM off
    {
        Charge_StateH_ControlHandler();
    }
    #endif
}

static void COR_AP_StateI_LongKey3S_Handler(void)
{
  Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_I_Y;
  #ifdef SKY_DEBUG  
       printf("I_Y\r\n");
  #endif
 // GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
  GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
  GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
}
static void COR_AP_StateI_Y_LongKey3S_Handler(void)
{
  Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_I;
   #ifdef SKY_DEBUG  
       printf("I\r\n");
  #endif
 // GPIO_WriteBit(S3_PORT, S3_PINS, SET);

  Charge_StateI_ControlHandler();
}

static void COR_AP_StateJ1_LongKey3S_Handler(void) //switch charging CASE
{
  uint16_t i  = 0;
  Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_J1_Y;
  #ifdef SKY_DEBUG  
       printf("J1_Y\r\n");
  #endif
  // Charge_StateJ1_BatteryChargeControlHandler();
  
   GPIO_WriteBit(S8_PORT, S8_PINS, SET);
  
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);  //SET
   GPIO_WriteBit(S6_PORT, S6_PINS, RESET);
   GPIO_WriteBit(S7_PORT, S7_PINS, RESET);
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);

   
   // GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
  // GPIO_WriteBit(S4_PORT, S4_PINS, SET);
  // GPIO_WriteBit(S7_PORT, S7_PINS, SET);
 //  GPIO_WriteBit(S1_PORT, S1_PINS, SET);


   
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, SET);
   /*
   for(i= 0; i<1000; i++)
        {
          ;
        } 
     */
   //GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   Charge_Obj_Data.check_OTG_Delay_TimeOut = 20; //20  //30;  //50;//100  // OTG_OFF_DELAY_TIMEOUT;
   OTG_delay_State  = OTG_DELAY_START;
  // GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
  // Charge_Obj_Data.ChargeTarget     = CHARGE_CASE;
   Set_Start_Charging_indicator();
}
static void COR_AP_StateJ1_Y_LongKey3S_Handler(void) //switch charging PHONE
{
uint16_t i  = 0;
 
  //Charge_StateH_ControlHandler();
  Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_J2;
  Charge_Obj_Data.check_phone_fullcharge_time = 0;
   #ifdef SKY_DEBUG  
       printf("J2 St \r\n");
  #endif

   #if 0
   
  Charge_StateJ2_ControlHandler();
  
   #else
   
    GPIO_WriteBit(S1_PORT, S1_PINS, RESET);
   
 // GPIO_WriteBit(S7_PORT, S7_PINS, SET);
  // GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   for(i= 0; i<1000; i++)
        {
          ;
        } 
        
   GPIO_WriteBit(S5_PORT, S5_PINS, SET);
   GPIO_WriteBit(S6_PORT, S6_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, SET);
   GPIO_WriteBit(S7_PORT, S7_PINS, SET);
   #endif
//   Set_Led_Indicator_State(STATE_IDLE);
 //  Charge_Default_ControlHandler();

   Charge_Obj_Data.check_fastcharge_time = 0;
  // Charge_Obj_Data.check_OTG_Delay_TimeOut = 100;//52  // OTG_OFF_DELAY_TIMEOUT;
   OTG_delay_State  = OTG_DELAY_END;
}

static void COR_AP_StateJ2_LongKey3S_Handler(void)  // switch charging CASE
{
  Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_J2_Y;
//  Set_Led_Indicator_State(STATE_IDLE);
  #if 0
  GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
  #else    //add i/o control logic to fixed delay stop charging phone about 7~8S
  GPIO_WriteBit(S8_PORT, S8_PINS, SET);
  
   GPIO_WriteBit(S5_PORT, S5_PINS, RESET);  //SET
   GPIO_WriteBit(S6_PORT, S6_PINS, SET);
   /*
   for(i= 0; i<1000; i++)
        {
          ;
        } 
        */
   GPIO_WriteBit(S2_PORT, S2_PINS, RESET);
   GPIO_WriteBit(S7_PORT, S7_PINS, SET);

   GPIO_WriteBit(S3_PORT, S3_PINS, RESET);
   GPIO_WriteBit(S4_PORT, S4_PINS, SET);
   GPIO_WriteBit(S1_PORT, S1_PINS, SET);

 //  GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, RESET);
   GPIO_WriteBit(S8_OTG_PORT,S8_OTG__PINS, SET);
   Charge_Obj_Data.check_OTG_Delay_TimeOut = 20; //20  //30;  //50;//100  // OTG_OFF_DELAY_TIMEOUT;
   OTG_delay_State  = OTG_DELAY_START;
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, SET);
  #endif
//  Set_Start_Charging_indicator();
   #ifdef SKY_DEBUG  
       printf("J2_Y St \r\n");
  #endif
}
static void COR_AP_StateJ2_Y_LongKey3S_Handler(void)   // switch charging PHONE
{
  Charge_Obj_Data.ChargeCtlState = CHARGE_STATE_J2;
  Charge_Obj_Data.check_phone_fullcharge_time = 0;

  #if 0
  GPIO_WriteBit(S2_PORT, S2_PINS, SET);
  #else  //add i/o control logic to fixed delay stop charging phone about 7~8S
  
   GPIO_WriteBit(S5_PORT, S5_PINS, SET);
   GPIO_WriteBit(S6_PORT, S6_PINS, SET);
   GPIO_WriteBit(S2_PORT, S2_PINS, SET);
   GPIO_WriteBit(S7_PORT, S7_PINS, SET);
   GPIO_WriteBit(TEMP_CHARGE_PORT,TEMP_CHARGE_PINS, RESET);
   
 //  Charge_StateJ2_ControlHandler();
  #endif
 // Set_Led_Indicator_State(STATE_IDLE);
 #ifdef SKY_DEBUG  
       printf("J2 St \r\n");
  #endif
}
/**
  * @brief  read key io value.
  * @param  None
  * @retval key io port value
  * @author: lifubing
  * @date:2017-05-11
  */
static uint8_t Key_IO_Value(void)
{
	uint8_t Key_IO = 0;
	
	Key_IO |= (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == 0)?0x01:0x00;
    /*
	Key_IO |= (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7) == 0)?0x02:0x00;       
	Key_IO |= (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1) == 0)?0x04:0x00;    
	Key_IO |= (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 0)?0x08:0x00;
      */
	return Key_IO;
}

/**
  * @brief  scan press key function.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */

void KeyScan_Task(void)
{

  uint8_t Key_TempValue;
  //uint8_t buf[2] ={0};
  //uint8_t i;
//  uint8_t index = 0;
 if(KeyScandData.KeyScan_flag)
 {
   
 		KeyScandData.KeyScan_flag = FALSE;

         #if 0 //def SKY_DEBUG  
            printf("key scan \r\n");
         #endif
        #if 0  //def SKY_DEBUG 
        #if 1
                  //Charge_Obj_Data.bat_obj.bat_remain_percent = BQ27541_ReadReg(BQ27541CMD_VOLT_LSB,BQ27541CMD_VOLT_MSB);
                 // Charge_Obj_Data.bat_obj.bat_remain_percent = BQ27541_ReadReg(BQ27541CMD_TEMP_LSB,BQ27541CMD_TEMP_MSB);
                   Charge_Obj_Data.bat_obj.bat_remain_percent = BQ27541_ReadReg(BQ27541CMD_SOC_LSB,BQ27541CMD_SOC_MSB);
                 //   Charge_Obj_Data.bat_obj.bat_remain_percent = 0;
                   if( Charge_Obj_Data.bat_obj.bat_remain_percent == 0)
                    {
                       Charge_Obj_Data.bat_obj.bat_level =  BATTERY_LEVEL_1;
                       Set_Led_Indicator_State(BAT_LEVEL_INDICATE);
                    }
                // printf("%d \r\n",Charge_Obj_Data.bat_obj.bat_remain_percent);
                  #else
                                i2c_bq27541_read(buf,BQ27541CMD_VOLT_LSB,2);
                                Charge_Obj_Data.bat_obj.bat_remain_percent = (buf[1]<<8)|buf[0];
                                printf(" bat vol:%d \r\n",Charge_Obj_Data.bat_obj.bat_remain_percent);
                    #endif
        #endif
  		switch(KeyScandData.State)
		  {

			case KEY_PAD_IDLE_STATE:

		        KeyScandData.DebounceCounter = 0;	
			    KeyScandData.Key_CurrentValue = 0;	
				KeyScandData.Key_Value = 0;
                KeyScandData.Key_OldValue =0;
				KeyScandData.WaitKeyReleaseTime = 0;
				KeyScandData.State = KEY_PAD_SCAN_GPIO;
                #if 0 //def SKY_DEBUG  
                    printf("key scan gpio \r\n");
                 #endif
				break;
			case KEY_PAD_SCAN_GPIO:
		        KeyScandData.Key_CurrentValue	= Key_IO_Value();

                 #if 0 //def SKY_DEBUG  
                    printf("get key \r\n");
                 #endif
				if(KeyScandData.Key_CurrentValue)
				{
		           KeyScandData.State = KEY_PAD_DEBOUNCE_STATE;
				}
				else
				{
					KeyScandData.State = KEY_PAD_IDLE_STATE; //KEY_PAD_FINISH;
				}
				break;
			case KEY_PAD_DEBOUNCE_STATE:
				Key_TempValue = Key_IO_Value();
				if(KeyScandData.Key_CurrentValue == Key_TempValue)
				{
					KeyScandData.DebounceCounter++;
					if(KeyScandData.DebounceCounter == KEY_SCAN_DEBOUNCE_PRESS_COUNT)
					{
					  	KeyScandData.State = KEY_PAD_WAIT_TILL_RELEASE;
		               	KeyScandData.DebounceCounter = 0;

                         KeyScandData.Key_NewValue  = n_KEY_SHORT;
					}
				}
				else
				{
					//KeyScandData.State = KEY_PAD_FINISH;
                    KeyScandData.State =KEY_PAD_IDLE_STATE;
				}
				break;
			
			case KEY_PAD_WAIT_TILL_RELEASE:
				Key_TempValue = Key_IO_Value();
 				if(Key_TempValue != KeyScandData.Key_CurrentValue)
 				{
					KeyScandData.DebounceCounter++;
					if(KeyScandData.DebounceCounter == KEY_SCAN_DEBOUNCE_PRESS_COUNT)
					{
						if(Key_TempValue == 0)
						{
							//KeyScandData.State = KEY_PAD_FINISH;
                            KeyScandData.State =KEY_PAD_IDLE_STATE;
                           KeyScandData.Key_Value = KeyScandData.Key_NewValue;
						}
					}
 				}
				else
				{
                      KeyScandData.WaitKeyReleaseTime += 1;
                      #if 1
                      if(KeyScandData.WaitKeyReleaseTime >= LONG_PRESS_KEY_3S_TIMEOUT)
                     {
                         KeyScandData.Key_NewValue  = n_KEY_LONGPRESS_3S;
                        // KeyScandData.State = KEY_PAD_FINISH;
                         KeyScandData.Key_Value = KeyScandData.Key_NewValue;
                        KeyScandData.WaitKeyReleaseTime = 0;
                     }
                     #else
                     
                     if(KeyScandData.WaitKeyReleaseTime >= LONG_PRESS_KEY_3S_TIMEOUT)
                     {
                         KeyScandData.Key_NewValue  = n_KEY_LONGPRESS_3S;
                     }
                     else if(KeyScandData.WaitKeyReleaseTime >= LONG_PRESS_KEY_1S_TIMEOUT)
                     {

                         KeyScandData.Key_NewValue  = n_KEY_LONGPRESS_1S;
                     }
                     #endif
				}
				
				break;
            #if 0
			case KEY_PAD_FINISH:
				
				KeyScandData.DebounceCounter = 0;	
			    KeyScandData.Key_CurrentValue = 0;	
				KeyScandData.Key_Value = 0;
                KeyScandData.Key_OldValue =0;
				KeyScandData.WaitKeyReleaseTime = 0;
				KeyScandData.State = KEY_PAD_IDLE_STATE;
				break;
            
			default:
				break;
            #endif
		  }
  
  		// if((KeyScandData.Key_Value)&&(KeyScandData.State == KEY_PAD_FINISH))
         if((KeyScandData.Key_Value)&&(KeyScandData.Key_Value != KeyScandData.Key_OldValue))
  		{
  		   KeyScandData.Key_OldValue =  KeyScandData.Key_Value;
            #if 0  //def SKY_DEBUG  
                if( KeyScandData.Key_Value == n_KEY_SHORT)
                 {
                  printf("s key\r\n");
                 }
                 else if( KeyScandData.Key_Value == n_KEY_LONGPRESS_1S)
                  printf("long press 1S key\r\n");
                else if( KeyScandData.Key_Value == n_KEY_LONGPRESS_3S)
                  printf("long press 3S key\r\n");
             #endif    
             if(product_test_flag == 1)
             {
                usart_send_data("<OK:key>");
             }
             else
             {
                 COR_AP_Key_Function();
                 if(( KeyScandData.Key_Value == n_KEY_SHORT)&&((Charge_Obj_Data.led_obj.led_indicate_state == STATE_IDLE)||(Charge_Obj_Data.led_obj.led_indicate_state == BAT_CHARGDONE_INDICATE)))
                 {
                    COR_AP_ShorKey_Common_Handler();
                  }
             }
  		}

 	}

}


//******************************************************************************
// Function Name:       void COR_AP_Key_Function(void)                         *
// Function:            Key Function                                           *
// return value:        void                                                   *
//******************************************************************************
void COR_AP_Key_Function(void)
{
 uint8_t i;
 //UINT32 j;
 void (*The_Func)(void) = NULL; // n_NULL; //定义一个函数指针
 //*************************************
 // 查表取得执行按键函数的指针         *
 //*************************************
 The_Func = 0;
// for(i = 0; i < Max_Menu_Key_Func_Number; i++)
 for( i=0 ; i < (sizeof(Menu_ID_to_KeyFunction) / sizeof(Menu_ID_to_KeyFunction[0])) ; i++ )
 {
  if((Charge_Obj_Data.ChargeCtlState == Menu_ID_to_KeyFunction[i].Menu_ID)
   &&(KeyScandData.Key_Value == Menu_ID_to_KeyFunction[i].Key_Value))
  // &&(Key_str.keyCode == Menu_ID_to_KeyFunction[i].Key_Value))
  {
   //Find the Function
   The_Func = Menu_ID_to_KeyFunction[i].Func_Pointer;
   break;
  }
 }
 //j = (UINT32)The_Func;
 //*************************************
 // 执行按键函数                       *
 //*************************************
 if(The_Func != NULL)  //n_NULL
  (*The_Func)();
}





