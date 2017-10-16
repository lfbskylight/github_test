/*************************************************************************************
 * Copyright:    Copyright (c) 2016 SkyLight R&D Centre ()
 *
 * File name:    led.c
 *
 * Description:  MCU
 *
 * Author:  lifubing   Version:v2.0.0 Date: 2017.4.13
 *                
 * History:                     
**************************************************************************************/
#include "led.h"
#include "time.h"
#include "charge.h"
#include "Battery.h"
#include "stdio.h"

static void Led_Flash(void);
static void Led_Roll(void);
void Led_ALL_OFF(void)
{

    GPIO_WriteBit(LED1_PORT, LED1_PINS, RESET);
    GPIO_WriteBit(LED2_PORT, LED2_PINS, RESET);
    GPIO_WriteBit(LED3_PORT, LED3_PINS, RESET);
    GPIO_WriteBit(LED4_PORT, LED4_PINS, RESET);
}
void Led_ALL_ON(void)
{

    GPIO_WriteBit(LED1_PORT, LED1_PINS, SET);
    GPIO_WriteBit(LED2_PORT, LED2_PINS, SET);
    GPIO_WriteBit(LED3_PORT, LED3_PINS, SET);
    GPIO_WriteBit(LED4_PORT, LED4_PINS, SET);
}
/**
  * @brief  indicate led acording to battery level.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Battery_Level_Led_Indicator(BATTERY_LEVEL level)
{
   switch(level)
    {
     case BATTERY_LEVEL_1:
            GPIO_WriteBit(LED4_PORT, LED4_PINS, SET);
            GPIO_WriteBit(LED3_PORT, LED3_PINS, RESET);
            GPIO_WriteBit(LED2_PORT, LED2_PINS, RESET);
            GPIO_WriteBit(LED1_PORT, LED1_PINS, RESET);
        break;
     case BATTERY_LEVEL_2:
             GPIO_WriteBit(LED4_PORT, LED4_PINS, SET);
             GPIO_WriteBit(LED3_PORT, LED3_PINS, SET);
             GPIO_WriteBit(LED2_PORT, LED2_PINS, RESET);
             GPIO_WriteBit(LED1_PORT, LED1_PINS, RESET);
        break;
     case BATTERY_LEVEL_3:
            GPIO_WriteBit(LED4_PORT, LED4_PINS, SET);
            GPIO_WriteBit(LED3_PORT, LED3_PINS, SET);
            GPIO_WriteBit(LED2_PORT, LED2_PINS, SET);
            GPIO_WriteBit(LED1_PORT, LED1_PINS, RESET);
        break;
     case BATTERY_LEVEL_4:
            Led_ALL_ON();
        break;
     case BATTERY_LEVEL_FULL:
         if(Charge_Obj_Data.led_obj.led_indicate_state == BAT_LEVEL_INDICATE)
            Led_ALL_ON();
           else
            Led_ALL_OFF();
          break;
      default:
            Led_ALL_OFF();
        break;
    }
   
}

/**
  * @brief  Set led state according to battery  level 
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Set_Led_Indicator_State(LED_indication_t state)
{
     switch (state)
    {

     case BAT_LEVEL_INDICATE:
           Charge_Obj_Data.led_obj.led_3s_off_time = 0;
           Charge_Obj_Data.led_obj.led_indicate_state  =  state;
           Battery_Level_Led_Indicator(Charge_Obj_Data.bat_obj.bat_level);
        break;
     case BAT_CHARGING_INDICATE:
           Charge_Obj_Data.led_obj.led_flash_time  = 0;
           Charge_Obj_Data.led_obj.led_indicate_state  =  state;
           Set_Led_Flash_State(Charge_Obj_Data.bat_obj.bat_level);
        break;
     
     case BAT_CHARGDONE_INDICATE:
          Charge_Obj_Data.led_obj.led_indicate_state  =  state;
        break;

    /*
     case LOW_BAT_INDICATE:
        
           Charge_Obj_Data.led_obj.led_flash_time  = 0;
           Charge_Obj_Data.led_obj.led_indicate_state  =  state;
           Battery_Level_Led_Indicator(Charge_Obj_Data.bat_obj.bat_level);
           Set_Led_Flash_State(Charge_Obj_Data.bat_obj.bat_level);
        break;
      */
    case BAT_ROLL_INDICATE:
          Charge_Obj_Data.led_obj.led_indicate_state = state;
          Charge_Obj_Data.led_obj.led_roll_state  = 0;
       //   Charge_Obj_Data.led_obj.led_flash_time =LED_ROLL_INTERVAL ;
     break;
     case  STATE_IDLE:
            Charge_Obj_Data.led_obj.led_indicate_state  =  state;
        break;
      default:
        break;
    }  
    
}

/**
  * @brief  Set led state according to battery  level 
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
  
void Set_Led_Flash_State(BATTERY_LEVEL level)
{

  switch(level)
    {
     case BATTERY_LEVEL_0:
            Charge_Obj_Data.led_obj.led_flash_state  =  LED4_FLASH; //LED1_FLASH;
       break;
     case BATTERY_LEVEL_1:
            Charge_Obj_Data.led_obj.led_flash_state  =  LED4_FLASH; //LED4_ON_LED3_FLASH;
        break;
     case BATTERY_LEVEL_2:
            Charge_Obj_Data.led_obj.led_flash_state  =  LED4_ON_LED3_FLASH; //LED4_LED3_ON_LED2_FLASH;
        break;
     case BATTERY_LEVEL_3:
            Charge_Obj_Data.led_obj.led_flash_state  =  LED4_LED3_ON_LED2_FLASH; // LED4_LED3_LED2_ON_LED1_FLASH;
        break;
     case BATTERY_LEVEL_4:
            Charge_Obj_Data.led_obj.led_flash_state  =  LED4_LED3_LED2_ON_LED1_FLASH;
        break;
      case BATTERY_LEVEL_FULL:
           Charge_Obj_Data.led_obj.led_flash_state  =  LED_ALL_ON;
        break;
      default:
        break;
    }
}

/**
  * @brief  battery led indicator 3S time out handler
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
static void Battery_Led_Indicator_3S_Timeout_handler()
{
  switch(Charge_Obj_Data.ChargeCtlState)
  {
     case CHARGE_STATE_H:
     case CHARGE_STATE_H_Y:
     case CHARGE_STATE_D:
     case CHARGE_STATE_DEFAULT:
     case CHARGE_STATE_L:
     case CHARGE_STATE_J1:
            Set_Led_Indicator_State(STATE_IDLE);
            Led_ALL_OFF();
           break;
     
     default:
           Set_Led_Indicator_State(BAT_CHARGDONE_INDICATE);
            Led_ALL_OFF();
           break;
     #if 0
     case CHARGE_STATE_J1:
     case CHARGE_STATE_J2:
     case CHARGE_STATE_J3:
            Charge_Obj_Data.current_det_adc   =  Current_Detect_Get_Vol();
            if(Charge_Obj_Data.current_det_adc <PHONE_CHARGE_FULL_ADC_MAX_VALUE)
             {
                Set_Led_Indicator_State(BAT_CHARGING_INDICATE);
             }
            else
             {
                Set_Led_Indicator_State(STATE_IDLE);
                Led_ALL_OFF();
             }
      break;
    
    case CHARGE_STATE_E:
    case CHARGE_STATE_M:
    case CHARGE_STATE_N:
    case CHARGE_STATE_F:
        Set_Start_Charging_indicator();
        break;
   
   // case CHARGE_STATE_J2:
           break;
   
     default:
            ;
          // Led_ALL_OFF();
           //Set_Led_Indicator_State(BAT_CHARGING_INDICATE);
     break;
      #endif

   }
}


/**
  * @brief  check led state 
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Check_Led_State (void)
{
   LED_indication_t  state;
   if(product_test_flag == 1)
     return;
   state  = Charge_Obj_Data.led_obj.led_indicate_state;
    switch (state)
    {

     case BAT_LEVEL_INDICATE:
           if(BATTERY_LEVEL_3S_TIMEOUT == Charge_Obj_Data.led_obj.led_3s_off_time)
           {
              Battery_Led_Indicator_3S_Timeout_handler();
           }
        break;
     case BAT_CHARGING_INDICATE:
           if(LED_FLASH_INTERVAL == Charge_Obj_Data.led_obj.led_flash_time)
           {
                  Charge_Obj_Data.led_obj.led_flash_time = 0;
                  Led_Flash();
           }
        break;
     /*
     case BAT_CHARGDONE_INDICATE:
        break;
     
     case LOW_BAT_INDICATE:
        if(LED_FLASH_INTERVAL == Charge_Obj_Data.led_obj.led_flash_time)
           {
              Charge_Obj_Data.led_obj.led_flash_time = 0;
              Led_Flash();
           }
        break;
        */
    case BAT_CHARGDONE_INDICATE:
    case STATE_IDLE:
            Led_ALL_OFF();
        break;
     case BAT_ROLL_INDICATE:
       if(LED_ROLL_INTERVAL <= Charge_Obj_Data.led_obj.led_flash_time)
       {
             Charge_Obj_Data.led_obj.led_flash_time = 0;
              Led_Roll();
       }
       break;
    }
}


/**
  * @brief flash led  
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
static void Led_Roll(void)
{
   if(Charge_Obj_Data.led_obj.led_roll_state < Charge_Obj_Data.bat_obj.bat_level)
    {
       switch(Charge_Obj_Data.led_obj.led_roll_state)
        {
        case 0:
            GPIO_WriteBit(LED4_PORT, LED4_PINS, SET);
            GPIO_WriteBit(LED3_PORT, LED3_PINS, RESET);
            GPIO_WriteBit(LED2_PORT, LED2_PINS, RESET);
            GPIO_WriteBit(LED1_PORT, LED1_PINS, RESET);
            break;
        case 1:
            GPIO_WriteBit(LED4_PORT, LED4_PINS, SET);
            GPIO_WriteBit(LED3_PORT, LED3_PINS, SET);
            GPIO_WriteBit(LED2_PORT, LED2_PINS, RESET);
            GPIO_WriteBit(LED1_PORT, LED1_PINS, RESET);
            break;
        case 2:
            GPIO_WriteBit(LED4_PORT, LED4_PINS, SET);
            GPIO_WriteBit(LED3_PORT, LED3_PINS, SET);
            GPIO_WriteBit(LED2_PORT, LED2_PINS, SET);
            GPIO_WriteBit(LED1_PORT, LED1_PINS, RESET);
            break;
        case 3:
            /*
            GPIO_WriteBit(LED4_PORT, LED4_PINS, SET);
            GPIO_WriteBit(LED3_PORT, LED3_PINS, SET);
            GPIO_WriteBit(LED2_PORT, LED2_PINS, SET);
            GPIO_WriteBit(LED1_PORT, LED1_PINS, SET);
            */
            Led_ALL_ON();
            break;
        default:
            break;
        }
        Charge_Obj_Data.led_obj.led_roll_state++;
    }
    else
    {
        Set_Led_Indicator_State(STATE_IDLE);
    }
}
/**
  * @brief flash led  
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
static void Led_Flash(void)
{
    switch(Charge_Obj_Data.led_obj.led_flash_state )
    {
       #if 1
        case LED4_FLASH:
            GPIO_ToggleBits(LED4_PORT,LED4_PINS);     
        break;

        case LED4_ON_LED3_FLASH:
            GPIO_ToggleBits(LED3_PORT,LED3_PINS);     
        break;

        case LED4_LED3_ON_LED2_FLASH:
            GPIO_ToggleBits(LED2_PORT,LED2_PINS);     
        break;

        case LED4_LED3_LED2_ON_LED1_FLASH:
            GPIO_ToggleBits(LED1_PORT,LED1_PINS);     
        break;
       #else
        case LED1_FLASH:
            GPIO_ToggleBits(LED1_PORT,LED1_PINS);     
        break;

        case LED1_ON_LED2_FLASH:
            GPIO_ToggleBits(LED2_PORT,LED2_PINS);     
        break;

        case LED1_LED2_ON_LED3_FLASH:
            GPIO_ToggleBits(LED3_PORT,LED3_PINS);     
        break;

        case LED1_LED2_LED3_ON_LED4_FLASH:
            GPIO_ToggleBits(LED4_PORT,LED4_PINS);     
        break;
        #endif
        default:
         Led_ALL_OFF();
        break;
    }
}


void Check_Charge_Led_Flash_State(void)
{
   if(Charge_Obj_Data.bat_obj.bat_det_time >= BATTERY_CHARGE_LED_TIME)
    {
        #if 0 //def SKY_DEBUG  
            printf("time 5Min charging check\r\n");
        #endif
        Charge_Obj_Data.bat_obj.bat_det_time  = 0;
        Get_Battery_Percent();
        Charge_Obj_Data.bat_obj.bat_level =  Battery_Get_Vol_Level(Charge_Obj_Data.bat_obj.bat_remain_percent);
        Battery_Level_Led_Indicator(Charge_Obj_Data.bat_obj.bat_level);
        if(BATTERY_LEVEL_FULL == Charge_Obj_Data.bat_obj.bat_level)
            Set_Led_Indicator_State(BAT_CHARGDONE_INDICATE);
       else
            Set_Led_Indicator_State(BAT_CHARGING_INDICATE);
    }

}




