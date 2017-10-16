/*************************************************************************************
 * Copyright:    Copyright (c) 2016 SkyLight R&D Centre ()
 *
 * File name:    main.c
 *
 * Description:  MCU
 *
 * Author:  lifubing   Version:v2.0.0 Date: 2017.5.20
 *                
 * History:                     
**************************************************************************************/

#include "charge.h"
#include "time.h"
#include "stdio.h"
#include "Keyproc.h"
#include "battery.h"
#include "flash.h"
#include "Charge.h"

//extern unsigned char   iap_flg;
/**
  * @brief  close pherial and enbale wake up source to save power when enter sleep .
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Enter_halt_mode_init(void)
{

   // Enter_FullSleepMode();
  // GPIO_Init(PRESS_KEY_PORT, PRESS_KEY_PORT_PINS, GPIO_Mode_In_PU_IT);
    GPIO_Init(PRESS_KEY_PORT, PRESS_KEY_PORT_PINS, GPIO_Mode_In_FL_IT);
 //  GPIO_Init(WIRLESS_DETECT_PORT, WIRLESS_DETECT_PINS, GPIO_Mode_In_PU_No_IT);
   GPIO_Init(WAKE_UP_PORT, WAKE_UP_PINS, GPIO_Mode_In_FL_IT);

 //  GPIO_Init(BAT_SCK,BAT_SCK_PINS,GPIO_Mode_In_FL_No_IT);  //GPIO_Mode_Out_PP_High_Slow//GPIO_Mode_Out_PP_High_Fast
 //  GPIO_Init(BAT_SDA,BAT_SDA_PINS,GPIO_Mode_In_FL_No_IT);  //GPIO_Mode_Out_PP_High_Slow
 //  Charge_StateL_ControlHandler();
   //Charge_Default_ControlHandler();
  //GPIO_Init(WAKE_UP_PORT, WAKE_UP_PINS, GPIO_Mode_In_PU_IT);
  // EXTI_SetPinSensitivity(EXTI_Pin_4,EXTI_Trigger_Falling);        // unmask it that can wake up but key not response.
 //  EXTI_SetPinSensitivity(EXTI_Pin_5,EXTI_Trigger_Rising_Falling); //camera  stream detect interrupt
   EXTI->SR1 = 0xFF; /* Setting SR1 bits in order to clear flags */
   EXTI->SR2 = 0xFF; /* Setting SR2 bits in order to clear flags */
  TIM4_DeInit();
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, DISABLE);
 //  CLK_PeripheralClockConfig(CLK_Peripheral_USART1, DISABLE);
   ADC_DeInit(ADC1); 
   CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, DISABLE); 

   I2C_DeInit(I2C1);
   CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, DISABLE);

  // USART_DeInit(USART1);
  //  GPIO_Init(GPIOA,GPIO_Pin_3, GPIO_Mode_In_FL_IT);
 // GPIO_Init(USART_RX_PORT,USART_RX_PINS,GPIO_Mode_Out_PP_Low_Slow);
   
}

/**
  * @brief  reinit pherial  when exit sleep .
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Exit_halt_mode_reinit(void)
{
    GPIO_Init(PRESS_KEY_PORT, PRESS_KEY_PORT_PINS, GPIO_Mode_In_FL_No_IT);
    GPIO_Init(WAKE_UP_PORT, WAKE_UP_PINS, GPIO_Mode_In_FL_No_IT);
  //  Charge_Init_USART();
    //init Timer
    Charge_Init_Timer();
    Charge_Init_ADC();
    Charge_Init_I2C();
   
}
/**
  * @brief  check enter sleep according to state .
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Check_Enter_Sleep(void)
{
   if(((CHARGE_STATE_DEFAULT ==Charge_Obj_Data.ChargeCtlState)
      ||(CHARGE_STATE_H ==Charge_Obj_Data.ChargeCtlState)
      )&&(KeyScandData.State == KEY_PAD_IDLE_STATE)
       &&(Charge_Obj_Data.led_obj.led_indicate_state == STATE_IDLE)
       &&(0 == Charge_Obj_Data.usart_obj.usart_rx_flg)
       &&(0 == Charge_Obj_Data.check_switch_state_time )
       &&(product_test_flag == 0)
      )
   {
      #ifdef SKY_DEBUG  
            printf("Enter halt\r\n");
      #endif
      Enter_halt_mode_init();
      halt();
      Exit_halt_mode_reinit();
       #ifdef SKY_DEBUG  
            printf("Exit halt\r\n");
      #endif
   }
   else
    {
       wfi();
    }

}

#if 0 //def  USE_FULL_ASSERT   //SKY_DEBUG
void assert_failed(uint8_t* file, uint32_t line)
{

   printf("filename:%s, line:%d", file, line);
   while(1);
 //  printf("filename:%s, line:%d\r\n", __FILE__, __LINE__);

}
#endif


void main(void)
{
   // uint16_t i = 0;
    unsigned char   clr_iap_flg  = 0;
    //init Clock
    Charge_Init_Clock();//
    
    //init GPIO
    Charge_Init_GPIO();
  
    //init Timer
    Charge_Init_Timer();

    // init I2C
    Charge_Init_I2C();
    //ADC
    Charge_Init_ADC();

    //init USART
    Charge_Init_USART();

    //init interrupt
    Charge_Init_Interrupt();


    //init iwdog
    #ifdef WDOG_EN
   // Charge_Init_iWatchdog();
     //init wdog
     WWDG_Init(0x7F, 0x7F);
    #endif
    //init Flash
    Charge_Init_Flash();

    
    enableInterrupts(); 


    //init Led
    Charge_Init_Led();

    //init user
    Charge_Init_User();

  
    #ifdef SKY_DEBUG  
    printf("up\r\n");
   // COR_AP_ShorKey_Common_Handler();
    #endif
   // assert_failed(__FILE__,__LINE__);
    /*
    Charge_Obj_Data.bat_obj.bat_remain_percent = BQ27541_ReadReg(BQ27541CMD_SOC_LSB,BQ27541CMD_SOC_MSB);
   // Charge_Obj_Data.bat_obj.bat_remain_percent = 35;
    #ifdef SKY_DEBUG  
    printf("\r\n Bat:%d \r\n",Charge_Obj_Data.bat_obj.bat_remain_percent);
    #endif
    */
   // Send_MCU_Shutdown_cmd();
   Write_Flash_Byte(FLASH_MemType_Data, IAP_ADDRESS, 1 , &clr_iap_flg);
  //  Charge_StateJ2_ControlHandler();
  //  Judge_J_State();
    while (1)  
    {
      KeyScan_Task();
      usart_dealing_func();
      Check_ChargeControl_State();
      Charge_State_Contrl_Handler();
      Check_Led_State();
    //  Charge_Uart_Cmd_Handler();
      Product_Test_Led_Handler();
       #ifdef WDOG_EN
      //IWDG_ReloadCounter();
       WWDG_SetCounter(0x7f);
       #endif
      Check_Enter_Sleep();
	 
      #if 0
	  if( iap_flg  == IAP_VALUE_FLG)
      {
       Led_ALL_ON();
       for(i = 0; i<60000; i++)
        ;
        asm("LDW X, SP");
        asm("LD  A, $FF");   
        asm("LD  XL, A");
        asm("LDW SP, X");
        asm("JPF $8000");
       }
      #endif
    }  
}


