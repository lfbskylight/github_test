/**
  ******************************************************************************
  * @file    Project/STM8L15x_StdPeriph_Template/stm8l15x_it.c
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    30-September-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8l15x_it.h"
//#include <stdio.h>
#include "battery.h"
#include "led.h"
#include "charge.h"
#include "help.h"
#include "string.h"
#include "Keyproc.h"
//#include "product_test.h"
/** @addtogroup STM8L15x_StdPeriph_Template
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

#ifdef _COSMIC_
/**
  * @brief Dummy interrupt routine
  * @par Parameters:
  * None
  * @retval 
  * None
*/
INTERRUPT_HANDLER(NonHandledInterrupt,0)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
#endif

/**
  * @brief TRAP interrupt routine
  * @par Parameters:
  * None
  * @retval 
  * None
*/
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief FLASH Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(FLASH_IRQHandler,1)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief DMA1 channel0 and channel1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(DMA1_CHANNEL0_1_IRQHandler,2)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief DMA1 channel2 and channel3 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(DMA1_CHANNEL2_3_IRQHandler,3)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief RTC / CSS_LSE Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(RTC_CSSLSE_IRQHandler,4)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief External IT PORTE/F and PVD Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTIE_F_PVD_IRQHandler,5)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PORTB / PORTG Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTIB_G_IRQHandler,6)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PORTD /PORTH Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTID_H_IRQHandler,7)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PIN0 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI0_IRQHandler,8)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    Charge_Obj_Data.state_switch_flag = TRUE;
    Charge_Obj_Data.check_switch_state_time = SWITCH_STATE_TIMEOUT;
    if (GPIO_ReadInputDataBit(MCU_DSP_SING_PORT,MCU_DSP_SING_PINS) == RESET) // camera plug out
        Charge_Obj_Data.camera_on_off_flag = FALSE;
    Charge_Obj_Data.IO_First_State  = ((GPIO_ReadInputDataBit(PHONE_DETECT_PORT,PHONE_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(USB_DETECT_PORT,USB_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(WIRLESS_DETECT_PORT,WIRLESS_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(MCU_DSP_SING_PORT, MCU_DSP_SING_PINS))
                                          );
     EXTI_ClearITPendingBit(EXTI_IT_Pin0);
}

/**
  * @brief External IT PIN1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI1_IRQHandler,9)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
     Charge_Obj_Data.state_switch_flag = TRUE;
    Charge_Obj_Data.check_switch_state_time = SWITCH_STATE_TIMEOUT;
    #if 0 //def SKY_DEBUG  
    printf("w int \r\n");
    #endif
    Charge_Obj_Data.IO_First_State  = ((GPIO_ReadInputDataBit(PHONE_DETECT_PORT,PHONE_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(USB_DETECT_PORT,USB_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(WIRLESS_DETECT_PORT,WIRLESS_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(MCU_DSP_SING_PORT, MCU_DSP_SING_PINS))
                                          );
    EXTI_ClearITPendingBit(EXTI_IT_Pin1);
}

/**
  * @brief External IT PIN2 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI2_IRQHandler,10)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
   
    Charge_Obj_Data.state_switch_flag = TRUE;
    Charge_Obj_Data.check_switch_state_time = 20; //SWITCH_STATE_TIMEOUT; //20; //; //;
     EXTI_ClearITPendingBit(EXTI_IT_Pin2);
      
    disableInterrupts() ;
  #if 1
    if (GPIO_ReadInputDataBit(USB_DETECT_PORT,USB_DETECT_PINS) == RESET) // plug-in
    {
       GPIO_Init(WIRLESS_DETECT_PORT, WIRLESS_DETECT_PINS, GPIO_Mode_In_FL_No_IT); // 
    
        EXTI->CR1 &=  (uint8_t)(~EXTI_CR1_P1IS);
    }
    else
    {
     GPIO_Init(WIRLESS_DETECT_PORT, WIRLESS_DETECT_PINS, GPIO_Mode_In_FL_IT); // OK 
     EXTI_SetPinSensitivity(EXTI_Pin_1,EXTI_Trigger_Rising_Falling); 
    }
    
   #endif
   Charge_Obj_Data.IO_First_State  = ((GPIO_ReadInputDataBit(PHONE_DETECT_PORT,PHONE_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(USB_DETECT_PORT,USB_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(WIRLESS_DETECT_PORT,WIRLESS_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(MCU_DSP_SING_PORT, MCU_DSP_SING_PINS))
                                          );
   
    enableInterrupts(); 
   #if 0 //def SKY_DEBUG 
        printf("usb \r\n");
   #endif
}

/**
  * @brief External IT PIN3 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI3_IRQHandler,11)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    EXTI_ClearITPendingBit(EXTI_IT_Pin3);
}

void Delay_Time(__IO uint16_t nCount)
{
    /* Decrement nCount value */
    while (nCount != 0)
    {
        nCount--;
    }
}


/**
  * @brief External IT PIN4 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI4_IRQHandler,12)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */ 

   EXTI_ClearITPendingBit(EXTI_IT_Pin4);

  //Charge_Obj_Data.state_switch_flag = TRUE;
  #if 0
   if (GPIO_ReadInputDataBit(PRESS_KEY_PORT,PRESS_KEY_PORT_PINS) == RESET) // falling
    {
     #ifdef SKY_DEBUG 
        printf("fall\r\n");
     #endif
    }
   else
    {
      #ifdef SKY_DEBUG 
        printf("rise\r\n");
       #endif
    }
   #endif
   #if 0
   charge_mgr.charge_btn.btn_time_wait = 0;
   charge_mgr.charge_btn.btn_time_long = 0;
   //charge_mgr.charge_btn.btn_it     = 1;
   charge_mgr.charge_obj.act_charge |= CHARGE_ACT_BTN_PRESS;
   #endif
}

/**
  * @brief External IT PIN5 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI5_IRQHandler,13)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    /*
    Charge_Obj_Data.state_switch_flag = TRUE;
    Charge_Obj_Data.check_switch_state_time = SWITCH_STATE_TIMEOUT;
    Charge_Obj_Data.IO_First_State  = ((GPIO_ReadInputDataBit(PHONE_DETECT_PORT,PHONE_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(USB_DETECT_PORT,USB_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(WIRLESS_DETECT_PORT,WIRLESS_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(MCU_DSP_SING_PORT, MCU_DSP_SING_PINS))
                                          );
       */
    #if 0 //def SKY_DEBUG  
    printf("w int \r\n");
    #endif
    EXTI_ClearITPendingBit(EXTI_IT_Pin5);
}

/**
  * @brief External IT PIN6 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI6_IRQHandler,14)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    #if 1
    Charge_Obj_Data.state_switch_flag = TRUE;
    Charge_Obj_Data.check_switch_state_time = SWITCH_STATE_TIMEOUT;
    Charge_Obj_Data.IO_First_State  = ((GPIO_ReadInputDataBit(PHONE_DETECT_PORT,PHONE_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(USB_DETECT_PORT,USB_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(WIRLESS_DETECT_PORT,WIRLESS_DETECT_PINS))
                                           |(GPIO_ReadInputDataBit(MCU_DSP_SING_PORT, MCU_DSP_SING_PINS))
                                          );
    #else
    if (0 == GPIO_ReadInputDataBit(MCU_DSP_SING_PORT, MCU_DSP_SING_PINS))
    {
        Charge_Vr_Plug_Out();
    }
    else
    {
        Charge_Vr_Plug_In();
    }
    #endif
    EXTI_ClearITPendingBit(EXTI_IT_Pin6);
    
}

/**
  * @brief External IT PIN7 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI7_IRQHandler,15)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief LCD /AES Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(LCD_AES_IRQHandler,16)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief CLK switch/CSS/TIM1 break Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(SWITCH_CSS_BREAK_DAC_IRQHandler,17)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief ADC1/Comparator Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(ADC1_COMP_IRQHandler,18)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief TIM2 Update/Overflow/Trigger/Break /USART2 TX Interrupt routine.
  * @param  None
  * @retval None
  */
//static unsigned int time2_flg = 0;
INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler,19)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */

    //TIM4_ClearITPendingBit(TIM2_IT_Update);
}

/**
  * @brief Timer2 Capture/Compare / USART2 RX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM2_CC_USART2_RX_IRQHandler,20)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}


/**
  * @brief Timer3 Update/Overflow/Trigger/Break Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQHandler,21)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief Timer3 Capture/Compare /USART3 RX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM3_CC_USART3_RX_IRQHandler,22)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief TIM1 Update/Overflow/Trigger/Commutation Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_COM_IRQHandler,23)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief TIM1 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_CC_IRQHandler,24)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief TIM4 Update/Overflow/Trigger Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM4_UPD_OVF_TRG_IRQHandler,25)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
       
    */    
  //  static uint8_t cnt = 0;
 //    ++cnt;
  //  if(cnt == 8)  //16Mhz  // 4  8Mhz
   // {
    //    cnt = 0;
        KeyScandData.KeyScan_flag = TRUE;
        Charge_Obj_Data.led_obj.led_3s_off_time++;
        Charge_Obj_Data.led_obj.led_flash_time ++;
         Charge_Obj_Data.I2C_timeout ++;
        Charge_Obj_Data.bat_obj.bat_det_time++;
        Charge_Obj_Data.bat_obj.bat_discharge_time ++;
        #if 0
        if(Charge_Obj_Data.bat_obj.bat_discharge_time >(CHECK_BATTERY_DISCHARGE_TIME+5))
        {
            Charge_Obj_Data.bat_obj.bat_discharge_time = 0;
        }
        #endif
        Charge_Obj_Data.check_phone_fullcharge_time ++;
        /*
        if(Charge_Obj_Data.check_phone_fullcharge_time >(CHECK_PHONE_FULL_CHARGE_TIME+5))
        {
            Charge_Obj_Data.check_phone_fullcharge_time = 0;
        }
        */
        Charge_Obj_Data.check_fastcharge_time ++;
        /*
        if(Charge_Obj_Data.check_fastcharge_time >(CHECK_FASTCHARGE_TIME+5))
        {
            Charge_Obj_Data.check_fastcharge_time = 0;
        }
        */
        Charge_Obj_Data.KEY_timeout++;
        if(Charge_Obj_Data.usart_obj.usart_rx_time)
            Charge_Obj_Data.usart_obj.usart_rx_time --;

        if(Charge_Obj_Data.check_switch_state_time)
            Charge_Obj_Data.check_switch_state_time --;
        if(Charge_Obj_Data.check_OTG_Delay_TimeOut)
            Charge_Obj_Data.check_OTG_Delay_TimeOut --;
        //if(Charge_Obj_Data.check_Wirless_S1S2_Delay_time)
        //    Charge_Obj_Data.check_Wirless_S1S2_Delay_time--;

        if(Charge_Obj_Data.check_BatTemp_TimeOut)
            Charge_Obj_Data.check_BatTemp_TimeOut--;

   // }
    TIM4_ClearITPendingBit(TIM4_IT_Update);
}
/**
  * @brief SPI1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(SPI1_IRQHandler,26)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief USART1 TX / TIM5 Update/Overflow/Trigger/Break Interrupt  routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQHandler,27)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    #if 0
    if(USART_GetITStatus(USART1,USART_IT_TXE) == SET)
     {
             if (USART_DATA_COMMUN == Charge_Obj_Data.usart_obj.usart_type)
             {
                UART1_SendStrLen(Charge_Obj_Data.usart_obj.usart_tx_buf, 
                                 Charge_Obj_Data.usart_obj.usart_buf_len);
             }
             else if (USART_DATA_PRO_TEST == Charge_Obj_Data.usart_obj.usart_type)
             {
                UART1_SendStr(Charge_Obj_Data.usart_obj.usart_tx_buf);
                UART1_SendByte(0x0D);
                UART1_SendByte(0x0A);
             }

             memset(Charge_Obj_Data.usart_obj.usart_tx_buf, 0x00, sizeof(Charge_Obj_Data.usart_obj.usart_rx_buf));
           //  memset(Charge_Obj_Data.usart_obj.usart_rx_buf, 0x00, sizeof(Charge_Obj_Data.usart_obj.usart_rx_buf));
          //   Charge_Obj_Data.usart_obj.buf_index     = 0;
           //  Charge_Obj_Data.usart_obj.usart_rx_flg  = 0;
            // Charge_Obj_Data.usart_obj.usart_buf_len = 0;
             
             USART_ITConfig (USART1,USART_IT_TXE,DISABLE);
   }
   #endif
}

/**
  * @brief USART1 RX / Timer5 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(USART1_RX_TIM5_CC_IRQHandler,28)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    unsigned char tmp = 0;
    
    USART_ClearITPendingBit(USART1,USART_IT_RXNE);   
    tmp = USART_ReceiveData8(USART1);
    Charge_Obj_Data.usart_obj.usart_rx_buf[Charge_Obj_Data.usart_obj.rx_write] = tmp;
    if(++Charge_Obj_Data.usart_obj.rx_write == USART_RX_BUF_LEN)
    {
        Charge_Obj_Data.usart_obj.rx_write = 0;
    }
    Charge_Obj_Data.usart_obj.usart_rx_time = UART_IDLE_TIME;
    Charge_Obj_Data.usart_obj.usart_rx_flg  = 1;
    Charge_Obj_Data.check_switch_state_time  = ENTER_SLEEP_TIMEOUT;
    #if 0
    if((tmp == 0xfe) && (Charge_Obj_Data.usart_obj.usart_pack_start_flag == 0))
    {
        Charge_Obj_Data.usart_obj.usart_pack_start_flag = 1;
     //   Charge_Obj_Data.usart_obj.buf_index  = Charge_Obj_Data.usart_obj.rx_write - 1;
       // return ;
    }
    else if(Charge_Obj_Data.usart_obj.usart_pack_start_flag == 1)
    {  
        if(tmp == 0xef)
        {
            /*清一包数据开始标志，数据接受完毕*/
            Charge_Obj_Data.usart_obj.usart_pack_start_flag   = 0;
          //  Charge_Obj_Data.usart_obj.usart_pack_dealing_flag = 1;  /*置数据处理标志,在whilw()循环中去处理*/

            Charge_Obj_Data.usart_obj.usart_pack_dealing_flag ++; 
        }
    }
    #endif
}

/**
  * @brief I2C1 / SPI2 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(I2C1_SPI2_IRQHandler,29)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    I2C_ClearITPendingBit(I2C1, I2C_IT_TIMEOUT);
    #if 0
     /* Read SR2 register to get I2C error */
  if (I2C_ReadRegister(I2C1, I2C_Register_SR2))
  {
    /* Clears SR2 register */
    I2C1->SR2 = 0;

    /* Set LED2 */
   // STM_EVAL_LEDOn(LED2);
  }

  switch (I2C_GetLastEvent(I2C1))
  {
      /* EV5 */
    case I2C_EVENT_MASTER_MODE_SELECT :   //start generation event 

      /* Send slave Address for write */
      I2C_Send7bitAddress(I2C1, BQ27541_ADDR, I2C_Direction_Transmitter);
      break;

      /* EV6 */
    case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:  //addr sent event
      if (NumOfBytes != 0)
      {
        /* Send the first Data */
        I2C_SendData(I2C1, TxBuffer[Tx_Idx++]);
        /* Decrement number of bytes */
        NumOfBytes--;
      }
      if (NumOfBytes == 0)
      {
        I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
      }
      break;

      /* EV8 */
    case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
      /* Transmit Data */
      I2C_SendData(I2C1, TxBuffer[Tx_Idx]);
      Tx_Idx++;
      /* Decrement number of bytes */
      NumOfBytes--;
      if (NumOfBytes == 0)
      {
        I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
      }
      break;

      /* EV8_2 */
    case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
      /* Read DR register to clear BTF Flag */
      I2C1->DR;
      if (NumOfBytes == 0)
      {
        /* Send STOP condition */
        I2C_GenerateSTOP(I2C1, ENABLE);
        Tx_Idx = 0;
        I2C_ITConfig(I2C1, I2C_IT_EVT, DISABLE);
      }
      break;
     // receive data byte
       /* Check on EV2*/
    case I2C_EVENT_SLAVE_BYTE_RECEIVED:
      Slave_Buffer_Rx[Rx_Idx++] = I2C_ReceiveData(I2C1);
      break;

      /* Check on EV4 */
    case (I2C_EVENT_SLAVE_STOP_DETECTED):
            /* write to CR2 to clear STOPF flag */
            I2C1->CR2 |= I2C_CR2_ACK;
      Rx_Idx = 0;
      CommunicationEnd = 0x01;
      break;
    default:
      break;
  }
  #endif
}
/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
