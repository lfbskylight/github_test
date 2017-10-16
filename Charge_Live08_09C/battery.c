/*************************************************************************************
 * Copyright:    Copyright (c) 2016 SkyLight R&D Centre ()
 *
 * File name:    battery.c
 *
 * Description:  MCU
 *
 * Author:  lifubing   Version:v2.0.0 Date: 2017.5.13
 *                
 * History:                     
**************************************************************************************/
#include "Battery.h"
#include "charge.h"

//const uint8_t  Battery_Percent[6] = {0,25,50,75,100,100};


/**
  * @brief  init I2C .
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Charge_Init_I2C(void)
{
  /* I2C  clock Enable*/
  CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, ENABLE);
//  Delay(400);
  #if 1
  /* Initialize I2C peripheral */
  I2C_Init(I2C1, I2C_SPEED, 0xA0,
           I2C_Mode_I2C, I2C_DutyCycle_2,
           I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);

  /* Enable Buffer and Event Interrupt*/
  //I2C_ITConfig(I2C1, (I2C_IT_TypeDef)(I2C_IT_EVT | I2C_IT_BUF) , ENABLE);
 //  I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
  #else
    GPIO_Init(GPIOC,GPIO_Pin_0 | GPIO_Pin_1,GPIO_Mode_Out_PP_High_Slow);
    CLK_PeripheralClockConfig(CLK_Peripheral_I2C1,ENABLE);
    I2C_Init(I2C1, 100000, 0,I2C_Mode_I2C,I2C_DutyCycle_2,I2C_Ack_Enable,I2C_AcknowledgedAddress_7bit); //40000
    I2C_Cmd(I2C1,ENABLE);
  #endif
}

#if 0
void i2c_bq27541_read(uint8_t *buff,uint8_t cmd,uint8_t bytes)
{
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
   // I2C_AcknowledgeConfig(I2C1, ENABLE); //fbli add
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C1, BQ27541_ADDR, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C1, cmd);
    //while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));  //fbli modify

    I2C_GenerateSTART(I2C1, ENABLE);
   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C1, BQ27541_ADDR, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    while(bytes)
    {
      if(bytes == 1)
      {
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        I2C_GenerateSTOP(I2C1, ENABLE);
        while (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET);
        *buff = I2C_ReceiveData(I2C1);
         buff++;
         bytes--;
      }
      else if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
      {
        *buff = I2C_ReceiveData(I2C1);
        buff++;
        bytes--;
      }
    }
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

#endif
/**
  * @brief  Write the specified register from the BQ27541.
  * @param  RegLSB: Lower addr of specified the BQ27541 register to be read
  *               RegMSB: High  addr of specified the BQ27541 register to be read
  * @retval  read specified the register value
  * @author: lifubing
  * @date:2017-05-11
  */
//static uint16_t BQ27541_ReadReg(uint8_t RegLSB,uint8_t RegMSB)
static uint16_t BQ27541_WriteReg(uint8_t RegName, uint16_t RegValue)
{
  /*-------------------------------- Transmission Phase -----------------------*/
  /* Send LM75_I2C START condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on LM75_I2C EV5 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))  /* EV5 */
  {
  }

  /* Send STLM75 slave address for write */
  I2C_Send7bitAddress(I2C1, BQ27541_ADDR, I2C_Direction_Transmitter);

  /* Test on LM75_I2C EV6 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) /* EV6 */
  {
  }

  /* Send the specified register data pointer */
  I2C_SendData(I2C1, RegName);

  /* Test on LM75_I2C EV8 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) /* EV8 */
  {
  }

  /* Send LM75_I2C data */
  I2C_SendData(I2C1, (uint8_t)(RegValue >> 8));

  /* Test on LM75_I2C EV8 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) /* EV8 */
  {
  }

  /* Send LM75_I2C data */
  I2C_SendData(I2C1, (uint8_t)RegValue);

  /* Test on LM75_I2C EV8 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) /* EV8 */
  {
  }

  /* Send LM75_I2C STOP Condition */
  I2C_GenerateSTOP(I2C1, ENABLE);
}

/**
  * @brief  Read the specified register from the BQ27541.
  * @param  RegLSB: Lower addr of specified the BQ27541 register to be read
  *               RegMSB: High  addr of specified the BQ27541 register to be read
  * @retval  read specified the register value
  * @author: lifubing
  * @date:2017-05-11
  */
//static uint16_t BQ27541_ReadReg(uint8_t RegLSB,uint8_t RegMSB)
static uint16_t BQ27541_ReadReg(uint8_t RegLSB)
{
  __IO uint16_t RegValue = 0;
       uint8_t buf[2] = {0};

  /* Enable I2C1 acknowledgement if it is already disabled by other function */
  I2C_AcknowledgeConfig(I2C1, ENABLE);

  /*--------------------------- Transmission Phase ----------------------------*/
  /* Send I2C1 START condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on I2C1 EV5 and clear it */
  Charge_Obj_Data.I2C_timeout = 0;
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if( Charge_Obj_Data.I2C_timeout > I2C_TIMEOUT)
        goto Exit;
   }

  /* Send STLM75 slave address for write */
  I2C_Send7bitAddress(I2C1, BQ27541_ADDR, I2C_Direction_Transmitter);

  /* Test on I2C1 EV6 and clear it */
  Charge_Obj_Data.I2C_timeout = 0;
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) /* EV6 */
  {
    if( Charge_Obj_Data.I2C_timeout > I2C_TIMEOUT)
        goto Exit;
   }


  /* Send the specified register data pointer */
  I2C_SendData(I2C1, RegLSB);

  /* Test on I2C1 EV8 and clear it */
  //while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) /* EV8 */   //fbli add
  Charge_Obj_Data.I2C_timeout = 0;
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) /* EV8_2 */  
  {
    if( Charge_Obj_Data.I2C_timeout > I2C_TIMEOUT)
        goto Exit;
  }


 #if 0
   /* Send the specified register data pointer */
  I2C_SendData(I2C1, RegMSB);

  /* Test on I2C1 EV8 and clear it */
  //while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) /* EV8 */   //fbli add
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) /* EV8_2 */  
  {
  }
 #endif
  /*------------------------------ Reception Phase ----------------------------*/
  /* Send Re-STRAT condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  Charge_Obj_Data.I2C_timeout = 0;
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))  /* EV5 */
  {
    if( Charge_Obj_Data.I2C_timeout > I2C_TIMEOUT)
        goto Exit;
  }


  /* Send STLM75 slave address for read */
  I2C_Send7bitAddress(I2C1, BQ27541_ADDR, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  Charge_Obj_Data.I2C_timeout = 0;
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))  /* EV6 */
  {
    if( Charge_Obj_Data.I2C_timeout > I2C_TIMEOUT)
        goto Exit;
   }


  /* Test on EV7 and clear it */
  Charge_Obj_Data.I2C_timeout = 0;
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))  /* EV7 */
  {
    if( Charge_Obj_Data.I2C_timeout > I2C_TIMEOUT)
        goto Exit;
   }


  /* Store I2C1 received data */
 // RegValue = (uint16_t)(I2C_ReceiveData(I2C1) << 8);
   buf[0] = I2C_ReceiveData(I2C1);

  /* Disable I2C1 acknowledgement */
  I2C_AcknowledgeConfig(I2C1, DISABLE);

  /* Send I2C1 STOP Condition */
  I2C_GenerateSTOP(I2C1, ENABLE);

  /* Test on RXNE flag */
 // while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));  /* EV7 */  ==>not call it that cause hang up
  Charge_Obj_Data.I2C_timeout = 0;
  while (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
  {
    if( Charge_Obj_Data.I2C_timeout > I2C_TIMEOUT)
        goto Exit;
  }

  /* Store I2C1 received data */
  //RegValue |= I2C_ReceiveData(I2C1);
    buf[1] = I2C_ReceiveData(I2C1);
  
   RegValue =  (buf[1]<<8)|buf[0];
  /* Return register value */
  return (RegValue);

 Exit:
    I2C_GenerateSTOP(I2C1,ENABLE);   
    I2C_DeInit(I2C1);
    I2C_SoftwareResetCmd(I2C1,ENABLE);
    Charge_Init_I2C();
    return 0;
}

#if 0
uint8_t BQ27541_ReadRegByte(uint8_t RegLSB,uint8_t RegMSB)
{
  __IO uint16_t RegValue = 0;

  /* Enable I2C1 acknowledgement if it is already disabled by other function */
  I2C_AcknowledgeConfig(I2C1, ENABLE);

  /*--------------------------- Transmission Phase ----------------------------*/
  /* Send I2C1 START condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on I2C1 EV5 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))  /* EV5 */
  {
  }

  /* Send STLM75 slave address for write */
  I2C_Send7bitAddress(I2C1, BQ27541_ADDR, I2C_Direction_Transmitter);

  /* Test on I2C1 EV6 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) /* EV6 */
  {
  }

  /* Send the specified register data pointer */
  I2C_SendData(I2C1, RegLSB);

  /* Test on I2C1 EV8 and clear it */
  //while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) /* EV8 */   //fbli add
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) /* EV8_2 */  
  {
  }

   /* Send the specified register data pointer */
  I2C_SendData(I2C1, RegMSB);

  /* Test on I2C1 EV8 and clear it */
  //while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) /* EV8 */   //fbli add
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) /* EV8_2 */  
  {
  }

  /*------------------------------ Reception Phase ----------------------------*/
  /* Send Re-STRAT condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))  /* EV5 */
  {
  }

  /* Send STLM75 slave address for read */
  I2C_Send7bitAddress(I2C1, BQ27541_ADDR, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))  /* EV6 */
  {
  }

  /* Test on EV7 and clear it */
  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))  /* EV7 */
  {
  }

  /* Store I2C1 received data */
  //RegValue = (uint16_t)(I2C_ReceiveData(I2C1) << 8);

  /* Disable I2C1 acknowledgement */
  I2C_AcknowledgeConfig(I2C1, DISABLE);

  /* Send I2C1 STOP Condition */
  I2C_GenerateSTOP(I2C1, ENABLE);

  /* Test on RXNE flag */
  while (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
  {}

  /* Store I2C1 received data */
 // RegValue |= I2C_ReceiveData(I2C1);
  RegValue = I2C_ReceiveData(I2C1);

  /* Return register value */
  return (RegValue);
}
#endif

static void Battery_sort_adc(unsigned short adc_buf[],int n)
{
     unsigned char i;
     unsigned char j;
    bool isSorted = FALSE;
    unsigned short temp = 0;

    for(i=0; i<n-1; i++)
    {
        isSorted = TRUE;
        for(j=0; j<n-1-i; j++)
        {
            if(adc_buf[j] > adc_buf[j+1])
            {
                isSorted = FALSE;
                temp = adc_buf[j];
                adc_buf[j] = adc_buf[j+1];
                adc_buf[j+1]=temp;
            }
        }
        if(isSorted) break; 
    }
}

static unsigned short Common_Get_ADC_Value(void)
{
    unsigned short adc_buf[20]={0};
    unsigned char i;
    unsigned int vol_sum = 0;
        
    for(i=0;i<20;i++)
    {
        /* start ADC convertion by software */
        ADC_SoftwareStartConv(ADC1);
        /* wait until end-of-covertion */
        while( ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0 );
        /* read ADC convertion result */
        adc_buf[i]= ADC_GetConversionValue(ADC1);    
    }

    Battery_sort_adc(adc_buf, 20);

    for(i=3;i<17;i++)
    {
        vol_sum += adc_buf[i];
    }

    vol_sum = vol_sum / 14; 
    return (unsigned short)vol_sum;
}

#if 0
unsigned int Battery_Cal_Vol_Compensation(unsigned char char_st)
{
    unsigned int res;
    CHARGE_ST st = (CHARGE_ST)char_st;
    __IO unsigned int bat_val = 0;
    
    bat_val = Battery_Get_Vol();
    switch(st)
    {
        case CHARGE_ST_IDEL:
        break;

        case CHARGE_ST_ADAPTER_PLUG_IN_POWER_VR:
        case CHARGE_ST_ADAPTER_PLUG_IN_POWER_PHONE:    
        case CHARGE_ST_ADAPTER_PLUG_IN:
            res = bat_val - charge_mgr.bat_obj.bat_diff_val;
        break;

        case CHARGE_ST_POWER_PHONE:
            res = bat_val + charge_mgr.bat_obj.bat_diff_val;
        break;

        case CHARGE_ST_POWER_VR:
            res = bat_val + charge_mgr.bat_obj.bat_diff_val;
        break;

        default:
        break;
    }

    return res;
}


#endif

unsigned int USB_v_Detect_Get_Vol(void)
{
    __IO unsigned int adc_val = 0;
   // __IO float vol_tmp = 0.0;
  
    ADC_ChannelCmd(ADC1, ADC_Channel_11, DISABLE);
    ADC_ChannelCmd(ADC1, ADC_Channel_18, DISABLE);
    ADC_ChannelCmd(ADC1, ADC_Channel_22, ENABLE);
    /*
    vol_tmp = Common_Get_ADC_Value()/1000.0;
    
    vol_tmp = 2.0*vol_tmp*1.5/4.096;

    bat_val = vol_tmp*1000*2;
    */ 
    adc_val = Common_Get_ADC_Value();
    return adc_val;
}


unsigned int Current_Detect_Get_Vol(void)
{
    __IO unsigned int adc_val = 0;
   // __IO float vol_tmp = 0.0;

    ADC_ChannelCmd(ADC1, ADC_Channel_22, DISABLE);
    ADC_ChannelCmd(ADC1, ADC_Channel_18, DISABLE);
    ADC_ChannelCmd(ADC1, ADC_Channel_11, ENABLE);
    
    /*
    vol_tmp = Common_Get_ADC_Value()/1000.0;
    
    vol_tmp = 2.0*vol_tmp*1.5/4.096;

    bat_val = vol_tmp*1000*2;
    */ 
    adc_val = Common_Get_ADC_Value();
    return adc_val;
}

#if 0
unsigned int MCU_NTC_Get_Vol(void)
{
    __IO unsigned int adc_val = 0;
   // __IO float vol_tmp = 0.0;

    ADC_ChannelCmd(ADC1, ADC_Channel_22, DISABLE);
    ADC_ChannelCmd(ADC1, ADC_Channel_11, DISABLE);
    ADC_ChannelCmd(ADC1, ADC_Channel_18, ENABLE);
    
    /*
    vol_tmp = Common_Get_ADC_Value()/1000.0;
    
    vol_tmp = 2.0*vol_tmp*1.5/4.096;

    bat_val = vol_tmp*1000*2;
    */ 
    adc_val = Common_Get_ADC_Value();
    return adc_val;
}
#endif

BATTERY_LEVEL Battery_Get_Vol_Level(uint16_t percent)
{   

   BATTERY_LEVEL level;
   uint8_t  temp_bat;
   uint8_t  calc_bat;
   if( percent >LOW_BATTERY_PROTECT_VOL)
    {
     calc_bat = ((percent - LOW_BATTERY_PROTECT_VOL)*100)/(100-LOW_BATTERY_PROTECT_VOL);
     temp_bat = ((percent - LOW_BATTERY_PROTECT_VOL)*100)%(100-LOW_BATTERY_PROTECT_VOL);
     if(temp_bat>=40)
        calc_bat++;
    }
   else
     calc_bat = 0;
  if(calc_bat == 0)
   {
     level = BATTERY_LEVEL_0;
   }
  else if(calc_bat <25)  //25
     level = BATTERY_LEVEL_1;
  else  if(calc_bat <50) //50
     level = BATTERY_LEVEL_2;
  else  if(calc_bat <75)  //75
     level = BATTERY_LEVEL_3;
  else
     level = BATTERY_LEVEL_4;

  #if 0 //def SKY_DEBUG
  if(calc_bat <94) //100
     level = BATTERY_LEVEL_4;
  else
  {
    level = BATTERY_LEVEL_FULL;
    //Charge_Obj_Data.led_obj.led_indicate_state =BAT_CHARGDONE_INDICATE;
  }
  #endif
  
  if(( Charge_Obj_Data.led_obj.led_indicate_state == BAT_CHARGING_INDICATE)
     &&(BATTERY_LEVEL_4 == level)
     )
    {
         Get_Battery_avgCurrent();
         if(Charge_Obj_Data.bat_obj.bat_current <CHARGING_FULL_CURRENT)
         {
            level = BATTERY_LEVEL_FULL;
         }
    }
   #if 0 //def SKY_DEBUG
   Charge_Obj_Data.bat_obj.bat_remain_percent = calc_bat;
  #endif
    return level;
}


void  Get_Battery_Percent(void)
{

  uint8_t  cnt = 0;
  uint16_t  percent = 0;
  for(cnt = 0;  cnt <3; cnt++)
   {
       percent = BQ27541_ReadReg(BQ27541CMD_SOC_LSB);
       if( percent> 0)
        {
            Charge_Obj_Data.bat_obj.bat_remain_percent = percent;
            break;
        }
       #ifdef SKY_DEBUG 
        else
       {
        
            printf("Get invalid percent\r\n");
     
        }
      #endif
   }
}

void  Get_Battery_Temp(void)
{

  //temperature  0K = -273.15C
  uint8_t  cnt = 0;
  uint16_t  temp  = 0;
  for(cnt = 0;  cnt <3; cnt++)
   {
       temp = BQ27541_ReadReg(BQ27541CMD_TEMP_LSB);  // uint 0.1K   0C = 2732
       if( temp> 0)
        {
            Charge_Obj_Data.bat_obj.bat_temperature = temp;
            break;
        }
   }
}


void  Get_Battery_avgCurrent(void)
{

  //temperature  0K = -273.15C
  uint8_t  cnt = 0;
  uint16_t  temp  = 0;
  for(cnt = 0;  cnt <3; cnt++)
   {
       temp = BQ27541_ReadReg(BQ27541CMD_AVG_CURRENT_LSB);  // uint 0.1K   0C = 2732
       if( temp>=0)
        {
            Charge_Obj_Data.bat_obj.bat_current =(int16_t)temp;
            break;
        }
   }
}



void Enter_FullSleepMode(void)
{
   BQ27541_WriteReg(0x00, 0x1000);  // 0x0010 that is error
 //  Delay(1000);
  // BQ27541_WriteReg(0x00, 0x0011);
}


#if 0
void Exit_HiberateMode(void)
{

    BQ27541_WriteReg(0x00, 0x0012);
}
#endif

