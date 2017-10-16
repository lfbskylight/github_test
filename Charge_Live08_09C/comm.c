/*************************************************************************************
 * Copyright:    Copyright (c) 2016 SkyLight R&D Centre (Xi 'an)
 *
 * File name:    comm.c
 *
 * Description:  MCU
 *
 * Author:  liuhang   Version:v2.0.0 Date: 2017.4.13
 *                
 * History:                     
**************************************************************************************/
#include "comm.h"
#include "usart.h"
#include "charge.h"
#include "string.h"
#include "help.h"
#include "flash.h"
#include "stdio.h"
#include "battery.h"
#include <string.h>
//#include<stdlib.h>

unsigned char product_test_flag = 0;
unsigned char product_testBat_flag = 0;

uint8_t sn_length = 0;
uint8_t ssn_length = 0;
uint8_t model_length = 0;
uint8_t hw_length = 0;
//unsigned char buff[128] = {0};
//unsigned char cnt = 0;
static unsigned short crc16(unsigned char *ptr, unsigned char count)
{
  #if 0
    unsigned short  crc =0;
    unsigned char  i;
    while(count-- >0)
    {
        crc = ( crc^(((unsigned int)*ptr)<<8));
        for(i=0;i<8;i++)
        {   
            if(crc&0x8000) crc= ((crc<<1)^0x1021);
            else crc <<= 1;
        }
        ptr++;
    }
    #else
    uint8_t i;
    uint16_t crc = 0xFFFF;
    while(count--)
    {
         crc ^= *ptr;
         for(i=0x01; i!= 0; i<<=1)
         {
            if((crc & 0x001) != 0)
            {
                   crc >>= 1;
                   crc ^= 0xA001;
            }
            else
            {
                 crc >>= 1;
            }
         }
         ptr++;
    }
    
    #endif
    return crc;
}

void Commun_Send_Cmd(void *msg, unsigned int len)
{
     memset(Charge_Obj_Data.usart_obj.usart_tx_buf, 0x00, sizeof(Charge_Obj_Data.usart_obj.usart_tx_buf));
     memcpy(Charge_Obj_Data.usart_obj.usart_tx_buf, msg, len);
   //  Charge_Obj_Data.usart_obj.usart_type    = USART_DATA_COMMUN;
     Charge_Obj_Data.usart_obj.usart_buf_len = len;
                
    // USART_ITConfig (USART1,USART_IT_TXE,ENABLE);  

     UART1_SendStrLen(Charge_Obj_Data.usart_obj.usart_tx_buf, 
                                 Charge_Obj_Data.usart_obj.usart_buf_len);

   memset(Charge_Obj_Data.usart_obj.usart_tx_buf, 0x00, sizeof(Charge_Obj_Data.usart_obj.usart_rx_buf));
}


void usart_send_data(void *str)
{
 uint8_t len;
 len = strlen(str);
  memset(Charge_Obj_Data.usart_obj.usart_tx_buf, 0x00, sizeof(Charge_Obj_Data.usart_obj.usart_tx_buf));
  memcpy(Charge_Obj_Data.usart_obj.usart_tx_buf, str, len);
 // strcpy((char*)Charge_Obj_Data.usart_obj.usart_tx_buf,(char*)str);
 // strcpy(Charge_Obj_Data.usart_obj.usart_tx_buf,str);
 // Charge_Obj_Data.usart_obj.usart_type    = USART_DATA_PRO_TEST;
  //Charge_Obj_Data.usart_obj.usart_buf_len = len;
       
#if 0        
  USART_ITConfig (USART1,USART_IT_TXE,ENABLE);    
#else
   UART1_SendStr(Charge_Obj_Data.usart_obj.usart_tx_buf);
   UART1_SendByte(0x0D);
   UART1_SendByte(0x0A);

   memset(Charge_Obj_Data.usart_obj.usart_tx_buf, 0x00, sizeof(Charge_Obj_Data.usart_obj.usart_rx_buf));
#endif
}

static void uart_pro_send(CMD_MCU_em cmd,unsigned char sec_number,unsigned int length,const char *data)
{
  unsigned int count = 0;
  int checksum = 0;
  char pack[64] = {0};
  pack[0] = 0xFE;
  pack[1] = cmd & 0xff;                  //取高位放在低地址
  pack[2] = cmd >> 8;                    //取地位放在高地址
  pack[3] = sec_number;
  pack[4] = length & 0xff;
  pack[5] = length >> 8;
  for(count=0;count<length;count++)
  {
    pack[6+count] = data[count];
  }
  checksum = crc16((unsigned char *)pack,length+6);
  pack[length+6] = checksum & 0xFF;
  pack[length+7] = checksum >> 8;
  pack[length+8] = 0xEF;               //帧尾
  
 //usart_send_data((unsigned char*)pack,length+9);
 UART1_SendStrLen((unsigned char*)pack,length+9);
}

#if 0
void Commun_Recv_Cmd(void)
{
    unsigned char  bat_val = 0;
    unsigned char  *ptr      = NULL;
    unsigned short  cmd      = 0;
    unsigned char   data[64] = {0};
    unsigned char   msg_data[64] = {0};
    unsigned char   iap_flg  = 0;
    unsigned char   bat_percent  = 0;
    unsigned short  msg_len  = 0;
    unsigned short  data_len  = 0;
    CMD_MCU_HEADER_st  msg_head;
    unsigned char   high = 0;
    unsigned char   low  = 0;
    //uint8_t buf[2] = {0};
  //  ptr = Charge_Obj_Data.usart_obj.usart_rx_buf;
    ptr = &Charge_Obj_Data.usart_obj.usart_rx_buf[Charge_Obj_Data.usart_obj.buf_index];
    

    if (0xFE  == *ptr)
    {
        memset(&msg_head, 0x00, sizeof(msg_head));
        memset(data, 0x00, sizeof(data));
        memset(msg_data, 0x00, sizeof(msg_data));
        msg_head.sync = *ptr;
        
        low = *++ptr;
        high = *++ptr;
        cmd =  (high << 8) | low;
        msg_head.usCmd = Convert_High_Low(cmd);
        
        msg_head.ucSeq = *++ptr;
        low = *++ptr;
        high = *++ptr;
        msg_head.usPayLoadLen = (high << 8) | low;
        if (msg_head.usPayLoadLen <= 0)
        {
            ;//msg_head.usPayLoadLen = 0;
        }
        else if ( msg_head.usPayLoadLen > 0)
        {
            ++ptr;
            memcpy(data, ptr, msg_head.usPayLoadLen);
            ptr += (msg_head.usPayLoadLen-1);
            
        }
       // low = *ptr;  
        low = *++ptr;
        high = *++ptr;
        msg_head.iCrc = (high << 8) | low;
        msg_head.iCrc = Convert_High_Low(msg_head.iCrc);
        msg_head.tail = *++ptr;

        switch(cmd)
        {
            /* a12 shut down notify mcu */
            case CMD_SHUT_DOWN_NOTIFY_MSG:
                 
                Commun_Send_Cmd(&msg_head, sizeof(CMD_MCU_HEADER_st));
                Charge_Obj_Data.state_switch_flag = TRUE;
                Charge_Obj_Data.camera_on_off_flag = FALSE;
                Charge_Obj_Data.check_switch_state_time = SWITCH_STATE_TIMEOUT;
                Charge_Obj_Data.force_check_state_flag = TRUE;
                #if 0 //def SKY_DEBUG  
                     printf("cam of\r\n");
                #endif
                
              //  Charge_Vr_Plug_Out_Or_Close(VR_SHUT_DOWN);  //fbli temp mask
            break;

            /*a12 power on notify mcu*/
            case CMD_A12_POWER_ON:
                Charge_Obj_Data.state_switch_flag       = TRUE;
                Charge_Obj_Data.check_switch_state_time = SWITCH_STATE_TIMEOUT;
                Charge_Obj_Data.camera_on_off_flag      = TRUE;
                Charge_Obj_Data.camera_uart_on_flag     = TRUE;
                Charge_Obj_Data.force_check_state_flag = TRUE;
                Commun_Send_Cmd(&msg_head, sizeof(CMD_MCU_HEADER_st));
                 #if 0 //def SKY_DEBUG  
                     printf("cam on\r\n");
                #endif
               // Charge_Vr_Plug_In();   //fbli temp mask
            break;

            /* a12 to mcu update start, mcu from app jump to iap */
            case CMD_MCU_UPDATE_JUMP:
                 Commun_Send_Cmd(&msg_head, sizeof(CMD_MCU_HEADER_st));
                 iap_flg  = IAP_VALUE_FLG;
                 Write_Flash_Byte(FLASH_MemType_Data, IAP_ADDRESS, 1 , &iap_flg);
                 
                /*
                            asm("LDW X, SP");
                            asm("LD  A, $FF");   
                            asm("LD  XL, A");
                            asm("LDW SP, X");
                            asm("JPF $8000");
                             */
                while(1);
            break;

            /*app(a12) get mcu current voltage */
            case CMD_GET_MCU_VOLTAGE:
            /* a12 get mcu version */   
            case CMD_GET_MCU_VERSION:

                /* sync cmd seq  (4)*/
                msg_len = sizeof(msg_head.sync) + sizeof(msg_head.usCmd) + sizeof(msg_head.ucSeq);
                memcpy(msg_data, &msg_head, msg_len);

                if (CMD_GET_MCU_VERSION == cmd)
                {
                    /* payloadLen */
                   data_len = strlen(MCU_version);  
                  //  data_len = sizeof(MCU_version);  //compile error
                    
                   // data_len = Convert_High_Low(Charge_Obj_Data.charge_info_obj.version_len);
                    data_len = Convert_High_Low(data_len);
                    memcpy(msg_data + msg_len, &data_len, sizeof(msg_head.usPayLoadLen));
                    msg_len += sizeof(msg_head.usPayLoadLen);

                    /* data */
                    data_len = strlen(MCU_version);
                    memcpy(msg_data + msg_len, MCU_version, data_len);
                    msg_len += data_len;
                    /*
                                 memcpy(msg_data + msg_len, Charge_Obj_Data.charge_info_obj.version, Charge_Obj_Data.charge_info_obj.version_len);
                                msg_len += Charge_Obj_Data.charge_info_obj.version_len;
                                */
                }
                else if (CMD_GET_MCU_VOLTAGE == cmd)
                {
                    /* payloadLen */
                    data_len = Convert_High_Low(0x0001);
                    memcpy(msg_data + msg_len, &data_len, sizeof(msg_head.usPayLoadLen));
                    msg_len += sizeof(msg_head.usPayLoadLen);

                    /* data */
                  // bat_percent = Battery_Percent[Charge_Obj_Data.bat_obj.bat_level];
                    Get_Battery_Percent();
                    bat_percent = ((Charge_Obj_Data.bat_obj.bat_remain_percent - LOW_BATTERY_PROTECT_VOL)*100)/(100-LOW_BATTERY_PROTECT_VOL);
                    memcpy(msg_data + msg_len, &bat_percent, 1);
                    msg_len += 1;
                }

                /* crc */
                msg_head.iCrc = crc16(msg_data, msg_len); 
                msg_head.iCrc = Convert_High_Low(msg_head.iCrc);
                memcpy(msg_data + msg_len, &msg_head.iCrc, sizeof(msg_head.iCrc));
                msg_len += sizeof(msg_head.iCrc);

                /* tail */
                memcpy(msg_data + msg_len, &msg_head.tail, sizeof(msg_head.tail));
                msg_len += sizeof(msg_head.tail);

                Commun_Send_Cmd((void *)msg_data, msg_len);
            break;

            case CMD_SET_MCU_STOP_CHARGE_THRESHOLD:
                msg_head.usPayLoadLen = 0;
                bat_val = data[0];
                //Charge_Obj_Data.bat_obj.bat_threshold = Charge_Set_Bat_Threshold(bat_val);
                Charge_Obj_Data.bat_obj.bat_threshold = bat_val;
                Commun_Send_Cmd(&msg_head, sizeof(CMD_MCU_HEADER_st));

            break;

            case CMD_READ_MCU_SN:
                 Read_Flash_Byte(FLASH_MemType_Data,SN_ADDRESS,SN_LENGTH,data);
                  /* sync cmd seq  (4)*/
                msg_len = sizeof(msg_head.sync) + sizeof(msg_head.usCmd) + sizeof(msg_head.ucSeq);
                memcpy(msg_data, &msg_head, msg_len);

                    /* payloadLen */
                   data_len = SN_LENGTH;  
                    
                   // data_len = Convert_High_Low(Charge_Obj_Data.charge_info_obj.version_len);
                    data_len = Convert_High_Low(data_len);
                    memcpy(msg_data + msg_len, &data_len, sizeof(msg_head.usPayLoadLen));
                    msg_len += sizeof(msg_head.usPayLoadLen);

                    /* data */
                    data_len = SN_LENGTH;
                    memcpy(msg_data + msg_len, data, data_len);
                    msg_len += data_len;
                    
                /* crc */
                msg_head.iCrc = crc16(msg_data, msg_len); 
                msg_head.iCrc = Convert_High_Low(msg_head.iCrc);
                memcpy(msg_data + msg_len, &msg_head.iCrc, sizeof(msg_head.iCrc));
                msg_len += sizeof(msg_head.iCrc);

                /* tail */
                memcpy(msg_data + msg_len, &msg_head.tail, sizeof(msg_head.tail));
                msg_len += sizeof(msg_head.tail);

                Commun_Send_Cmd((void *)msg_data, msg_len);
            break;
            default:
            break;
        }
       
        
   }

    
}
#endif
void usart_dealing_func(void)
{
  unsigned char i = 0;
  unsigned char cnt = 0;
  unsigned int command = 0;
  unsigned char length = 0;
  unsigned char sec_number = 0;
  unsigned char *ptr = NULL;
  unsigned char data[64] = {0};
  unsigned char buff[64] = {0};
  unsigned char   iap_flg  = 0;
  unsigned char   bat_percent  = 0;
  uint8_t temp_bat = 0;
  
 if ((1 == Charge_Obj_Data.usart_obj.usart_rx_flg)
     &&(Charge_Obj_Data.usart_obj.usart_rx_time == 0)
     )
   {
      Charge_Obj_Data.usart_obj.usart_rx_flg = 0;
    //  Charge_Obj_Data.usart_obj.usart_rx_time = 0;
      while(Charge_Obj_Data.usart_obj.rx_read != Charge_Obj_Data.usart_obj.rx_write)
      {
        buff[cnt++] = Charge_Obj_Data.usart_obj.usart_rx_buf[Charge_Obj_Data.usart_obj.rx_read];
        if(++Charge_Obj_Data.usart_obj.rx_read == USART_RX_BUF_LEN)
          Charge_Obj_Data.usart_obj.rx_read = 0;
      }
     
      //PRODUCT TEST
      //begin product test
      if(strstr(buff,"<set_test:on>") != NULL)
      {
        product_test_flag = 1;
        cnt  = 0;
        usart_send_data("<OK>");
        memset(Charge_Obj_Data.usart_obj.usart_rx_buf, 0x00, sizeof(Charge_Obj_Data.usart_obj.usart_rx_buf));
        Charge_Obj_Data.usart_obj.rx_read = 0;
        Charge_Obj_Data.usart_obj.rx_write = 0;
      }
      else
      //end product test
      if(strstr(buff,"<set_test:off>") != NULL)
      {
        product_test_flag = 0;
        product_testBat_flag = 0;
       // Charge_Obj_Data.state_switch_flag = TRUE;
       // Charge_Obj_Data.check_switch_state_time = 0;
      //  Charge_Obj_Data.force_check_state_flag = TRUE;
        usart_send_data("<OK>");
        memset(Charge_Obj_Data.usart_obj.usart_rx_buf, 0x00, sizeof(Charge_Obj_Data.usart_obj.usart_rx_buf));
        Charge_Obj_Data.usart_obj.rx_read = 0;
        Charge_Obj_Data.usart_obj.rx_write = 0;
        #if 0
        if(Charge_Obj_Data.led_obj.led_indicate_state != STATE_IDLE )
        {
               Charge_Obj_Data.bat_obj.bat_remain_percent = BQ27541_ReadReg(BQ27541CMD_SOC_LSB,BQ27541CMD_SOC_MSB);
               Charge_Obj_Data.bat_obj.bat_level =  Battery_Get_Vol_Level(Charge_Obj_Data.bat_obj.bat_remain_percent);
               Battery_Level_Led_Indicator(Charge_Obj_Data.bat_obj.bat_level);
             
         }
       #endif
      }
      
      if(product_test_flag == 1)
      {
        //1.1: wriet serial number
        if((ptr = strstr(buff,"set_Sn")) != NULL)
        {
          ptr += 7;
          sn_length = strlen(ptr) -1;
          Write_Flash_Byte(FLASH_MemType_Data,SN_ADDRESS,sn_length,ptr);
       //   Write_Flash_Byte(FLASH_MemType_Data,SN_ADDRESS,SN_LENGTH,ptr);
          
          usart_send_data("<OK>");
        }
        else if(strstr(buff,"<get_Sn>") != NULL)  //1.2: read serial number
        {
        //  Read_Flash_Byte(FLASH_MemType_Data,SN_ADDRESS,SN_LENGTH,data);
          Read_Flash_Byte(FLASH_MemType_Data,SN_ADDRESS,32,data);
          sprintf(buff,"<OK:%s>",data);
          usart_send_data(buff);
        }
        else  if(strstr(buff,"<get_fw>") != NULL)
        //2: check fw unmber
        {
        //  Read_Flash_Byte(FLASH_MemType_Data,FW_VERSION_ADDRESS,FW_VERSION_LENGTH,data);
          sprintf(buff,"<OK:%s>",MCU_version);
          usart_send_data(buff);
        }
        // check half product version
        else if((ptr = strstr(buff,"set_ssn")) != NULL)
        {
          ptr += 8;
          ssn_length = strlen(ptr) -1;
          Write_Flash_Byte(FLASH_MemType_Data,SSN_ADDRESS,ssn_length,ptr);
        //  Write_Flash_Byte(FLASH_MemType_Data,SSN_ADDRESS,SSN_LENGTH,ptr);
        
          usart_send_data("<OK>");
        }
        else if(strstr(buff,"<get_ssn>") != NULL)
        //1.2: read serial number
        {
        //  Read_Flash_Byte(FLASH_MemType_Data,SSN_ADDRESS,SSN_LENGTH,data);
          Read_Flash_Byte(FLASH_MemType_Data,SSN_ADDRESS,32,data);
          sprintf(buff,"<OK:%s>",data);
          usart_send_data(buff);
        }
        else if(strstr(buff,"<set_led:on>") != NULL)
        //3.1: led on
        {
          Led_ALL_ON();
          usart_send_data("<OK>");
        }
        else  if(strstr(buff,"<set_led:off>") != NULL)
        //3.2: led off
        {
          Led_ALL_OFF();
          usart_send_data("<OK>");
        }
        else if(strstr(buff,"<get_bat>") != NULL)
        //4: get battery capacity
        {
          product_testBat_flag = 1;
          Charge_Obj_Data.led_obj.led_flash_time = 0;
          Get_Battery_Percent();
          sprintf(buff,"<OK:%d>",Charge_Obj_Data.bat_obj.bat_remain_percent);
          usart_send_data(buff);
         // Charge_Obj_Data.led_obj.led_indicate_state  = BAT_LEVEL_INDICATE;
        }
        else if((ptr = strstr(buff,"set_model")) != NULL)
        //5.1: write model
        {
          ptr += 10;
          model_length = strlen(ptr) -1;
          Write_Flash_Byte(FLASH_MemType_Data,MODEL_ADDRESS,model_length,ptr);
        //  Write_Flash_Byte(FLASH_MemType_Data,MODEL_ADDRESS,MODEL_LENGTH,ptr);
          
          usart_send_data("<OK>");
        }
        else if(strstr(buff,"<get_model>") != NULL)
        //5.2: read model
        {
          //Read_Flash_Byte(FLASH_MemType_Data,MODEL_ADDRESS,MODEL_LENGTH,data);
           Read_Flash_Byte(FLASH_MemType_Data,MODEL_ADDRESS,32,data);
          sprintf(buff,"<OK:%s>",data);
          usart_send_data(buff);
        }
        else if((ptr = strstr(buff,"set_hw")) != NULL)
        //6.1: write hw
        {
          ptr += 7;
          hw_length = strlen(ptr) -1;
          Write_Flash_Byte(FLASH_MemType_Data,HW_ADDRESS,hw_length,ptr);
        //  Write_Flash_Byte(FLASH_MemType_Data,HW_ADDRESS,HW_LENGTH,ptr);
          
          usart_send_data("<OK>");
        }
        else if(strstr(buff,"<get_hw>") != NULL)
        //6.2:read hw
        {
        //  Read_Flash_Byte(FLASH_MemType_Data,HW_ADDRESS,HW_LENGTH,data);
          Read_Flash_Byte(FLASH_MemType_Data,HW_ADDRESS,32,data);
          sprintf(buff,"<OK:%s>",data);
          usart_send_data(buff);
        }
         else if(strstr(buff,"<get_charging_method>") != NULL)
        //6.2:get charging method : wireless or USB-C
        {
          if( RESET == GPIO_ReadInputDataBit(USB_DETECT_PORT,USB_DETECT_PINS))
                usart_send_data("<OK:USB-C>");
          else if(RESET != GPIO_ReadInputDataBit(WIRLESS_DETECT_PORT,WIRLESS_DETECT_PINS))
               usart_send_data("<OK:Wireless>");
          else
               usart_send_data("<Err:No charge>");
        }
        else if(strstr(buff,"<get_charging_mode>") != NULL)
        //6.2:get charging mode : fast charging or normal charging
        {
            if( RESET == GPIO_ReadInputDataBit(USB_DETECT_PORT,USB_DETECT_PINS))
            {
                    Charge_Obj_Data.usb_det_adc       =       USB_v_Detect_Get_Vol();
                    if(Charge_Obj_Data.usb_det_adc >FAST_CHARGE_ADC_VALUE) // fast charge adapter
                    {
                        usart_send_data("<OK:Fast charging>");
                    }
                    else if( Charge_Obj_Data.usb_det_adc <SLOW_CHARGE_ADC_VALUE)// slow charge adapter
                    {
                        usart_send_data("<OK:Normal charging>");       
                    }
             }
             else
              usart_send_data("<Err:No plug in USB-C>");  
        }
         
        //key test
      }
     // else
      {
           //communication command
          for(i=0;i<cnt;i++)
          {
            if(buff[i] == 0xfe)    //head
            {
              command = buff[i+2]<<8 | buff[i+1];
              length = buff[i+5]<<8 | buff[i+4];
              sec_number = buff[i+3];
              //if(crc16(&buff[i],length+5) == (buff[i+5+length+1] << 8 | buff[i+5+length]))
              if(1)
              {
                switch(command)
                {
                  case CMD_GET_MCU_VERSION:
                     length = strlen(MCU_version);
                     /*
                     memcpy(&data[0],MCU_version,length);
                     data[length] = '\0';
                     length ++;
                     uart_pro_send(CMD_GET_MCU_VERSION,sec_number,length,(const char*)data);
                     */
                    uart_pro_send(CMD_GET_MCU_VERSION,sec_number,length,MCU_version);   
                  break;
                  case CMD_MCU_SET_SHUT_DOWN:   
                      lowbattery_cnt =5;
                      Charge_Obj_Data.state_switch_flag = TRUE;
                      Charge_Obj_Data.camera_on_off_flag = FALSE;
                      Charge_Obj_Data.check_switch_state_time = SWITCH_STATE_TIMEOUT;
                      Charge_Obj_Data.force_check_state_flag = TRUE;
                  break;
                  case CMD_SET_MCU_STOP_CHARGE_THRESHOLD:              
                  //  Charge_Obj_Data.bat_obj.bat_threshold =  buff[i+6];
                    Charge_Obj_Data.bat_obj.bat_threshold = (( buff[i+6]*(100-LOW_BATTERY_PROTECT_VOL))/100) + LOW_BATTERY_PROTECT_VOL;
                 //   Charge_Obj_Data.bat_obj.bat_threshold += LOW_BATTERY_PROTECT_VOL;
                    uart_pro_send(CMD_SET_MCU_STOP_CHARGE_THRESHOLD,sec_number,0,NULL);
                  break;
                  case CMD_SHUT_DOWN_NOTIFY_MSG:                        
                    uart_pro_send(CMD_SHUT_DOWN_NOTIFY_MSG,sec_number,0,NULL);    
                    Charge_Obj_Data.state_switch_flag = TRUE;
                    Charge_Obj_Data.camera_on_off_flag = FALSE;
                    Charge_Obj_Data.check_switch_state_time = SWITCH_STATE_TIMEOUT;
                    Charge_Obj_Data.force_check_state_flag = TRUE;
                  break;
                  case CMD_GET_MCU_VOLTAGE:
                    Get_Battery_Percent();
                    if( Charge_Obj_Data.bat_obj.bat_remain_percent >LOW_BATTERY_PROTECT_VOL)
                    {
                        bat_percent = ((Charge_Obj_Data.bat_obj.bat_remain_percent - LOW_BATTERY_PROTECT_VOL)*100)/(100-LOW_BATTERY_PROTECT_VOL);
                        temp_bat = ((Charge_Obj_Data.bat_obj.bat_remain_percent - LOW_BATTERY_PROTECT_VOL)*100)%(100-LOW_BATTERY_PROTECT_VOL);
                            if(temp_bat>=40)
                              bat_percent++;
                    }
                    else
                        bat_percent = 0;
                    uart_pro_send(CMD_GET_MCU_VOLTAGE,sec_number,1,(const char*)&bat_percent);          
                  break;
                  case CMD_A12_POWER_ON:
                    Charge_Obj_Data.state_switch_flag       = TRUE;
                    Charge_Obj_Data.check_switch_state_time = SWITCH_STATE_TIMEOUT;
                    Charge_Obj_Data.camera_on_off_flag      = TRUE;
                    Charge_Obj_Data.camera_uart_on_flag     = TRUE;
                    Charge_Obj_Data.force_check_state_flag = TRUE;
                    Get_Battery_Percent();
                    if( Charge_Obj_Data.bat_obj.bat_remain_percent >LOW_BATTERY_PROTECT_VOL)
                        data[0] = ((Charge_Obj_Data.bat_obj.bat_remain_percent - LOW_BATTERY_PROTECT_VOL)*100)/(100-LOW_BATTERY_PROTECT_VOL);
                    else
                         data[0] = 0;
                    length = strlen(MCU_version);
                    memcpy(&data[1],MCU_version,length);
                    length++;
                  // data[length] = '\0';
                  // length++;
                    uart_pro_send(CMD_A12_POWER_ON,sec_number,length,(const char*)data);            
                  break;
                  case CMD_LOW_VOLTAGE_SHUT_DOWN_MSG:                             
                    uart_pro_send(CMD_LOW_VOLTAGE_SHUT_DOWN_MSG,sec_number,0,NULL); 
                    Charge_Obj_Data.state_switch_flag = TRUE;
                    Charge_Obj_Data.camera_on_off_flag = FALSE;
                    Charge_Obj_Data.check_switch_state_time = SWITCH_STATE_TIMEOUT;
                    Charge_Obj_Data.force_check_state_flag = TRUE;
                  break;
                  case CMD_MCU_UPDATE_JUMP:
                   iap_flg  = IAP_VALUE_FLG;
                   Write_Flash_Byte(FLASH_MemType_Data, IAP_ADDRESS, 1 , &iap_flg);
                    uart_pro_send(CMD_MCU_UPDATE_JUMP,sec_number,0,NULL);         
                    while(1);                                                      
                    break;
                  case CMD_GET_MCU_SN:
                    Read_Flash_Byte(FLASH_MemType_Data,SN_ADDRESS,32,data);
                    sn_length = strlen(data);
                    for(i = 0; i< sn_length; i++)
                    {
                          if(data[i] == 0x3E)
                          {
                             sn_length = i;
                             data[i] = 0;
                             break;
                          }
                    }
					/*
                    length = SN_LENGTH;
                    data[length] ='\0';
                    length ++;
                    uart_pro_send(CMD_GET_MCU_SN,sec_number,length,(const char*)data);   
					*/   
					uart_pro_send(CMD_GET_MCU_SN,sec_number,sn_length,data);   
                  break;
                  case CMD_GET_MCU_HW_VERSION:
                   Read_Flash_Byte(FLASH_MemType_Data, HW_ADDRESS, 32, data);
                   hw_length = strlen((char *)data);
                   for(i = 0; i< hw_length; i++)
                    {
                          if(data[i] == 0x3E)
                          {
                             hw_length = i;
                             data[i] = 0;
                             break;
                          }
                    }
                   uart_pro_send(CMD_GET_MCU_HW_VERSION,sec_number,hw_length,(char *)data);
                   break;
                  default:
                    break;
                }
              }
            }
          }
            
        }
      
     }
 
}


/**
  * @brief  uart cmd handler.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
#if 0
void Charge_Uart_Cmd_Handler(void)
{
     usart_dealing_func();
}
#endif
/**
  * @brief  mcu notify A12 shutdown.
  * @param  None
  * @retval None
  * @author: lifubing
  * @date:2017-05-11
  */
void Send_MCU_Shutdown_cmd(void)
{
   #if 1
    CMD_MCU_HEADER_st    msg_head;
    memset(&msg_head, 0x00, sizeof(CMD_MCU_HEADER_st));
    msg_head.sync = 0xFE;
    msg_head.usCmd = Convert_High_Low(CMD_MCU_SET_SHUT_DOWN);
    msg_head.ucSeq = 0;
    msg_head.usPayLoadLen = 0;
    msg_head.iCrc = crc16((unsigned char *)&msg_head, 6);;
    msg_head.iCrc = Convert_High_Low(msg_head.iCrc);
    msg_head.tail = 0XEF;
    Commun_Send_Cmd((void *)&msg_head, 9);
   #else
    unsigned int count = 0;
  int checksum = 0;
  unsigned char pack[16] = {0};
  pack[0] = 0xFE;
  pack[1] = 0x02;                  //取高位放在低地址
  pack[2] = 0x80;                    //取地位放在高地址
  pack[3] = 0;
  pack[4] = 0;
  pack[5] = 0;
  checksum = crc16(pack,6);
  pack[6] = checksum & 0xFF;
  pack[7] = checksum >> 8;
  pack[8] = 0xEF;               //帧尾
  Commun_Send_Cmd((void *)pack, 9);
   #endif
}

