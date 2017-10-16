/*************************************************************************************
 * Copyright:    Copyright (c) 2016 SkyLight R&D Centre (Xi 'an)
 *
 * File name:    flash.c
 *
 * Description:  MCU
 *
 * Author:  liuhang   Version:v2.0.0 Date: 2017.4.13
 *                
 * History:                     
**************************************************************************************/
#include "flash.h"

//¶ÁFlash
void Read_Flash_Byte(FLASH_MemType_TypeDef FLASH_MemType,uint32_t address,unsigned int num,uint8_t *buff)
{
  unsigned int cnt;
  FLASH_Unlock(FLASH_MemType);

  for(cnt=0;cnt<num;cnt++)
  {
    *(buff+cnt) = FLASH_ReadByte(address+cnt);
    FLASH_WaitForLastOperation(FLASH_MemType);
  }
  
  FLASH_Lock(FLASH_MemType);
}

//Ð´Flash
void Write_Flash_Byte(FLASH_MemType_TypeDef FLASH_MemType,uint32_t address,unsigned int num,uint8_t *buff)
{
  unsigned int cnt;
  FLASH_Unlock(FLASH_MemType);

  for(cnt=0;cnt<32;cnt++)
  {
    WWDG_SetCounter(0x7f);
    FLASH_EraseByte(address+cnt);
    FLASH_WaitForLastOperation(FLASH_MemType);
  }
  for(cnt=0;cnt<num;cnt++)
  {
   // FLASH_EraseByte(address+cnt);
   // FLASH_WaitForLastOperation(FLASH_MemType);
    FLASH_ProgramByte(address+cnt,*(buff+cnt));
    FLASH_WaitForLastOperation(FLASH_MemType);
  }
  
  FLASH_Lock(FLASH_MemType);
}

