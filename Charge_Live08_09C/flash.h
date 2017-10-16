/*************************************************************************************
 * Copyright:    Copyright (c) 2016 SkyLight R&D Centre (Xi 'an)
 *
 * File name:    flash.h
 *
 * Description:  MCU
 *
 * Author:  liuhang   Version:v2.0.0 Date: 2017.4.13
 *                
 * History:                     
**************************************************************************************/
#ifndef __FLASH__
#define __FLASH__

#include "stm8l15x.h"

#define IAP_ADDRESS    (0x001010)
#define IAP_VALUE_FLG  (0x12)

void Write_Flash_Byte(FLASH_MemType_TypeDef FLASH_MemType,uint32_t address,unsigned int num,uint8_t *buff);
void Read_Flash_Byte(FLASH_MemType_TypeDef FLASH_MemType,uint32_t address,unsigned int num,uint8_t *buff);
#endif
