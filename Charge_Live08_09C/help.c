/*************************************************************************************
 * Copyright:    Copyright (c) 2016 SkyLight R&D Centre (Xi 'an)
 *
 * File name:    help.c
 *
 * Description:  MCU
 *
 * Author:  liuhang   Version:v2.0.0 Date: 2017.4.13
 *                
 * History:                     
**************************************************************************************/
#if 0
void Delay(volatile unsigned short nCount)
{
    /* Decrement nCount value */
    while (nCount > 1)
    {
        nCount--;
    }
}
#endif
unsigned short Convert_High_Low(unsigned short param)
{
    return ( ((param & 0x00FF)<<8) | ((param & 0xFF00) >> 8) );
}

