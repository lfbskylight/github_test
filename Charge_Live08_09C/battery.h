/*************************************************************************************
 * Copyright:    Copyright (c) 2016 SkyLight R&D Centre (Xi 'an)
 *
 * File name:    battery.h
 *
 * Description:  MCU
 *
 * Author:  liuhang   Version:v2.0.0 Date: 2017.4.13
 *                
 * History:                     
**************************************************************************************/
#ifndef __BATTERY_H
#define __BATTERY_H

#include "stm8l15x.h"

#define  I2C_TIMEOUT    2
#define  BQ27541_ADDR   0xAA
/*
#define BQ27541CMD_CNTL_LSB  0x00
#define BQ27541CMD_CNTL_MSB  0x01
#define BQ27541CMD_AR_LSB    0x02
#define BQ27541CMD_AR_MSB    0x03
#define BQ27541CMD_ARTTE_LSB 0x04
#define BQ27541CMD_ARTTE_MSB 0x05
*/
#define BQ27541CMD_TEMP_LSB             0x06
#define BQ27541CMD_AVG_CURRENT_LSB      0x14
/*
#define BQ27541CMD_TEMP_MSB  0x07
#define BQ27541CMD_VOLT_LSB  0x08
#define BQ27541CMD_VOLT_MSB  0x09

#define BQ27541CMD_FLAGS_LSB 0x0A
#define BQ27541CMD_FLAGS_MSB 0x0B
#define BQ27541CMD_NAC_LSB   0x0C
#define BQ27541CMD_NAC_MSB   0x0D
#define BQ27541CMD_FAC_LSB   0x0E
#define BQ27541CMD_FAC_MSB   0x0F

#define BQ27541CMD_RM_LSB    0x10      //compensated battery capcity remaining
#define BQ27541CMD_RM_MSB    0x11
#define BQ27541CMD_FCC_LSB   0x12      //compensated full battery  capcity 
#define BQ27541CMD_FCC_MSB   0x13
#define BQ27541CMD_AI_LSB    0x14

#define BQ27541CMD_AI_MSB    0x15
#define BQ27541CMD_TTE_LSB   0x16
#define BQ27541CMD_TTE_MSB   0x17
#define BQ27541CMD_TTF_LSB   0x18
#define BQ27541CMD_TTF_MSB   0x19
#define BQ27541CMD_SI_LSB    0x1A
#define BQ27541CMD_SI_MSB    0x1B
#define BQ27541CMD_STTE_LSB  0x1C
#define BQ27541CMD_STTE_MSB  0x1D
#define BQ27541CMD_MLI_LSB   0x1E
#define BQ27541CMD_MLI_MSB   0x1F
#define BQ27541CMD_MLTTE_LSB 0x20
#define BQ27541CMD_MLTTE_MSB 0x21
#define BQ27541CMD_AE_LSB    0x22
#define BQ27541CMD_AE_MSB    0x23
#define BQ27541CMD_AP_LSB    0x24
#define BQ27541CMD_AP_MSB    0x25
#define BQ27541CMD_TTECP_LSB 0x26
#define BQ27541CMD_TTECP_MSB 0x27
#define BQ27541CMD_RSVD_LSB  0x28
#define BQ27541CMD_RSVD_MSB  0x29
#define BQ27541CMD_CC_LSB    0x2A
#define BQ27541CMD_CC_MSB    0x2B
*/
#define BQ27541CMD_SOC_LSB   0x2C  // remaini battery capacity percent
//#define BQ27541CMD_SOC_MSB   0x2D

/*
#define BQ27541CMD_DCAP_LSB  0x3C
#define BQ27541CMD_DCAP_MSB  0x3D
#define BQ27541CMD_DFCLS     0x3E
#define BQ27541CMD_DFBLK     0x3F
#define BQ27541CMD_ADF       0x40
#define BQ27541CMD_ACKSDFD   0x54
#define BQ27541CMD_DFDCKS    0x60
#define BQ27541CMD_DFDCNTL   0x61
#define BQ27541CMD_DNAMELEN  0x62
#define BQ27541CMD_DNAME     0x63
*/


/*
#define BATTERY_LEVEL_MAX               BATTERY_LEVEL_FULL
#define BATTERY_LEVEL_LOW_PROTECTION    BATTERY_LEVEL_0
#define BATTERY_LEVEL_PERCENT_25        BATTERY_LEVEL_1
*/


typedef enum _BATTERY_LEVEL{
    BATTERY_LEVEL_0,
    BATTERY_LEVEL_1,
    BATTERY_LEVEL_2, 
    BATTERY_LEVEL_3,
    BATTERY_LEVEL_4,
    BATTERY_LEVEL_FULL,
}BATTERY_LEVEL;



typedef struct _BATTERY_MGR_OBJ_T{
    BATTERY_LEVEL    bat_level;
   //BATTERY_LEVEL    bat_threshold;
    uint8_t          bat_threshold;
    uint16_t         bat_det_time;
    uint16_t         bat_discharge_time;
    uint16_t         bat_temperature;
    uint16_t         bat_voltage;
    uint16_t         bat_remain_percent;
    int16_t          bat_current;
    uint16_t         bat_capacity;
    uint16_t         bat_fullcapacity;
}BATTERY_MGR_OBJ;

//declare variable
extern const uint8_t  Battery_Percent[6];

// declare function 
//unsigned int Battery_Get_Vol(void);
//BATTERY_LEVEL Battery_Get_Vol_Level(__IO unsigned int bat_val, unsigned char pow_type, unsigned char st);
unsigned int USB_v_Detect_Get_Vol(void);
//unsigned int MCU_NTC_Get_Vol(void);
unsigned int Current_Detect_Get_Vol(void);
//uint16_t BQ27541_ReadReg(uint8_t RegLSB,uint8_t RegMSB);
void Charge_Init_I2C(void);
//void i2c_bq27541_read(uint8_t *buff,uint8_t cmd,uint8_t bytes);
BATTERY_LEVEL Battery_Get_Vol_Level(uint16_t percent);
void  Get_Battery_Percent(void);
void  Get_Battery_Temp(void);
void  Get_Battery_avgCurrent(void);
void Enter_FullSleepMode(void);
//void Exit_HiberateMode(void);
#endif

