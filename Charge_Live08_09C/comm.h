/*************************************************************************************
 * Copyright:    Copyright (c) 2016 SkyLight R&D Centre (Xi 'an)
 *
 * File name:    comm.h
 *
 * Description:  MCU
 *
 * Author:  liuhang   Version:v2.0.0 Date: 2017.4.13
 *                
 * History:                     
**************************************************************************************/
#ifndef __COMM_H
#define __COMM_H

//#define DATA_LEN  32
#define UART_IDLE_TIME   2// 2  //5  // 10

#define SN_ADDRESS          0x1040
#define SSN_ADDRESS         SN_ADDRESS+0x20
#define MODEL_ADDRESS       SSN_ADDRESS+0x20
#define HW_ADDRESS          MODEL_ADDRESS+0x20

/*
#define SN_LENGTH           19  //18
#define SSN_LENGTH          15
#define MODEL_LENGTH        11
#define HW_LENGTH           11
*/
/**
* MCU(UART1) CMD SUPPORT
*/
typedef enum CMD_MCU_tag
{   
    /* a12 get mcu version */   
    CMD_GET_MCU_VERSION         = 0x8001, 

    CMD_START = CMD_GET_MCU_VERSION,
    /*** mcu to a12  ***/

    /* mcu set a12 shutdown */  
    CMD_MCU_SET_SHUT_DOWN       = 0x8002,

    /*** a12(app) to mcu  ***/
    /* app(a12) set mcu stop charge threshold */
    CMD_SET_MCU_STOP_CHARGE_THRESHOLD = 0x8003,   
    
    /* app(a12) mcu update */
    CMD_MCU_UPDATE_START        = 0x8004,
    CMD_MCU_PACKET_TRANS        = 0X8005,
    
    /* a12 shut down notify mcu */
    CMD_SHUT_DOWN_NOTIFY_MSG    = 0x8006,
    
    /*app(a12) get mcu current voltage */
    CMD_GET_MCU_VOLTAGE         = 0x8007,   

    CMD_A12_POWER_ON            = 0x8009,

    CMD_LOW_VOLTAGE_SHUT_DOWN_MSG = 0x800A,
    CMD_GET_MCU_HW_VERSION        = 0x800B,
    /* a12 to mcu update start, mcu from app jump to iap */
    CMD_MCU_UPDATE_JUMP           = 0x8010,
    
    CMD_GET_MCU_SN                = 0x8011,
    CMD_END = CMD_GET_MCU_SN,

}CMD_MCU_em;
    

typedef enum CMD_MCU_ERR_CODE_tag
{
    MCU_ERR_OK                      = 0,
    MCU_ERR_SEND                    = -1000,        // send Err
    MCU_ERR_RECEIVE_TIMEOUT         = -1002,        // receive timeout
    MCU_ERR_CMD_SEQ                 = -1003,        // index of answer is not correct
    MCU_ERR_NULL                    = -1004,        // pointer is null
    MCU_ERR_INVALIDPAR              = -1005,        // params is out of range
    MCU_ERR_MD5_CHECKED_FAILED      = -1006,        // md5 is not correct
}MCU_ERR_CODE_em;


typedef struct CMD_MCU_HEADER_st_
{
    unsigned char       sync;
    unsigned short      usCmd;
    unsigned char       ucSeq;
    union{
        short  usPayLoadLen;
        short  usErrCode;
    };
    unsigned short      iCrc;
    unsigned char       tail;
}CMD_MCU_HEADER_st;


extern unsigned char product_test_flag;
extern unsigned char product_testBat_flag;
void Commun_Send_Cmd(void  *msg, unsigned int len);
//void Commun_Recv_Cmd(void);
//void Charge_Uart_Cmd_Handler(void);
void usart_send_data(void *str);
void Send_MCU_Shutdown_cmd(void);
void usart_dealing_func(void);
#endif