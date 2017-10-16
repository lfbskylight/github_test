#ifndef __USART_H
#define __USART_H

//#define USART_ENABLE 1
#define USART_RX_BUF_LEN  64  //128
#define USART_TX_BUF_LEN  64  //128

/*
typedef enum _USART_DATA_TYPE{
    USART_DATA_NULL,
    USART_DATA_PRO_TEST,
    USART_DATA_COMMUN,
}USART_DATA_TYPE;
*/
typedef struct _USART_OBJ_T{
    //unsigned  short  usart_rx_time;
    //unsigned  char   usart_rx_time;
    unsigned  char   usart_pack_start_flag;
    unsigned  char   usart_pack_dealing_flag;
    unsigned  char   usart_rx_flg;
    unsigned  char   usart_buf_len;
    unsigned  char   usart_rx_time;
  //  USART_DATA_TYPE  usart_type;
    unsigned  short  buf_index;
    unsigned  char   rx_read;
    unsigned  char   rx_write;
    unsigned  char   usart_rx_buf[USART_RX_BUF_LEN];
    unsigned  char   usart_tx_buf[USART_TX_BUF_LEN];
}USART_OBJ;


//#ifdef USART_ENABLE
unsigned char UART1_ReceiveByte(void);
void UART1_SendStr(unsigned char *str);
void UART1_SendStrLen(unsigned char *str, unsigned int len);
void UART1_DeInit(void);
void UART1_SendByte(unsigned char data);
//#endif

#endif
