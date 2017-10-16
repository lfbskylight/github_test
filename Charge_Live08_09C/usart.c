#include "stm8l15x.h"
#include "usart.h"

#ifdef SKY_DEBUG  //USART_ENABLE
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)


PUTCHAR_PROTOTYPE
{
  /* Write a character to the USART */
  USART_SendData8(USART1, c);
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

  return (c);
}
/**
  * @brief Retargets the C library scanf function to the USART.
  * @param[in] None
  * @retval char Character to Read
  * @par Required preconditions:
  * - None
  */
GETCHAR_PROTOTYPE
{
  int c = 0;
  /* Loop until the Read data register flag is SET */
  while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    c = USART_ReceiveData8(USART1);
    return (c);
}
#endif


unsigned char UART1_ReceiveByte(void)
{
    unsigned char UART1_RX_BUF; 
     
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
     
    UART1_RX_BUF = USART_ReceiveData8(USART1);
     
    return  UART1_RX_BUF;
}

void UART1_SendByte(unsigned char data)
{
    USART_SendData8(USART1, data);
   // while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); //always use
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}


void UART1_SendStr(unsigned char *str)
{
    while(*str != '\0')
    {
        UART1_SendByte(*str++); 
    }   
}

void UART1_SendStrLen(unsigned char *str, unsigned int len)
{
    unsigned int i;
    for (i=0;i<len;i++)
    {
        UART1_SendByte(str[i]); 
    }
}



//#endif



