#include<string.h>
#include "product_test.h"
#include "charge.h"

#ifdef PRO_LINE_TEST
void Product_Test_Detct(void)
{
    char *tmp = NULL;
    int len = 0;
    char buf[80] = {0};
    
    if (1 == charge_mgr.usart_obj.usart_rx_flg &&
        charge_mgr.usart_obj.usart_rx_time > 40)
    {
        if (strcmp(charge_mgr.usart_obj.usart_rx_buf, "t producttest begin") == 0)
        {
            UART1_SendStr(PRO_LINE_PRINTF_PRODUCTTEST_OK);
        }
        else if (strcmp(charge_mgr.usart_obj.usart_rx_buf, "t producttest end") == 0)
        {
            UART1_SendStr(PRO_LINE_PRINTF_EXIT_PRODUCTTEST);
        }
        else if (strstr(charge_mgr.usart_obj.usart_rx_buf, "SNwrite") != 0)
        {
            
            tmp = strstr(charge_mgr.usart_obj.usart_rx_buf, "SNwrite");
            tmp += strlen("SNwrite") + 1;
            if (*tmp != '\0')
            {
                memset(charge_mgr.test_obj.sn_num, 0x00, sizeof(charge_mgr.test_obj.sn_num));
                len = strlen(tmp);
                if (len > 1)
                {
                    //strncpy(charge_mgr.test_obj.sn_num, tmp, len);
                    strncpy(charge_mgr.test_obj.sn_num, tmp, sizeof(charge_mgr.test_obj.sn_num) - 1);
                    UART1_SendStr(PRO_LINE_PRINTF_SNWRITE_OK);
                }
            }
        }
        else if (strcmp(charge_mgr.usart_obj.usart_rx_buf, "t producttest SNread") == 0)
        {
            memset(buf, 0x00, sizeof(buf));
            sprintf(buf, "<ProductTest> %s <ProductTest>", charge_mgr.test_obj.sn_num);
            UART1_SendStr(buf);
        }
        else if (strcmp(charge_mgr.usart_obj.usart_rx_buf, "t producttest FW-version") == 0) //t producttest FW-version
        {
            memset(buf, 0x00, sizeof(buf));
            sprintf(buf, "<ProductTest> %s <ProductTest>", charge_mgr.test_obj.fw_ver);
            UART1_SendStr(buf);
        }
        else if (strcmp(charge_mgr.usart_obj.usart_rx_buf, "t producttest LED on") == 0)
        {
            Led_ON_OFF_Mgr(LED_ALL_ON);
        }
        else if (strcmp(charge_mgr.usart_obj.usart_rx_buf, "t producttest LED off") == 0)
        {
            Led_ON_OFF_Mgr(LED_ALL_OFF);
        }
        
        memset(charge_mgr.usart_obj.usart_rx_buf, 0x00, sizeof(charge_mgr.usart_obj.usart_rx_buf));
        charge_mgr.usart_obj.buf_index = 0;
        charge_mgr.usart_obj.usart_rx_flg = 0;
    }
}
#endif


