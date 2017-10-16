#ifndef __PRODUCT_TEST_H
#define __PRODUCT_TEST_H

#define SN_NUM            64
#define FW_VER            64


#define PRO_LINE_PRINTF_PRODUCTTEST_OK   "<ProductTest> producttest OK <ProductTest>"
#define PRO_LINE_PRINTF_EXIT_PRODUCTTEST "<ProductTest> Exit producttest <ProductTest>"

#define PRO_LINE_PRINTF_SNWRITE_OK       "<ProductTest> SNwrite OK <ProductTest>"

#define PRO_LINE_PRINTF_MCU_FW_VERSION         "Apollo-mcui7_v2.03.01"
#define PRO_LINE_PRINTF_SHORT_PRESS      "KEY-BATTERY PRESS"


typedef struct _PRO_LINE_TEST_OBJ_T{
    unsigned  char  sn_num[SN_NUM];
    unsigned  char  fw_ver[FW_VER];
}PRO_LINE_TEST_OBJ;



void Product_Test_Detct(void);



#endif
