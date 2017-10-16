#ifndef __BUTTON_H
#define __BUTTON_H

typedef enum _CHARGE_BTN_STA{
    CHARGE_BTN_UP,
    CHARGE_BTN_DOWN,
    CHARGE_BTN_INVALID = -1,
}CHARGE_BTN_STA;

typedef enum _CHARGE_BTN_ACTION{
    CHARGE_BTN_ACT_SHORT,
    CHARGE_BTN_ACT_LONG,
    CHARGE_BTN_ACT_DOUBLE,
    CHARGE_BTN_ACT_INVALID = -1,
}CHARGE_BTN_ACTION;

typedef enum _BTN_POW_CHARGE_PHONE{
    BTN_UN_POW_PHONE,
    BTN_POW_PHONE,
    BTN_POW_INVALID = -1,
}BTN_POW_CHARGE_PHONE;


typedef struct _CHARGE_BTN_T{
   //unsigned char  btn_it; //press
   unsigned char  btn_time_wait;//time_ji
   unsigned char  btn_time_long;
   unsigned char  btn_percent25_flg;
   CHARGE_BTN_STA btn_press;//short
   //BTN_POW_CHARGE_PHONE  btn_pow_phone;
}CHARGE_BTN;


extern CHARGE_BTN_ACTION Btn_Detect_Action(void);


#endif
