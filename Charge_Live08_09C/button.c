/*************************************************************************************
 * Copyright:    Copyright (c) 2016 SkyLight R&D Centre ()
 *
 * File name:    button.c
 *
 * Description:  MCU
 *
 * Author:  lifubing   Version:v2.0.0 Date: 2017.4.13
 *                
 * History:                     
**************************************************************************************/
#include "button.h"
#include "charge.h"
#include "stm8l15x.h"
#include "stdio.h"
#if 0
CHARGE_BTN_ACTION Btn_Detect_Action(void)
{
    bool btn_press = FALSE;
    btn_press = (CHARGE_ACT_BTN_PRESS == 
                (charge_mgr.charge_obj.act_charge & CHARGE_ACT_BTN_PRESS))?
                TRUE : FALSE;
    //if (1 == charge_mgr.charge_btn.btn_it)
    if (btn_press)
    {
        if (7 == charge_mgr.charge_btn.btn_time_wait)
        {
            charge_mgr.charge_btn.btn_press = CHARGE_BTN_DOWN;
        }
        if (CHARGE_BTN_DOWN == charge_mgr.charge_btn.btn_press)
        {
           if (0 == GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)) 
           {
                if (69 == charge_mgr.charge_btn.btn_time_long)
                {
                    //charge_mgr.charge_btn.btn_it = 0;
                    //charge_mgr.charge_obj.act_charge &= ~CHARGE_ACT_BTN_PRESS;
                    #ifdef SKY_DEBUG  
                    printf("long press key\r\n");
                    #endif
                    return CHARGE_BTN_ACT_LONG;          
                }
           }
           else
           {
                //charge_mgr.charge_btn.btn_it = 0;
                //charge_mgr.charge_obj.act_charge &= ~CHARGE_ACT_BTN_PRESS;
                #ifdef SKY_DEBUG  
                    printf("short press key\r\n");
                #endif
                return CHARGE_BTN_ACT_SHORT;
           }
        }

    }

    return CHARGE_BTN_ACT_INVALID;
}
#endif

