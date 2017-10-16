#include "time.h"



void Charge_Time4_Start(void)
{
    //CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
    TIM4_DeInit();
    TIM4_TimeBaseInit(TIM4_Prescaler_128, 249);  // timer base  timer = ( 249+1)/(2Mhz/128) = 16.ms   // slck = 2Mhz
    /* Clear TIM4 update flag */
    TIM4_ClearFlag(TIM4_FLAG_Update);
    TIM4_ITConfig(TIM4_IT_Update, ENABLE);
    TIM4_ARRPreloadConfig(ENABLE); 
    TIM4_Cmd(ENABLE);    
}

#if 0
void Charge_Time4_Stop(void)
{
    
    TIM4_DeInit();
}


void Charge_Time2_Start(void)
{
    TIM2_DeInit();
    TIM2_TimeBaseInit(TIM4_Prescaler_256, TIM2_CounterMode_Up, 0x7FFF);
    TIM2_ITConfig(TIM2_IT_Update, ENABLE);
    TIM2_ARRPreloadConfig(ENABLE); 
    TIM2_Cmd(ENABLE);    
}

void Charge_Time2_Stop(void)
{
    
    TIM2_DeInit();
}
#endif
