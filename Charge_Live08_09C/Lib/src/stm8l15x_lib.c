
#include "stm8l15x_lib.h"

void ADC_DeInit(ADC_TypeDef* ADCx)
{
  /*  Set the Configuration registers to their reset values */
  ADCx->CR1 =  ADC_CR1_RESET_VALUE;
  ADCx->CR2 =  ADC_CR2_RESET_VALUE;
  ADCx->CR3 =  ADC_CR3_RESET_VALUE;

  /*  Set the status registers to their reset values */
  ADCx->SR =  (uint8_t)~ADC_SR_RESET_VALUE;

  /*  Set the High threshold registers to their reset values */
  ADCx->HTRH =  ADC_HTRH_RESET_VALUE;
  ADCx->HTRL =  ADC_HTRL_RESET_VALUE;

  /*  Set the low threshold registers to their reset values */
  ADCx->LTRH =  ADC_LTRH_RESET_VALUE;
  ADCx->LTRL =  ADC_LTRL_RESET_VALUE;

  /*  Set the channels sequence registers to their reset values */
  ADCx->SQR[0] =  ADC_SQR1_RESET_VALUE;
  ADCx->SQR[1] =  ADC_SQR2_RESET_VALUE;
  ADCx->SQR[2] =  ADC_SQR3_RESET_VALUE;
  ADCx->SQR[3] =  ADC_SQR4_RESET_VALUE;

  /*  Set the channels Trigger registers to their reset values */
  ADCx->TRIGR[0] =  ADC_TRIGR1_RESET_VALUE;
  ADCx->TRIGR[1] =  ADC_TRIGR2_RESET_VALUE;
  ADCx->TRIGR[2] =  ADC_TRIGR3_RESET_VALUE;
  ADCx->TRIGR[3] =  ADC_TRIGR4_RESET_VALUE;
}

void ADC_Init(ADC_TypeDef* ADCx,
              ADC_ConversionMode_TypeDef ADC_ConversionMode,
              ADC_Resolution_TypeDef ADC_Resolution,
              ADC_Prescaler_TypeDef ADC_Prescaler)
{
  /* Check the parameters */
  //assert_param(IS_ADC_CONVERSION_MODE(ADC_ConversionMode));
  //assert_param(IS_ADC_RESOLUTION(ADC_Resolution));
  //assert_param(IS_ADC_PRESCALER(ADC_Prescaler));

  /*clear CR1 register */
  ADCx->CR1 &= (uint8_t)~(ADC_CR1_CONT | ADC_CR1_RES);

  /* set the resolution and the conversion mode */
  ADCx->CR1 |= (uint8_t)((uint8_t)ADC_ConversionMode | (uint8_t)ADC_Resolution);

  /*clear CR2 register */
  ADCx->CR2 &= (uint8_t)~(ADC_CR2_PRESC);

  /* set the Prescaler */
  ADCx->CR2 |= (uint8_t) ADC_Prescaler;
}

void ADC_Cmd(ADC_TypeDef* ADCx,
             FunctionalState NewState)
{
  /* Check the parameters */
  //assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Set the ADON bit to wake up the specified ADC from power down mode */
    ADCx->CR1 |= ADC_CR1_ADON;
  }
  else
  {
    /* Disable the selected ADC peripheral */
    ADCx->CR1 &= (uint8_t)~ADC_CR1_ADON;
  }
}

/**
  * @brief  Starts ADC conversion, by software trigger.
  * @param  ADCx where x can be 1 to select the specified ADC peripheral.
  * @retval None
  */
void ADC_SoftwareStartConv(ADC_TypeDef* ADCx)
{
  /*  Start the ADC software conversion */
  ADCx->CR1 |= ADC_CR1_START;
}


void ADC_ChannelCmd(ADC_TypeDef* ADCx, ADC_Channel_TypeDef ADC_Channels, FunctionalState NewState)
{
  uint8_t regindex = 0;
  /* Check the parameters */
  //assert_param(IS_FUNCTIONAL_STATE(NewState));

  regindex = (uint8_t)((uint16_t)ADC_Channels >> 8);

  if (NewState != DISABLE)
  {
    /* Enable the selected ADC channel(s). */
    ADCx->SQR[regindex] |= (uint8_t)(ADC_Channels);
  }
  else
  {
    /* Disable the selected ADC channel(s). */
    ADCx->SQR[regindex] &= (uint8_t)(~(uint8_t)(ADC_Channels));
  }
}

void ADC_SamplingTimeConfig(ADC_TypeDef* ADCx,
                            ADC_Group_TypeDef ADC_GroupChannels,
                            ADC_SamplingTime_TypeDef ADC_SamplingTime)
{
  /* Check the parameters */
  //assert_param(IS_ADC_GROUP(ADC_GroupChannels));
  //assert_param(IS_ADC_SAMPLING_TIME_CYCLES(ADC_SamplingTime));

  if ( ADC_GroupChannels != ADC_Group_SlowChannels)
  {
    /* Configures the sampling time for the Fast ADC channel group. */
    ADCx->CR3 &= (uint8_t)~ADC_CR3_SMPT2;
    ADCx->CR3 |= (uint8_t)(ADC_SamplingTime << 5);
  }
  else
  {
    /* Configures the sampling time for the Slow ADC channel group. */
    ADCx->CR2 &= (uint8_t)~ADC_CR2_SMPT1;
    ADCx->CR2 |= (uint8_t)ADC_SamplingTime;
  }
}

void ADC_SchmittTriggerConfig(ADC_TypeDef* ADCx, ADC_Channel_TypeDef ADC_Channels,
                              FunctionalState NewState)
{
  uint8_t regindex = 0;
  /* Check the parameters */
  //assert_param(IS_FUNCTIONAL_STATE(NewState));

  regindex = (uint8_t)((uint16_t)ADC_Channels >> 8);

  if (NewState != DISABLE)
  {
    /* Enable the Schmitt Trigger for the selected ADC channel(s).*/
    ADCx->TRIGR[regindex] &= (uint8_t)(~(uint8_t)ADC_Channels);
  }
  else
  {
    /* Disable the Schmitt Trigger for the selected ADC channel(s).*/
    ADCx->TRIGR[regindex] |= (uint8_t)(ADC_Channels);
  }
}


uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx)
{
  uint16_t tmpreg = 0;

  /* Get last ADC converted data.*/
  tmpreg = (uint16_t)(ADCx->DRH);
  tmpreg = (uint16_t)((uint16_t)((uint16_t)tmpreg << 8) | ADCx->DRL);

  /* Return the selected ADC conversion value */
  return (uint16_t)(tmpreg) ;
}



