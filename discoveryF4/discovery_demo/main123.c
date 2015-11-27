// STM32 ADC1 CH11 (PC.1) STM32F4 Discovery - sourcer32@gmail.com
 
#include "main.h"
#include "usbd_hid_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

//Library config for this project!!!!!!!!!!!
#include "stm32f4xx_conf.h"
#include "stm32f4xx_adc.h"

/**************************************************************************************/
 
void RCC_Configuration(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
}
 
/**************************************************************************************/
 
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
 
  /* ADC Channel 11 -> PC1 */
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}
 
/**************************************************************************************/
 
void ADC_Configuration(void)
{
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_InitTypeDef ADC_InitStructure;
 
  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
 
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 1 Channel
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Conversions Triggered
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // Manual
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
 
  /* ADC1 regular channel 11 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_144Cycles); // PC1
 
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
}
 
/**************************************************************************************/
 
#define BUFFERSIZE 128
 
uint16_t ADCConvertedValues[BUFFERSIZE];
 
int main(void)
{
  int i;
 
    RCC_Configuration();
 
    GPIO_Configuration();
 
  ADC_Configuration();
 
  STM_EVAL_LEDInit(LED3); /* Configure LEDs to monitor program status */
 
  STM_EVAL_LEDOn(LED3); /* Turn LED3 on */
 
  i = 0;
 
  while(1) // Don't want to exit
  {
    /* Start ADC1 Software Conversion */
    ADC_SoftwareStartConv(ADC1);
 
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
 
    ADCConvertedValues[i++] = ADC_GetConversionValue(ADC1);
 
    i %= BUFFERSIZE;
 
    /* Toggle LED3 */
    STM_EVAL_LEDToggle(LED3);
  }
}