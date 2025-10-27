#include "adc.h"

ADC_HandleTypeDef hadc1;

void MX_ADC1_Init(void)
{
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);  // PCLK2/6 ≈ 12 MHz

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc1);

  /* 开内部通道 */
  ADC1->CR2 |= ADC_CR2_TSVREFE;
  HAL_Delay(2);
  HAL_ADCEx_Calibration_Start(&hadc1);
}

uint32_t Read_VDDA_mV(void)
{
  ADC_ChannelConfTypeDef s = {0};
  s.Channel = ADC_CHANNEL_VREFINT;
  s.Rank = ADC_REGULAR_RANK_1;
  s.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &s);

  uint32_t sum = 0;
  for (int i=0;i<8;i++){
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    sum += HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
  }
  uint32_t raw = sum / 8;
  if (!raw) raw = 1;

  return (1200UL * 4095UL) / raw;  // Vrefint≈1.20V 估算
}
