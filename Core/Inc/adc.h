#ifndef __ADC_H__
#define __ADC_H__

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern ADC_HandleTypeDef hadc1;

void MX_ADC1_Init(void);
uint32_t Read_VDDA_mV(void);

#ifdef __cplusplus
}
#endif

#endif
