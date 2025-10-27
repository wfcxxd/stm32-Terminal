#ifndef __DHT11_H__
#define __DHT11_H__

#include "stm32f1xx_hal.h"

#define DHT11_PORT GPIOA
#define DHT11_PIN  GPIO_PIN_1

typedef struct {
  uint8_t temperature;
  uint8_t humidity;
} DHT11_DataTypeDef;

void DWT_Delay_Init(void);
void DWT_Delay_us(uint32_t us);
HAL_StatusTypeDef DHT11_Read(DHT11_DataTypeDef* out);

#endif
