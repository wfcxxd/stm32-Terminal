#ifndef __BH1750_H__
#define __BH1750_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BH1750_ADDR_LO   0x23u  // ADDR=GND/浮空
#define BH1750_ADDR_HI   0x5Cu  // ADDR=VCC

// 常用模式：连续高分辨率(1lx/step，典型120ms)
#define BH1750_CONT_HIRES   0x10u
#define BH1750_ONE_HIRES    0x20u

HAL_StatusTypeDef BH1750_Init(uint8_t prefer_addr, uint8_t mode);
HAL_StatusTypeDef BH1750_ReadLux(float *lux);

#ifdef __cplusplus
}
#endif
#endif
