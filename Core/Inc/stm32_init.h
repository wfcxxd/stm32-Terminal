#ifndef __STM32_INIT_H
#define __STM32_INIT_H

#include "main.h"
#include "gpio.h"
#include "spi.h"   // 使用 SPI1 的 hspi1

static inline void OLED_CS_L(void){ HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET); }
static inline void OLED_CS_H(void){ HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET); }
static inline void OLED_DC_Cmd(void){ HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET); }
static inline void OLED_DC_Data(void){ HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET); }
static inline void OLED_RST_L(void){ HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET); }
static inline void OLED_RST_H(void){ HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET); }

#endif
