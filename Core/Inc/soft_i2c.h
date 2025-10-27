#ifndef __SOFT_I2C_H__
#define __SOFT_I2C_H__

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 仅启用 us 级延时；不修改引脚模式（PB6=推挽，PB7=开漏 由 CubeMX 配好） */
void SoftI2C_Begin(void);

/* 7 位地址的写/读接口 */
HAL_StatusTypeDef SoftI2C_Write(uint8_t addr7, const uint8_t *data, uint16_t len);
HAL_StatusTypeDef SoftI2C_Read (uint8_t addr7,       uint8_t *buf , uint16_t len);

/* —— 线路与地址自检 —— */
int SoftI2C_BusIdleOK(void);     // 释放 SDA 后是否能读到高电平（上拉/RC 正常）
int SoftI2C_Ping(uint8_t addr7); // 该 7位地址是否应答

#ifdef __cplusplus
}
#endif
#endif
