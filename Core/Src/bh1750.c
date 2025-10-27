#include "bh1750.h"
#include "soft_i2c.h"
#include "stm32f1xx_hal.h"   // 为 HAL_Delay

/* 运行时保存当前使用的地址与模式 */
static uint8_t s_addr = BH1750_ADDR_LO;
static uint8_t s_mode = BH1750_CONT_HIRES;

/* 发送 1 字节命令 */
static HAL_StatusTypeDef bh1750_write(uint8_t cmd) {
  return SoftI2C_Write(s_addr, &cmd, 1);
}

/**
 * @brief  初始化 BH1750
 * @param  prefer_addr  首选地址（BH1750_ADDR_LO=0x23 或 BH1750_ADDR_HI=0x5C）
 * @param  mode         工作模式（如 BH1750_CONT_HIRES / ONE_SHOT_HIRES 等）
 * @note   若首选地址不应答，会自动切换到另一地址再试
 */
HAL_StatusTypeDef BH1750_Init(uint8_t prefer_addr, uint8_t mode)
{
  s_addr = prefer_addr;
  s_mode = mode;

  /* Power On (0x01) */
  uint8_t cmd = 0x01;
  if (SoftI2C_Write(s_addr, &cmd, 1) != HAL_OK) {
    /* 首选地址失败，切到另一地址 */
    s_addr = (prefer_addr == BH1750_ADDR_LO) ? BH1750_ADDR_HI : BH1750_ADDR_LO;
    if (SoftI2C_Write(s_addr, &cmd, 1) != HAL_OK) {
      return HAL_ERROR;
    }
  }

  /* Reset (0x07) */
  cmd = 0x07;
  if (SoftI2C_Write(s_addr, &cmd, 1) != HAL_OK) {
    return HAL_ERROR;
  }

  /* 设置工作模式 */
  if (bh1750_write(s_mode) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief  读取光照强度（lux）
 * @param  lux  输出（原始值/1.2）
 * @note   ONE_SHOT 模式下每次读都会先触发一次转换再等待；
 *         CONT 模式直接读两个字节。
 */
HAL_StatusTypeDef BH1750_ReadLux(float *lux)
{
  if (!lux) return HAL_ERROR;

  /* ONE_SHOT 模式：先触发一次转换并等待完成 */
  if ((s_mode & 0x20) == 0x20) {
    if (bh1750_write(s_mode) != HAL_OK) return HAL_ERROR;

    /* 典型转换时间：Hi-Res/Hi-Res2 ≈120ms，Low-Res ≈16ms
       为稳妥（加上你的“超慢”I²C线），这里取安全等待 */
    uint32_t wait_ms = 180;
    if ((s_mode & 0x03) == 0x00) { // Low-Res
      wait_ms = 30;
    }
    HAL_Delay(wait_ms);
  }

  /* 连续模式直接读两个数据字节 */
  uint8_t buf[2] = {0};
  HAL_StatusTypeDef st = SoftI2C_Read(s_addr, buf, 2);
  if (st != HAL_OK) return st;

  uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
  *lux = (float)raw / 1.2f;

  return HAL_OK;
}
