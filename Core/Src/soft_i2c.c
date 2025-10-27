#include "soft_i2c.h"

/* ==== 超慢时序（重负载线路更稳） ==== */
#define T_SU_US         50
#define T_HD_US         50
#define T_LOW_US      500
#define T_HIGH_US     500
#define T_SDA_RISE_US 1000   /* 释放 SDA 后等待其经 RC 上升 */

/* ★ 将 SCL/SDA 对调：PB7 = SCL (推挽), PB6 = SDA (开漏) */
#define SCL_PORT GPIOB
#define SCL_PIN  GPIO_PIN_7
#define SDA_PORT GPIOB
#define SDA_PIN  GPIO_PIN_6

/* 快速操作宏（SCL 推挽；SDA 开漏） */
#define SCL_H()    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET)
#define SCL_L()    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET)
#define SDA_REL()  HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET)   /* 开漏释放=1(高阻) */
#define SDA_L()    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET) /* 开漏拉低=0 */
#define SDA_READ() (HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN)==GPIO_PIN_SET)

/* us 延时（优先 DWT，回退简易循环） */
static inline void delay_us(uint32_t us){
  if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)){
    volatile uint32_t n = us * 12; while(n--){ __NOP(); }
    return;
  }
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (SystemCoreClock/1000000u) * us;
  while ((DWT->CYCCNT - start) < ticks) {}
}

void SoftI2C_Begin(void){
  /* 只启用 DWT 计数；GPIO 模式由 CubeMX 初始化（SCL 推挽、SDA 开漏） */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* 基本原语 */
static void i2c_start(void){
  SCL_H(); SDA_REL(); delay_us(T_SDA_RISE_US);
  SDA_L(); delay_us(T_HD_US);
  SCL_L(); delay_us(T_LOW_US);
}
static void i2c_stop(void){
  SDA_L(); delay_us(T_SU_US);
  SCL_H(); delay_us(T_HIGH_US);
  SDA_REL(); delay_us(T_SDA_RISE_US);
}
static void i2c_write_bit(int b){
  if (b){ SDA_REL(); delay_us(T_SDA_RISE_US); } else { SDA_L(); delay_us(T_SU_US); }
  SCL_H(); delay_us(T_HIGH_US);
  SCL_L(); delay_us(T_LOW_US);
}
static int i2c_read_bit(void){
  SDA_REL(); delay_us(T_SDA_RISE_US);
  SCL_H(); delay_us(T_HIGH_US);
  int bit = SDA_READ();
  SCL_L(); delay_us(T_LOW_US);
  return bit;
}
static int i2c_write_byte(uint8_t v){
  for (int i=7;i>=0;i--) i2c_write_bit((v>>i)&1);
  return i2c_read_bit()==0; /* ACK=0 */
}
static uint8_t i2c_read_byte(int ack){
  uint8_t v=0; for (int i=7;i>=0;i--) v |= i2c_read_bit()<<i;
  i2c_write_bit(ack?0:1);   /* ack=1 发送ACK(0)，最后一个发 NACK(1) */
  return v;
}

/* 对外接口 */
HAL_StatusTypeDef SoftI2C_Write(uint8_t addr7,const uint8_t *data,uint16_t len){
  i2c_start();
  if(!i2c_write_byte((addr7<<1)|0)){ i2c_stop(); return HAL_ERROR; }
  for(uint16_t i=0;i<len;i++){ if(!i2c_write_byte(data[i])){ i2c_stop(); return HAL_ERROR; } }
  i2c_stop(); return HAL_OK;
}
HAL_StatusTypeDef SoftI2C_Read(uint8_t addr7,uint8_t *buf,uint16_t len){
  i2c_start();
  if(!i2c_write_byte((addr7<<1)|1)){ i2c_stop(); return HAL_ERROR; }
  for(uint16_t i=0;i<len;i++){ buf[i]=i2c_read_byte(i<(len-1)); }
  i2c_stop(); return HAL_OK;
}

/* —— 线路与地址自检 —— */
int SoftI2C_BusIdleOK(void){
  SCL_H(); SDA_REL(); delay_us(T_SDA_RISE_US);
  return SDA_READ();       // 1=空闲&上拉OK；0=SDA 被拉低或上不去
}
int SoftI2C_Ping(uint8_t addr7){
  i2c_start();
  int ack = i2c_write_byte((addr7<<1)|0);
  i2c_stop();
  return ack;              // 1=有设备应答，0=无
}
