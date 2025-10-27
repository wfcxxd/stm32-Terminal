#include "DHT11.h"

/* ---------- DWT 微秒延时 ---------- */
void DWT_Delay_Init(void){
  if(!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)){
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t dwt_now(void){ return DWT->CYCCNT; }
static inline uint32_t us_to_cycles(uint32_t us){
  return (HAL_RCC_GetHCLKFreq()/1000000U) * us;
}
void DWT_Delay_us(uint32_t us){
  uint32_t start = dwt_now();
  uint32_t span  = us_to_cycles(us);
  while ((dwt_now() - start) < span) { __NOP(); }
}

/* ---------- GPIO 方向切换 ---------- */
static void dht_set_output_pp(void){
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin   = DHT11_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;         // 仅用于主动拉低起始信号
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_PORT, &g);
}
static void dht_set_input_pullup(void){
  GPIO_InitTypeDef g = {0};
  g.Pin   = DHT11_PIN;
  g.Mode  = GPIO_MODE_INPUT;
  g.Pull  = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_PORT, &g);
}

/* 等待电平=level，带超时(us)。成功返回0，超时返回-1 */
static int wait_level(GPIO_PinState level, uint32_t timeout_us){
  uint32_t deadline = dwt_now() + us_to_cycles(timeout_us);
  while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) != level){
    if ((int32_t)(deadline - dwt_now()) <= 0) return -1;
  }
  return 0;
}

/* 测量某电平持续时长(us)，最长等待 timeout_us，返回测到的us；超时返回-1 */
static int pulse_in(GPIO_PinState level, uint32_t timeout_us){
  if (wait_level(level, timeout_us) < 0) return -1;      // 等到进入该电平
  uint32_t start = dwt_now();
  uint32_t deadline = start + us_to_cycles(timeout_us);  // 等到离开或超时
  while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == level){
    if ((int32_t)(deadline - dwt_now()) <= 0) return -1;
  }
  uint32_t cyc = dwt_now() - start;
  uint32_t mhz = HAL_RCC_GetHCLKFreq()/1000000U;
  return (int)(cyc / mhz);
}

HAL_StatusTypeDef DHT11_Read(DHT11_DataTypeDef* out)
{
  static uint32_t last_ms = 0;
  uint32_t now = HAL_GetTick();
  if (now < 1500U) return HAL_BUSY;
  if (last_ms && (now - last_ms) < 1000U) return HAL_BUSY;

  /* 线路健康检查：若输入上拉后仍然为低，多半未上电/短路 */
  dht_set_input_pullup();
  DWT_Delay_us(1000);
  if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET){
    return HAL_ERROR;
  }

  /* 起始：仅拉低 -> 释放为输入上拉 */
  dht_set_output_pp();
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
  DWT_Delay_us(25000);
  dht_set_input_pullup();
  DWT_Delay_us(30);

  /* 响应：低~80us，高~80us（宽松阈值） */
  if (pulse_in(GPIO_PIN_RESET, 120) < 60)  return HAL_ERROR;
  if (pulse_in(GPIO_PIN_SET,   120) < 60)  return HAL_ERROR;

  /* 读取40位 */
  uint8_t b[5] = {0};
  for (int i = 0; i < 40; i++){
    if (pulse_in(GPIO_PIN_RESET, 100) < 30) return HAL_ERROR;
    int t = pulse_in(GPIO_PIN_SET, 120);
    if (t < 0) return HAL_ERROR;
    b[i/8] <<= 1;
    b[i/8] |= (t > 45) ? 1 : 0;
  }

  uint8_t sum = (uint8_t)(b[0] + b[1] + b[2] + b[3]);
  if (sum != b[4]) return HAL_ERROR;

  out->humidity    = b[0];
  out->temperature = b[2];
  last_ms = HAL_GetTick();
  return HAL_OK;
}
