#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "soft_i2c.h"

#include "ssd1306.h"
#include "DHT11.h"
#include "adc.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "bh1750.h"
#include "stm32_init.h"   // Read_VDDA_mV()

/* =============================================================================
 *                      配置与说明（保持页面顺序：ENV->LUX->NB）
 * =============================================================================
 * - 开机默认在 ENV 页
 * - BT 页：只在收到回车/换行或空闲超时后显示本次文本（两行空间，不加省略号）
 * - 每 10 秒主动上报一行状态到手机/电脑（可通过宏关闭）
 * - 电机：PB11 短按=进入/留在手动并启停；长按=退出手动回自动（已修复长按后误触发短按）
 * - 顶部右侧：喇叭（告警），左 10px 风扇（占空比>0 时转），再左 10px “M”手动指示
 * - PWM 使用 TIM2_CH1@PA0（需在 CubeMX 里启用）
 * ============================================================================= */

/* Demo 主动上报 */
#define BT_DEMO_TX_ENABLE   1
#define BT_DEMO_PERIOD_MS   10000

#define WELCOME_SHOW_MS       5000u
#define SELFTEST_SHOW_MS      5000u
#define OLED_VISUAL_TEST_MS   0u
#define WELCOME_TUNE_ENABLE   1

#define WELCOME_TUNE_MS_MIN   1200u
#define WELCOME_TUNE_MS_MAX   4000u

/* OLED 刷新 & 心跳 LED */
#define OLED_REFRESH_MS       350u
#define HB_PERIOD_MS          2000u
#define HB_ON_MS              60u

/* ======== 温湿报警参数======== */
#define TEMP_HIGH             35
#define TEMP_LOW              24
#define TEMP_HYST              1

#define HUMI_HIGH             80
#define HUMI_LOW              35
#define HUMI_HYST              3

#define ALARM_COOLDOWN_MS  30000u

/* 页面定义（顺序保持：ENV -> LUX -> BT） */
typedef enum { PAGE_ENV = 0, PAGE_LUX = 1, PAGE_BT = 2, PAGE_COUNT = 3 } page_t;
static volatile page_t g_page = PAGE_ENV;

/* BH1750 运行期缓存 */
static HAL_StatusTypeDef g_bh1750_status = HAL_ERROR;
static float             g_last_lux      = 0.0f;
static uint32_t          g_next_lux_ms   = 0;

/* ====== 翻页键：PB10 下一页（低电平按下） ====== */
#define BTN_ACTIVE_LOW   1
#define BTN_NEXT_PORT    GPIOB
#define BTN_NEXT_PIN     GPIO_PIN_10

/* ====== 电机人工控制按键：PB11（低电平按下） ====== */
#define MOTOR_BTN_PORT   GPIOB
#define MOTOR_BTN_PIN    GPIO_PIN_11

/* ====== BT 缓冲（不加省略号） ====== */
static char g_bt_last[64] = "--";      // 仅在回车/换行或空闲超时后更新，BT页显示用
static volatile uint8_t bt_rx_byte;
static char             bt_line[64];   // 正在输入的一行（未提交前不显示）
static volatile uint8_t bt_line_len = 0;
static volatile uint32_t bt_rx_bytes = 0;
static volatile uint32_t bt_last_rx_ms = 0;
#define BT_COMMIT_IDLE_MS  400u

/* -------------------- 前置声明 -------------------- */
static void BT_UART_Start(void);
static void UART1_SetBaud(uint32_t baud);
static int  BT_AT_raw(const char* cmd, const char* ending, char* out, int max, uint32_t timeout_ms);
static void BT_AutoProbe(void);
static int  BT_SetName(const char* name);
static int  BT_SetAdvInterval(char code);
static void BT_Send(const char* s);
static void BT_SendLine(const char* s);

static void Buttons_Init(void);
static uint8_t NextPageButton_Scan10ms(void);

static void draw_centered6x8(int y, const char* s);
static void draw_centered6x8_ellipsized(int y, const char* s); // 备用
static void Beep_Tickle(void);
static inline void BEEP_SetFreq(uint32_t freq_hz);
static inline void BEEP_on(void);
static inline void BEEP_off(void);
static void PlayWelcomeMelody(uint32_t max_ms);
static inline void UI_NextPage(void){ g_page = (page_t)((g_page + 1) % PAGE_COUNT); }

/* =============================================================================
 *                         顶部右侧图标（8px 高）
 * ===========================================================================*/
#undef  SPEAKER_W
#define SPEAKER_W 12
#undef  SPEAKER_H
#define SPEAKER_H 8

/* 基础绘图小工具 */
static void draw_hline(int x,int y,int w){ for(int i=0;i<w;i++) SSD1306_DrawPixel(x+i,y,1); }
static void fill_rect(int x,int y,int w,int h){ for(int r=0;r<h;r++) draw_hline(x,y+r,w); }
static void clear_rect(int x,int y,int w,int h){
  for(int r=0;r<h;r++){
    for(int i=0;i<w;i++){ SSD1306_DrawPixel(x+i, y+r, 0); }
  }
}
static void draw_line(int x0,int y0,int x1,int y1){
  int dx = (x1>x0)? (x1-x0):(x0-x1), sx = (x0<x1)? 1:-1;
  int dy = -((y1>y0)? (y1-y0):(y0-y1)), sy = (y0<y1)? 1:-1;
  int err = dx + dy, e2;
  for(;;){
    SSD1306_DrawPixel(x0,y0,1);
    if (x0==x1 && y0==y1) break;
    e2 = 2*err;
    if (e2 >= dy){ err += dy; x0 += sx; }
    if (e2 <= dx){ err += dx; y0 += sy; }
  }
}

/* 扬声器图标（报警提示） */
static void Draw_SpeakerIcon(int x, int y){
  const int s = 2;
  const int square_x = x + 0;
  const int square_y = y + (SPEAKER_H - s)/2;
  const int rect_w = s;
  const int rect_h = 3 * s;
  const int rect_x = square_x + s;
  const int rect_y = square_y - (rect_h - s)/2;

  fill_rect(square_x, square_y, s, s);
  fill_rect(rect_x, rect_y, rect_w, rect_h);

  const int rx   = rect_x + rect_w;
  const int midy = rect_y + rect_h/2;
  const int Ls = 3;
  const int Lm = 5;
  draw_line(rx + 1, midy - 2, rx + 1 + Ls, midy - 3);
  draw_line(rx + 1, midy,     rx + 1 + Lm, midy);
  draw_line(rx + 1, midy + 2, rx + 1 + Ls, midy + 3);
}

/* 小风扇图标，phase 每次刷新+1 实现“旋转”动画 */
static void Draw_FanIcon_8x8(int x, int y, uint8_t phase){
  SSD1306_DrawPixel(x+4, y+4, 1);  // 中心点
  uint8_t p = phase & 0x03;
  const int dirs[4][3][2] = {
    {{ 0,-3},{ 3, 0},{-3, 0}},
    {{ 2,-2},{ 2, 2},{-2, 2}},
    {{ 0, 3},{ 3, 0},{-3, 0}},
    {{-2,-2},{ 2,-2},{-2, 2}},
  };
  for (int i=0;i<3;i++){
    int dx = dirs[p][i][0], dy = dirs[p][i][1];
    SSD1306_DrawPixel(x+4+dx, y+4+dy, 1);
    if (dx) SSD1306_DrawPixel(x+4+(dx>0?dx-1:dx+1), y+4+dy, 1);
    if (dy) SSD1306_DrawPixel(x+4+dx, y+4+(dy>0?dy-1:dy+1), 1);
  }
}

/* 手动模式图标（8x8，字母 'M'） */
static void Draw_ManualIcon_8x8(int x, int y){
  for (int i=0;i<8;i++){ // 立柱
    SSD1306_DrawPixel(x+1, y+i, 1);
    SSD1306_DrawPixel(x+6, y+i, 1);
  }
  // 顶部横线
  for (int i=1;i<=6;i++) SSD1306_DrawPixel(x+i, y, 1);
  // 中间形成 "M"
  SSD1306_DrawPixel(x+2, y+1, 1);
  SSD1306_DrawPixel(x+3, y+2, 1);
  SSD1306_DrawPixel(x+4, y+2, 1);
  SSD1306_DrawPixel(x+5, y+1, 1);
}

/* ===== ASCII/HEX 安全转换 ===== */
static void BT_SanitizeToAscii(const char* in, char* out, size_t outsz){
  if (!in || !out || outsz < 4) return;
  size_t n = 0; int has_ascii = 0; const unsigned char* s = (const unsigned char*)in;
  for (size_t i = 0; s[i] && n < outsz - 1; ++i){
    unsigned char c = s[i];
    if (c >= 32 && c < 127){ out[n++] = (char)c; has_ascii = 1; }
  }
  out[n] = 0;
  if (!has_ascii){
    n = (size_t)snprintf(out, outsz, "HEX:");
    for (size_t i=0; s[i] && i<20 && n < outsz-3; ++i){
      n += (size_t)snprintf(out+n, outsz-n, " %02X", s[i]);
    }
    out[n] = 0;
  }
}

/* ===== BT 两行显示（不加省略号，超出宽度直接裁掉） ===== */
static void draw_bt_two_lines(int y1, int y2){
  const int chars_per_line = SSD1306_WIDTH / 6; // 21
  const char* s = g_bt_last;
  char l1[32]={0}, l2[32]={0};
  size_t len = strlen(s);

  if (len == 0) { strcpy(l1, "--"); l2[0]=0; }
  else {
    strncpy(l1, s, chars_per_line);
    l1[chars_per_line] = 0;
    if (len > (size_t)chars_per_line){
      strncpy(l2, s + chars_per_line, chars_per_line);
      l2[chars_per_line] = 0;
    }
  }

  clear_rect(0, y1, SSD1306_WIDTH, 8);
  clear_rect(0, y2, SSD1306_WIDTH, 8);

  draw_centered6x8(y1, l1);
  if (l2[0]) draw_centered6x8(y2, l2);
}

/* ===== UART：接收回调（只在回车/换行时提交显示） ===== */
static void BT_UART_Start(void){
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&bt_rx_byte, 1);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (huart->Instance == USART1){
    char c = (char)bt_rx_byte;

    bt_rx_bytes++;                 /* 调试计数 */
    bt_last_rx_ms = HAL_GetTick(); /* 记录最近一次收到字节的时间 */

    if (c == '\r' || c == '\n'){
      bt_line[bt_line_len] = 0;
      if (bt_line_len == 0){ bt_line[0]='-'; bt_line[1]='-'; bt_line[2]=0; }

      /* 只有此处提交“最后一次发送”的显示文本（ASCII/HEX 安全） */
      BT_SanitizeToAscii(bt_line, g_bt_last, sizeof(g_bt_last));

      BT_SendLine(bt_line);  /* 回显整行到上位机 */
      bt_line_len = 0;
    }else{
      /* 仅缓存；不更新 g_bt_last。等回车或空闲超时再整体提交 */
      if (bt_line_len < (int)sizeof(bt_line)-1) bt_line[bt_line_len++] = c;
    }
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&bt_rx_byte, 1);
  }
}

/* ===== BT AT 与自动探测波特率 ===== */
static int BT_AT_raw(const char* cmd, const char* ending, char* out, int max, uint32_t timeout_ms){
  HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 200);
  if (ending && *ending) HAL_UART_Transmit(&huart1, (uint8_t*)ending, strlen(ending), 200);
  uint32_t t0 = HAL_GetTick();
  int i = 0;
  while((HAL_GetTick() - t0) < timeout_ms && i < max - 1){
    uint8_t c;
    if(HAL_UART_Receive(&huart1, &c, 1, 10) == HAL_OK){
      out[i++] = (char)c;
      if(c == '\n' || c == '\r') break;
    }
  }
  out[i] = 0;
  return i;
}
static void UART1_SetBaud(uint32_t baud){
  HAL_UART_DeInit(&huart1);
  huart1.Init.BaudRate = baud;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&bt_rx_byte, 1);
}
static void BT_AutoProbe(void){
  const uint32_t bauds[] = {9600, 115200, 57600, 38400};
  const char* ends[] = {"\r\n", "\r", ""};
  char resp[64];
  for (unsigned bi=0; bi<sizeof(bauds)/sizeof(bauds[0]); ++bi){
    UART1_SetBaud(bauds[bi]);
    HAL_Delay(80);
    for (unsigned ei=0; ei<sizeof(ends)/sizeof(ends[0]); ++ei){
      if (BT_AT_raw("AT", ends[ei], resp, sizeof(resp), 300) > 0){
        (void)BT_AT_raw("AT+VERSION", ends[ei], resp, sizeof(resp), 300);
        return;
      }
    }
  }
  UART1_SetBaud(9600);
}

/* ===== AT：改名/广播（保留，但默认不开机调用） ===== */
static int BT_SetName(const char* name){
  if(!name || !*name) return -1;
  char cmd[48], resp[64];
  const char* ends[] = {"\r\n", "\r", ""};

  /* 1) HC-05/BT08 风格：AT+NAME=xxx */
  snprintf(cmd, sizeof(cmd), "AT+NAME=%s", name);
  for (unsigned ei=0; ei<3; ++ei){
    if (BT_AT_raw(cmd, ends[ei], resp, sizeof(resp), 800) > 0) {
      BT_AT_raw("AT+RESET", ends[ei], resp, sizeof(resp), 800);
      return 0;
    }
  }

  /* 2) HC-06/HM-10 常见：AT+NAMExxx */
  snprintf(cmd, sizeof(cmd), "AT+NAME%s", name);
  for (unsigned ei=0; ei<3; ++ei){
    if (BT_AT_raw(cmd, ends[ei], resp, sizeof(resp), 800) > 0) {
      BT_AT_raw("AT+RESET", ends[ei], resp, sizeof(resp), 800);
      return 0;
    }
  }
  return -1;
}
static int BT_SetAdvInterval(char code){
  if(!((code>='0'&&code<='9')||(code>='A'&&code<='D'))) return -1;
  char cmd[32], resp[64];
  const char* ends[] = {"\r\n","\r",""};
  snprintf(cmd, sizeof(cmd), "AT+ADVI%c", code);
  for (unsigned ei=0; ei<3; ++ei){
    if (BT_AT_raw(cmd, ends[ei], resp, sizeof(resp), 800) > 0){
      BT_AT_raw("AT+RESET", ends[ei], resp, sizeof(resp), 800);
      return 0;
    }
  }
  return -1;
}

/* ===== 发送：MCU -> 蓝牙 ===== */
static void BT_Send(const char* s){
  if(!s) return;
  HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 200);
}
static void BT_SendLine(const char* s){
  BT_Send(s);
  const uint8_t nl[2] = {'\r','\n'};
  HAL_UART_Transmit(&huart1, (uint8_t*)nl, 2, 200);
}

/* ===== UI 小工具：6x8 字体整行居中 ===== */
static void draw_centered6x8(int y, const char* s){
  int w = SSD1306_StringWidth6x8(s);
  int x = (SSD1306_WIDTH - w)/2; if (x < 0) x = 0;
  SSD1306_DrawString(x, y, s);
}
/* 备用：单行省略号（BT页不用） */
static void draw_centered6x8_ellipsized(int y, const char* s){
  const int maxChars = SSD1306_WIDTH / 6; // 128/6 ≈ 21
  size_t len = strlen(s);
  if (len <= (size_t)maxChars){ draw_centered6x8(y, s); return; }
  char tmp[72];
  size_t keep = (maxChars > 3) ? (size_t)(maxChars - 3) : 0;
  if (keep > sizeof(tmp)-4) keep = sizeof(tmp)-4;
  strncpy(tmp, s, keep);
  tmp[keep] = '\0';
  strcat(tmp, "...");
  draw_centered6x8(y, tmp);
}

/* ===== 心跳 LED & 蜂鸣器（TIM3_CH2 @ PB5） ===== */
#define LED_PORT GPIOC
#define LED_PIN  GPIO_PIN_13
static inline void LED_off(void){ HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); }
static inline void LED_on(void){  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); }
static void LED_Init(void){
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = LED_PIN; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &g); LED_off();
}
static inline void BEEP_on(void){
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (arr + 1)/2);
}
static inline void BEEP_off(void){ __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); }
static inline void BEEP_SetFreq(uint32_t freq_hz){
  if (freq_hz < 400)  freq_hz = 400;
  if (freq_hz > 5000) freq_hz = 5000;
  uint32_t timclk = 1000000UL;      // PSC=71 => 1 MHz
  uint32_t arr = (timclk / freq_hz); if (arr) arr -= 1;
  __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (arr + 1)/2);
}
static void Beep_Tickle(void){
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, arr); HAL_Delay(180);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);   HAL_Delay(40);
  BEEP_SetFreq(2400); BEEP_on(); HAL_Delay(220); BEEP_off(); HAL_Delay(40);
}
static void Beep_Pattern(uint8_t times, uint16_t on_ms, uint16_t off_ms, uint16_t freq){
  BEEP_SetFreq(freq);
  for(uint8_t i=0;i<times;i++){ BEEP_on(); HAL_Delay(on_ms); BEEP_off(); HAL_Delay(off_ms); }
}

/* ===== 心跳 LED ===== */
static uint8_t  hb_on = 0;
static uint32_t hb_deadline = 0;
static uint32_t hb_next_start = 0;
static void Heartbeat_Task(uint32_t now_ms){
  if (!hb_on){
    if (now_ms >= hb_next_start){ LED_on(); hb_on = 1; hb_deadline = now_ms + HB_ON_MS; }
  }else{
    if (now_ms >= hb_deadline){ LED_off(); hb_on = 0; hb_next_start = now_ms + (HB_PERIOD_MS - HB_ON_MS); }
  }
}

/* ===== PB10/PB11 配置 ===== */
static void Buttons_Init(void){
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Mode  = GPIO_MODE_INPUT;
  g.Speed = GPIO_SPEED_FREQ_LOW;
#if BTN_ACTIVE_LOW
  g.Pull  = GPIO_PULLUP;
#else
  g.Pull  = GPIO_PULLDOWN;
#endif
  g.Pin   = BTN_NEXT_PIN;  HAL_GPIO_Init(BTN_NEXT_PORT, &g);   // PB10：翻页
  g.Pin   = MOTOR_BTN_PIN; HAL_GPIO_Init(MOTOR_BTN_PORT, &g);  // PB11：电机按钮
}
static uint8_t NextPageButton_Scan10ms(void){
  static uint8_t st=0, prev=0; static uint32_t t=0;
  uint32_t now = HAL_GetTick();
  uint8_t raw = ( BTN_ACTIVE_LOW ? (HAL_GPIO_ReadPin(BTN_NEXT_PORT,BTN_NEXT_PIN)==GPIO_PIN_RESET)
                                 : (HAL_GPIO_ReadPin(BTN_NEXT_PORT,BTN_NEXT_PIN)==GPIO_PIN_SET) );
  if (raw != st){ if (now - t >= 20){ st=raw; t=now; } } else { t=now; }
  uint8_t evt = 0; if (st && !prev) evt = 1; prev = st; return evt;
}

/* ===== 自检结果结构 ===== */
typedef struct {
  uint8_t oled_visual;
  uint8_t bh_found;
  uint8_t bh_addr;
  uint8_t bh_read_ok;
  uint8_t dht_ok;
  uint8_t buzzer_ok;
  uint16_t vdd_mv;
  uint8_t  vdd_ok;
} SelfTestResult;

/* ===== 自检：BH1750 读一次；DHT11 重试；BEEP 线路切换检测；BT不检测 ===== */
static void RunSelfTest(SelfTestResult* r){
  memset(r, 0, sizeof(*r));

  if (OLED_VISUAL_TEST_MS > 0){ SSD1306_Fill(1); SSD1306_Update(); HAL_Delay(OLED_VISUAL_TEST_MS); }
  SSD1306_Fill(0); SSD1306_Update(); r->oled_visual = 1;

  r->vdd_mv = Read_VDDA_mV();
  r->vdd_ok = (r->vdd_mv >= 3000 && r->vdd_mv <= 3600);

  r->bh_found = 0; r->bh_read_ok = 0; r->bh_addr = 0x00;
  uint8_t try_addrs[2] = { BH1750_ADDR_LO, BH1750_ADDR_HI };
  for (int i=0;i<2;i++){
    uint8_t a = try_addrs[i];
    if (SoftI2C_Ping(a)){
      r->bh_found = 1; r->bh_addr = a;
      if (BH1750_Init(a, BH1750_CONT_HIRES) == HAL_OK){
        HAL_Delay(180);
        float tmp;
        if (BH1750_ReadLux(&tmp) == HAL_OK){
          r->bh_read_ok = 1;
          g_bh1750_status = HAL_OK; g_last_lux = tmp;
        }
      }
      break;
    }
  }

  DHT11_DataTypeDef dtmp = {0};
  for (int i=0;i<3;i++){ if (DHT11_Read(&dtmp) == HAL_OK){ r->dht_ok = 1; break; } HAL_Delay(150); }

  {
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, arr); HAL_Delay(10);
    GPIO_PinState s_on = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);   HAL_Delay(10);
    GPIO_PinState s_off = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
    r->buzzer_ok = (s_on != s_off) ? 1 : 0;

    if (r->buzzer_ok){ BEEP_SetFreq(2000); BEEP_on(); HAL_Delay(120); BEEP_off(); }
    else             { Beep_Pattern(2, 60, 60, 1200); }
    HAL_Delay(20);
  }
}

/* ===== Lux 文本（1 位小数） ===== */
static void fmt_lux_1dp(char *buf, size_t n, float v){
  if (!buf || n < 8) return;
  int32_t lx10 = (int32_t)(v * 10 + (v >= 0 ? 0.5f : -0.5f));
  long ip = lx10 / 10; long fp = lx10 % 10; if (fp < 0) fp = -fp;
  snprintf(buf, n, "Lux: %ld.%ld lx", ip, fp);
}

/* ===== 开机曲：一闪一闪亮晶晶 ===== */
typedef struct { uint16_t f; uint8_t beats; } tw_note_t;
static void PlayWelcomeMelody(uint32_t max_ms){
  uint32_t target = max_ms;
  if (target < WELCOME_TUNE_MS_MIN) target = WELCOME_TUNE_MS_MIN;
  if (target > WELCOME_TUNE_MS_MAX) target = WELCOME_TUNE_MS_MAX;
  if (target > max_ms)              target = max_ms;
  Beep_Tickle();
  const tw_note_t song[] = {
    {523,1},{523,1},{784,1},{784,1},{880,1},{880,1},{784,2},
    {698,1},{698,1},{659,1},{659,1},{587,1},{587,1},{523,2},
  };
  const int N = (int)(sizeof(song)/sizeof(song[0]));
  uint32_t total_beats = 0; for (int i=0;i<N;i++) total_beats += song[i].beats;
  if (total_beats == 0) { HAL_Delay(target); return; }
  uint32_t beat_ms = target / total_beats; if (beat_ms < 110) beat_ms = 110;
  uint32_t spent = 0;
  for (int i=0;i<N;i++){
    uint32_t dur = beat_ms * song[i].beats;
    uint32_t on_ms = (dur * 85) / 100, off_ms = dur - on_ms;
    if (song[i].f){ BEEP_SetFreq(song[i].f); BEEP_on(); HAL_Delay(on_ms); BEEP_off(); }
    else { HAL_Delay(on_ms); }
    HAL_Delay(off_ms);
    spent += dur; if (spent >= target) break;
  }
  if (spent < target) HAL_Delay(target - spent);
}

/* ===== 时钟 & 错误处理 ===== */
void SystemClock_Config(void);
void Error_Handler(void);

/* ===== 温湿报警（≥/≤ 触发；迟滞+冷却） ===== */
typedef struct {
  uint8_t temp_abn; uint8_t humi_abn;
  uint32_t t_last; uint32_t h_last; uint32_t both_last;
} AlarmState;
static AlarmState g_alarm = {0,0,0,0,0};
static void Alarm_CheckAndBeep(const DHT11_DataTypeDef* d, uint32_t now_ms){
  if (!d) return;
  if (!g_alarm.temp_abn){
    if (d->temperature >= TEMP_HIGH || d->temperature <= TEMP_LOW) g_alarm.temp_abn = 1;
  }else if ((d->temperature <= (TEMP_HIGH - TEMP_HYST)) && (d->temperature >= (TEMP_LOW + TEMP_HYST))){
    g_alarm.temp_abn = 0;
  }
  if (!g_alarm.humi_abn){
    if (d->humidity >= HUMI_HIGH || d->humidity <= HUMI_LOW) g_alarm.humi_abn = 1;
  }else if ((d->humidity <= (HUMI_HIGH - HUMI_HYST)) && (d->humidity >= (HUMI_LOW + HUMI_HYST))){
    g_alarm.humi_abn = 0;
  }
  if (g_alarm.temp_abn && g_alarm.humi_abn){
    if (now_ms - g_alarm.both_last >= ALARM_COOLDOWN_MS){
      Beep_Pattern(5, 90, 60, 2700);
      g_alarm.both_last = g_alarm.t_last = g_alarm.h_last = now_ms;
    }
    return;
  }
  if (g_alarm.temp_abn && (now_ms - g_alarm.t_last >= ALARM_COOLDOWN_MS)){
    Beep_Pattern(2, 120, 120, 2400); g_alarm.t_last = now_ms;
  }
  if (g_alarm.humi_abn && (now_ms - g_alarm.h_last >= ALARM_COOLDOWN_MS)){
    Beep_Pattern(3, 110, 100, 2200); g_alarm.h_last = now_ms;
  }
}

/* =============================================================================
 *                          ★★★ 电机控制（TIM2_CH1@PA0）★★★
 * ===========================================================================*/
/* 模式说明：
 * - AUTO：温度>32℃ 开始启动并线性提升；38℃ 达到 100%
 * - MANUAL：短按 PB11 在开/关切换，不退出手动；长按 PB11（≥1.5s）退出手动回自动
 */
typedef enum { MOTOR_AUTO=0, MOTOR_MANUAL=1 } motor_mode_t;
static struct {
  motor_mode_t mode;     // 当前模式
  uint8_t      manual_on;// 人工模式下 1=100%，0=0%
  uint8_t      duty_pct; // 当前输出占空比（0~100）
} g_motor = {MOTOR_AUTO, 0, 0};

static inline void MOTOR_StartPWM(void){ HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); }
static inline void MOTOR_SetDutyPct(uint8_t pct){
  if (pct > 100) pct = 100;
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
  uint32_t ccr = (uint32_t)((arr + 1) * pct / 100);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
}
static uint8_t Motor_AutoDuty_FromTemp(int tempC){
  if (tempC <= 32) return 0;
  if (tempC >= 38) return 100;
  int delta = tempC - 32;                  // 1..6
  int duty  = 50 + (delta * 50) / 6;       // 32->~50%，38->100%
  if (duty < 0) duty = 0; if (duty > 100) duty = 100;
  return (uint8_t)duty;
}

/* ===== 修复版：每 10ms 调用一次，短按/长按事件去抖状态机 =====
 * 返回：0=无、1=短按、2=长按（长按只触发一次，直到松手不再有事件）
 */
static uint8_t MotorButton_Update10ms(void){
  static uint8_t  st  = 0;    // 去抖后的电平
  static uint8_t  fsm = 0;    // 0 idle, 1 pressed, 2 long_reported_wait_release
  static uint32_t t_edge = 0, t_press = 0;
  const uint32_t  LONG_MS = 1500;

  uint32_t now = HAL_GetTick();
  uint8_t raw = ( BTN_ACTIVE_LOW
                ? (HAL_GPIO_ReadPin(MOTOR_BTN_PORT, MOTOR_BTN_PIN)==GPIO_PIN_RESET)
                : (HAL_GPIO_ReadPin(MOTOR_BTN_PORT, MOTOR_BTN_PIN)==GPIO_PIN_SET) );

  // 简易去抖：20ms 稳定才改变 st
  if (raw != st){ if (now - t_edge >= 20){ st = raw; t_edge = now; } } else { t_edge = now; }

  switch (fsm){
    case 0: // idle
      if (st){ fsm = 1; t_press = now; }
      break;

    case 1: // 按下计时中
      if (st){
        if ((now - t_press) >= LONG_MS){
          fsm = 2;               // 长按已上报，直到松手不再有事件
          return 2;              // —— 长按事件（只触发一次）
        }
      }else{
        fsm = 0;
        return 1;                // —— 短按事件
      }
      break;

    case 2: // 等待松手
      if (!st) fsm = 0;
      break;
  }
  return 0;
}

/* =============================================================================
 *                                  主函数
 * ===========================================================================*/
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  SystemCoreClockUpdate();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  BT_UART_Start();
  MX_ADC1_Init();

  /* TIM2 用作电机 PWM（CubeMX 需已开启 TIM2 CH1@PA0） */
  MX_TIM2_Init();
  MX_TIM3_Init();                               // 蜂鸣器
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);     // 播欢迎曲前确保 PWM 已启
  MOTOR_StartPWM();
  MOTOR_SetDutyPct(0);                          // 初始停转

  /* 欢迎页 */
  SSD1306_Init();
  SSD1306_Fill(0);
  {
    const int scale_x = 2, scale_y = 1;
    const int line_h  = 8 * scale_y;
    const int gap     = 12;
    const int total_h = line_h * 2 + gap;
    const int lift    = 4;
    int y0_base = (SSD1306_HEIGHT - total_h) / 2;
    int y0      = y0_base - lift;
    if (y0 < 0) y0 = 0;
    SSD1306_DrawStringCenteredScaled(y0,                "Welcome to",            1, scale_x, scale_y);
    SSD1306_DrawStringCenteredScaled(y0 + line_h + gap, "AI Farmland Terminal",  1, scale_x, scale_y);
  }
  SSD1306_Update();
#if WELCOME_TUNE_ENABLE
  PlayWelcomeMelody(WELCOME_SHOW_MS);
#else
  HAL_Delay(WELCOME_SHOW_MS);
#endif

  /* 自检前准备 */
  HAL_Delay(10);
  SoftI2C_Begin();
  BT_AutoProbe();

#if BT_CFG_AT_ON_BOOT
  if (BT_SetName(BT_DEFAULT_NAME) == 0) strncpy(g_bt_last, "NAME OK", sizeof(g_bt_last)-1);
  else                                   strncpy(g_bt_last, "NAME ERR", sizeof(g_bt_last)-1);
  HAL_Delay(300); (void)BT_SetAdvInterval(BT_ADV_CODE); HAL_Delay(300);
#endif

  /* 自检 */
  SelfTestResult st = {0};
  RunSelfTest(&st);

  char line[64];
  SSD1306_Fill(0);
  SSD1306_DrawStringCenteredScaled(0, "Hardware Check", 1, 2, 1);
  snprintf(line, sizeof(line), "OLED %s", st.oled_visual ? "OK" : "ERR");           draw_centered6x8(20, line);
  snprintf(line, sizeof(line), "BH1750 %s  DHT11 %s",
           (st.bh_found && st.bh_read_ok) ? "OK" : "ERR", st.dht_ok ? "OK" : "ERR");draw_centered6x8(32, line);

  /*BT 行固定显示 “BT loading” */
  draw_centered6x8(44, "BT loading...");

  snprintf(line, sizeof(line), "VDD=%umV", (unsigned)st.vdd_mv);                     draw_centered6x8(56, line);
  SSD1306_Update();
  HAL_Delay(SELFTEST_SHOW_MS);

  /* 运行期变量 */
  Buttons_Init();
  BEEP_off();
  LED_Init();

  DHT11_DataTypeDef d = {0};
  uint8_t   have_valid_dht = st.dht_ok;
  uint32_t  next_dht_ms    = HAL_GetTick();
  const uint32_t DHT_PERIOD_MS = 2000;
  uint32_t  next_oled_ms   = HAL_GetTick();
  uint32_t  last_vdd_mv    = st.vdd_mv;
  HAL_StatusTypeDef last_dht_status = st.dht_ok ? HAL_OK : HAL_ERROR;

  if (!have_valid_dht){
    for (int i = 0; i < 5; i++) { if (DHT11_Read(&d) == HAL_OK){ have_valid_dht = 1; break; } HAL_Delay(250); }
  }

  static uint8_t fan_phase = 0;

  /* ============================== 主循环 ============================== */
  for (;;)
  {
    uint32_t now = HAL_GetTick();

    /* —— 按键 —— */
    static uint32_t next_btn_scan = 0;
    if (now >= next_btn_scan){
      next_btn_scan = now + 10;

      /* PB10: 下一页 */
      if (NextPageButton_Scan10ms() == 1) UI_NextPage();

      /* PB11: 电机按钮（短按=进入/留在手动并启停，长按=退出手动） */
      uint8_t mEvt = MotorButton_Update10ms();
      if (mEvt == 1){ // 短按
        if (g_motor.mode == MOTOR_AUTO){
          // 从自动 → 手动，并立即启动
          g_motor.mode = MOTOR_MANUAL;
          g_motor.manual_on = 1;
        }else{
          // 已在手动：启/停切换，不退出手动
          g_motor.manual_on = !g_motor.manual_on;
        }
      }else if (mEvt == 2){ // 长按：退出手动，回自动
        Beep_Pattern(1, 80, 0, 1800);
        g_motor.mode = MOTOR_AUTO;
        // 可选：退出时停转
        // g_motor.manual_on = 0;
      }
    }

    /* —— 传感器 —— */
    if (now >= g_next_lux_ms){
      float lx; HAL_StatusTypeDef stlux = BH1750_ReadLux(&lx);
      g_bh1750_status = stlux; if (stlux == HAL_OK) g_last_lux = lx;
      g_next_lux_ms = now + 500;
    }

    Heartbeat_Task(now);

    /* 供电监测 */
    last_vdd_mv = Read_VDDA_mV();
    uint8_t low_vdd = (last_vdd_mv < 3050);

    /* DHT11 每 2 秒采样一次（低压跳过） */
    if (!low_vdd && now >= next_dht_ms) {
      HAL_StatusTypeDef st_d2 = DHT11_Read(&d);
      if (st_d2 != HAL_OK) { HAL_Delay(200); st_d2 = DHT11_Read(&d); }
      last_dht_status = st_d2;
      if (st_d2 == HAL_OK) have_valid_dht = 1;
      next_dht_ms = now + DHT_PERIOD_MS;
    }

    /* 报警判定 */
    if (!low_vdd && have_valid_dht && last_dht_status == HAL_OK){
      Alarm_CheckAndBeep(&d, now);
    }

#if BT_DEMO_TX_ENABLE
    static uint32_t next_demo_tx = 0;
    if (now >= next_demo_tx){
      char msg[64]; int n = 0;
      n += snprintf(msg+n, sizeof(msg)-n, "VDD=%lu", (unsigned long)last_vdd_mv);
      if (have_valid_dht && last_dht_status==HAL_OK){
        n += snprintf(msg+n, sizeof(msg)-n, " T=%dC H=%d%%", d.temperature, d.humidity);
      }
      if (g_bh1750_status==HAL_OK){
        int lux = (int)(g_last_lux + 0.5f);
        n += snprintf(msg+n, sizeof(msg)-n, " L=%d", lux);
      }
      BT_SendLine(msg);
      next_demo_tx = now + BT_DEMO_PERIOD_MS;
    }
#endif

    /* ★ 无回车空闲超时提交一行到 BT 页显示 */
    if (bt_line_len > 0){
      if ((uint32_t)(HAL_GetTick() - bt_last_rx_ms) >= BT_COMMIT_IDLE_MS){
        bt_line[bt_line_len] = 0;
        BT_SanitizeToAscii(bt_line, g_bt_last, sizeof(g_bt_last));
        bt_line_len = 0;
      }
    }

    /* —— 电机控制 —— */
    uint8_t target = 0;
    if (g_motor.mode == MOTOR_MANUAL){
      target = g_motor.manual_on ? 100 : 0;
    }else{
      if (have_valid_dht && last_dht_status == HAL_OK) target = Motor_AutoDuty_FromTemp((int)d.temperature);
      else target = 0;
    }
    MOTOR_SetDutyPct(target);
    g_motor.duty_pct = target;

    /* —— OLED 刷新 —— */
    if (now >= next_oled_ms) {
      SSD1306_Fill(0);

      if (low_vdd) {
        draw_centered6x8(0,  "LOW VDD!");
        snprintf(line, sizeof(line), "VDD=%lumV", (unsigned long)last_vdd_mv);
        draw_centered6x8(16, line);
        draw_centered6x8(32, "Check 5V/3V3");
      } else {
        /* 顶部左：标题 */
        SSD1306_DrawString(0, 0, "Hello STM32");

        /* 顶部右：报警喇叭（最右） */
        int right_x = SSD1306_WIDTH - SPEAKER_W;
        if (g_alarm.temp_abn || g_alarm.humi_abn){
          Draw_SpeakerIcon(right_x, 0);   // 12x8
        }
        /* 顶部右：风扇图标（在喇叭左边 10px，只有占空比>0 时显示并转动） */
        if (g_motor.duty_pct > 0){
          Draw_FanIcon_8x8(right_x - 10, 0, fan_phase++);
        }
        /* 顶部右：手动模式“M”图标（在风扇左边 10px，仅手动模式显示） */
        if (g_motor.mode == MOTOR_MANUAL){
          Draw_ManualIcon_8x8(right_x - 20, 0);
        }

        switch (g_page) {
          case PAGE_ENV: {
            draw_centered6x8(16, "DHT11");
            clear_rect(0, 28, SSD1306_WIDTH, 8);
            clear_rect(0, 36, SSD1306_WIDTH, 8);
            if (have_valid_dht && last_dht_status == HAL_OK) {
              snprintf(line, sizeof(line), "Tem:%2d C", d.temperature); draw_centered6x8(28, line);
              snprintf(line, sizeof(line), "Hum:%2d %%", d.humidity);   draw_centered6x8(36, line);
            } else {
              draw_centered6x8(28, "NO DHT11");
              draw_centered6x8(36, "or wiring error");
            }
            break;
          }
          case PAGE_LUX: {
            draw_centered6x8(16, "BH1750");
            clear_rect(0, 28, SSD1306_WIDTH, 8);
            clear_rect(0, 36, SSD1306_WIDTH, 8);
            if (g_bh1750_status == HAL_OK){
              fmt_lux_1dp(line, sizeof(line), g_last_lux); draw_centered6x8(28, line);
            }else{
              draw_centered6x8(28, "BH1750 N/A");
              draw_centered6x8(36, "Check ADDR/I2C");
            }
            break;
          }
          case PAGE_BT: {
            draw_centered6x8(16, "BT");
            draw_bt_two_lines(28, 36); // 两行空间
            clear_rect(0, 44, SSD1306_WIDTH, 8);
            snprintf(line, sizeof(line), "RX=%lu  Baud:%lu",
                     (unsigned long)bt_rx_bytes, (unsigned long)huart1.Init.BaudRate);
            draw_centered6x8(44, line);
            break;
          }
          default: break;
        }

        /* 底部 VDD 居中（所有页面共用） */
        snprintf(line, sizeof(line), "VDD=%lu mV", (unsigned long)last_vdd_mv);
        draw_centered6x8(56, line);
      }

      SSD1306_Update();
      next_oled_ms = now + OLED_REFRESH_MS;
    }

    HAL_Delay(5);
  }
}

/* ===== 时钟配置（与工程一致） ===== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}
void Error_Handler(void){
  __disable_irq();
  while(1){ HAL_GPIO_TogglePin(LED_PORT, LED_PIN); HAL_Delay(100); }
}
