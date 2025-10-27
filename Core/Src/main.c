
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
#include "nb_iot.h"
#include "bh1750.h"
#include "stm32_init.h"   // Read_VDDA_mV()

/* =============================================================================
 *                      配置与说明（页面顺序：ENV -> LUX -> NB）
 * =============================================================================
 * - 开机默认在 ENV 页
 * - NB 页：两行文本窗口（不省略，超宽直接裁切）
 * - 每 10 秒向 NB（UDP）上报一行状态文本（VDD/温湿/光照，若可用）
 * - 电机：PB11 短按=进入/留在手动并启停；长按=退出手动回自动
 * - 顶部右侧：喇叭（告警），左 10px 风扇（占空比>0 时转），再左 10px “M”手动指示
 * - PWM 使用 TIM2_CH1@PA0（需在 CubeMX 里启用）
 * ============================================================================= */

#define WELCOME_SHOW_MS       2000u
#define SELFTEST_SHOW_MS      2000u
#define OLED_REFRESH_MS       350u
#define HB_PERIOD_MS          2000u
#define HB_ON_MS              60u

/* ======== 温湿报警参数（可按需修改） ======== */
#define TEMP_HIGH             35
#define TEMP_LOW              24
#define TEMP_HYST              1
#define HUMI_HIGH             80
#define HUMI_LOW              35
#define HUMI_HYST              3
#define ALARM_COOLDOWN_MS  30000u

/* 页面定义（顺序保持：ENV -> LUX -> NB） */
typedef enum { PAGE_ENV = 0, PAGE_LUX = 1, PAGE_NB = 2, PAGE_COUNT = 3 } page_t;
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

/* ====== NB 文本缓冲（不加省略号） ====== */
static char g_nb_last[64] = "--";

/* -------------------- 前置声明 -------------------- */
static void Buttons_Init(void);
static uint8_t NextPageButton_Scan10ms(void);
static void draw_centered6x8(int y, const char* s);
static void clear_rect(int x,int y,int w,int h);
static void draw_line(int x0,int y0,int x1,int y1);
static inline void BEEP_SetFreq(uint32_t freq_hz);
static inline void BEEP_on(void);
static inline void BEEP_off(void);
static void Beep_Pattern(uint8_t times, uint16_t on_ms, uint16_t off_ms, uint16_t freq);
static inline void UI_NextPage(void){ g_page = (page_t)((g_page + 1) % PAGE_COUNT); }

/* =============================================================================
 *                         顶部右侧图标（8px 高）
 * ===========================================================================*/
#define SPEAKER_W 12
#define SPEAKER_H 8

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
static void Draw_FanIcon_8x8(int x, int y, uint8_t phase){
  SSD1306_DrawPixel(x+4, y+4, 1);
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
static void Draw_ManualIcon_8x8(int x, int y){
  for (int i=0;i<8;i++){
    SSD1306_DrawPixel(x+1, y+i, 1);
    SSD1306_DrawPixel(x+6, y+i, 1);
  }
  for (int i=1;i<=6;i++) SSD1306_DrawPixel(x+i, y, 1);
  SSD1306_DrawPixel(x+2, y+1, 1);
  SSD1306_DrawPixel(x+3, y+2, 1);
  SSD1306_DrawPixel(x+4, y+2, 1);
  SSD1306_DrawPixel(x+5, y+1, 1);
}

/* 两行显示（裁切不省略） */
static void draw_nb_two_lines(int y1, int y2){
  const int chars_per_line = SSD1306_WIDTH / 6; // 21
  const char* s = g_nb_last;
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

/* ===== UI 小工具：6x8 字体整行居中 ===== */
static void draw_centered6x8(int y, const char* s){
  int w = SSD1306_StringWidth6x8(s);
  int x = (SSD1306_WIDTH - w)/2; if (x < 0) x = 0;
  SSD1306_DrawString(x, y, s);
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
static void Beep_Pattern(uint8_t times, uint16_t on_ms, uint16_t off_ms, uint16_t freq){
  BEEP_SetFreq(freq);
  for(uint8_t i=0;i<times;i++){ BEEP_on(); HAL_Delay(on_ms); BEEP_off(); HAL_Delay(off_ms); }
}
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
  g.Pin   = BTN_NEXT_PIN;  HAL_GPIO_Init(BTN_NEXT_PORT, &g);
  g.Pin   = MOTOR_BTN_PIN; HAL_GPIO_Init(MOTOR_BTN_PORT, &g);
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

/* ===== 自检：BH1750 读一次；DHT11 重试；蜂鸣器拨测 ===== */
static void RunSelfTest(SelfTestResult* r){
  memset(r, 0, sizeof(*r));
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

  // 蜂鸣器通断（简单拨测）
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, arr); HAL_Delay(10);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);   HAL_Delay(10);
  r->buzzer_ok = 1;
}

/* ===== Lux 文本（1 位小数） ===== */
static void fmt_lux_1dp(char *buf, size_t n, float v){
  if (!buf || n < 8) return;
  int32_t lx10 = (int32_t)(v * 10 + (v >= 0 ? 0.5f : -0.5f));
  long ip = lx10 / 10; long fp = lx10 % 10; if (fp < 0) fp = -fp;
  snprintf(buf, n, "Lux: %ld.%ld lx", ip, fp);
}

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

/* ======================= 电机控制（TIM2_CH1@PA0） ======================= */
typedef enum { MOTOR_AUTO=0, MOTOR_MANUAL=1 } motor_mode_t;
static struct {
  motor_mode_t mode;
  uint8_t      manual_on;
  uint8_t      duty_pct;
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
  int delta = tempC - 32;
  int duty  = 50 + (delta * 50) / 6;
  if (duty < 0) duty = 0; if (duty > 100) duty = 100;
  return (uint8_t)duty;
}
static uint8_t MotorButton_Update10ms(void){
  static uint8_t  st  = 0;
  static uint8_t  fsm = 0;
  static uint32_t t_edge = 0, t_press = 0;
  const uint32_t  LONG_MS = 1500;
  uint32_t now = HAL_GetTick();
  uint8_t raw = ( BTN_ACTIVE_LOW
                ? (HAL_GPIO_ReadPin(MOTOR_BTN_PORT, MOTOR_BTN_PIN)==GPIO_PIN_RESET)
                : (HAL_GPIO_ReadPin(MOTOR_BTN_PORT, MOTOR_BTN_PIN)==GPIO_PIN_SET) );
  if (raw != st){ if (now - t_edge >= 20){ st = raw; t_edge = now; } } else { t_edge = now; }
  switch (fsm){
    case 0:
      if (st){ fsm = 1; t_press = now; }
      break;
    case 1:
      if (st){
        if ((now - t_press) >= LONG_MS){ fsm = 2; return 2; }
      }else{ fsm = 0; return 1; }
      break;
    case 2:
      if (!st) fsm = 0;
      break;
  }
  return 0;
}

/* ================================ 主函数 ================================ */
void SystemClock_Config(void);
void Error_Handler(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  SystemCoreClockUpdate();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  MOTOR_StartPWM();
  MOTOR_SetDutyPct(0);

  /* OLED 欢迎页 */
  SSD1306_Init();
  SSD1306_Fill(0);
  SSD1306_DrawStringCenteredScaled(18, "Welcome", 1, 2, 1);
  SSD1306_DrawStringCenteredScaled(34, "AI Farmland", 1, 2, 1);
  SSD1306_Update();
  HAL_Delay(WELCOME_SHOW_MS);

  /* NB 初始化（按需修改 APN/IP/PORT） */
  NB_Init("cmiot","1.2.3.4",9001);

  /* 自检 */
  SoftI2C_Begin();
  SelfTestResult st = {0};
  RunSelfTest(&st);
  char line[64];
  SSD1306_Fill(0);
  SSD1306_DrawStringCenteredScaled(0, "Hardware Check", 1, 2, 1);
  snprintf(line, sizeof(line), "OLED %s", st.oled_visual ? "OK" : "ERR");           draw_centered6x8(20, line);
  snprintf(line, sizeof(line), "BH1750 %s  DHT11 %s",
           (st.bh_found && st.bh_read_ok) ? "OK" : "ERR", st.dht_ok ? "OK" : "ERR");draw_centered6x8(32, line);
  draw_centered6x8(44, "NB loading...");
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

  /* 主循环 */
  uint32_t next_demo_tx = HAL_GetTick();
  for (;;){
    uint32_t now = HAL_GetTick();

    /* —— 按键 —— */
    static uint32_t next_btn_scan = 0;
    if (now >= next_btn_scan){
      next_btn_scan = now + 10;
      if (NextPageButton_Scan10ms() == 1) UI_NextPage();
      uint8_t mEvt = MotorButton_Update10ms();
      if (mEvt == 1){ // 短按
        if (g_motor.mode == MOTOR_AUTO){ g_motor.mode = MOTOR_MANUAL; g_motor.manual_on = 1; }
        else { g_motor.manual_on = !g_motor.manual_on; }
      }else if (mEvt == 2){ // 长按：退出手动
        Beep_Pattern(1, 80, 0, 1800);
        g_motor.mode = MOTOR_AUTO;
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

    /* 周期上报到 NB */
    if (now - next_demo_tx >= 10000){
      char msg[64]; int n = 0;
      n += snprintf(msg+n, sizeof(msg)-n, "VDD=%lu", (unsigned long)last_vdd_mv);
      if (have_valid_dht && last_dht_status==HAL_OK){
        n += snprintf(msg+n, sizeof(msg)-n, " T=%dC H=%d%%", d.temperature, d.humidity);
      }
      if (g_bh1750_status==HAL_OK){
        int lux = (int)(g_last_lux + 0.5f);
        n += snprintf(msg+n, sizeof(msg)-n, " L=%d", lux);
      }
      NB_SendLine(msg);
      strncpy(g_nb_last, msg, sizeof(g_nb_last)-1);
      g_nb_last[sizeof(g_nb_last)-1] = 0;
      next_demo_tx = now;
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
        SSD1306_DrawString(0, 0, "Hello STM32");
        int right_x = SSD1306_WIDTH - SPEAKER_W;
        if (g_alarm.temp_abn || g_alarm.humi_abn){ Draw_SpeakerIcon(right_x, 0); }
        if (g_motor.duty_pct > 0){ Draw_FanIcon_8x8(right_x - 10, 0, fan_phase++); }
        if (g_motor.mode == MOTOR_MANUAL){ Draw_ManualIcon_8x8(right_x - 20, 0); }

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
          case PAGE_NB: {
            draw_centered6x8(16, "NB");
            draw_nb_two_lines(28, 36);
            clear_rect(0, 44, SSD1306_WIDTH, 8);
            snprintf(line, sizeof(line), "VDD=%lu mV", (unsigned long)last_vdd_mv);
            draw_centered6x8(44, line);
            break;
          }
          default: break;
        }

        snprintf(line, sizeof(line), "VDD=%lu mV", (unsigned long)last_vdd_mv);
        draw_centered6x8(56, line);
      }
      SSD1306_Update();
      next_oled_ms = now + OLED_REFRESH_MS;
    }
    HAL_Delay(5);
  }
}

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
