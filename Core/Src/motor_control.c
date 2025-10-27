#include "motor_control.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "gpio.h"

/* ===== 按钮脚（CubeMX 已设：PB11 上拉输入，低电平=按下） ===== */
#define MOTOR_BTN_PORT   GPIOB
#define MOTOR_BTN_PIN    GPIO_PIN_11
#define BTN_ACTIVE_LOW   1

/* ===== 长按阈值（毫秒） ===== */
#define LONG_PRESS_MS    1500u

/* ===== 内部状态 ===== */
static motor_mode_t s_mode = MOTOR_MODE_AUTO;
static uint8_t      s_manual_on = 0;  // 手动模式下 1=100% / 0=0%
static uint8_t      s_duty = 0;       // 当前实际占空比(0~100)

static void prv_set_duty(uint8_t pct)
{
    if (pct > 100) pct = 100;
    s_duty = pct;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t ccr = (uint32_t)((arr + 1) * pct / 100);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
}

static uint8_t prv_auto_duty_from_temp(int tempC)
{
    if (tempC <= 32) return 0;
    if (tempC >= 38) return 100;
    /* 32~38°C: 从 50% 线性升到 100%（跨度 6°C） */
    int delta = tempC - 32;           // 1..5
    int duty  = 50 + (delta * 50) / 6;
    if (duty < 0) duty = 0; if (duty > 100) duty = 100;
    return (uint8_t)duty;
}

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    prv_set_duty(0);  // 开机默认停转
}

void Motor_ButtonPoll10ms(void)
{
    static uint8_t  st = 0, pressed = 0;
    static uint32_t t_edge = 0, t_press = 0;
    uint32_t now = HAL_GetTick();

    uint8_t raw = (BTN_ACTIVE_LOW
                  ? (HAL_GPIO_ReadPin(MOTOR_BTN_PORT, MOTOR_BTN_PIN) == GPIO_PIN_RESET)
                  : (HAL_GPIO_ReadPin(MOTOR_BTN_PORT, MOTOR_BTN_PIN) == GPIO_PIN_SET));

    /* 简单去抖：要求 20ms 稳定 */
    if (raw != st) {
        if (now - t_edge >= 20) { st = raw; t_edge = now; }
    } else {
        t_edge = now;
    }

    if (st && !pressed) { pressed = 1; t_press = now; }
    if (pressed) {
        if (st && (now - t_press >= LONG_PRESS_MS)) {
            /* 长按事件：返回自动模式 */
            s_mode = MOTOR_MODE_AUTO;
            pressed = 0;
            return;
        }
        if (!st) {
            /* 短按释放：进入/保持手动模式，并在开/关间切换 */
            s_mode = MOTOR_MODE_MANUAL;
            s_manual_on = !s_manual_on;
            pressed = 0;
            return;
        }
    }
}

void Motor_Update_FromTemp(int tempC, uint8_t has_valid_temp)
{
    uint8_t target = 0;
    if (s_mode == MOTOR_MODE_MANUAL) {
        target = s_manual_on ? 100 : 0;
    } else {
        target = has_valid_temp ? prv_auto_duty_from_temp(tempC) : 0;
    }
    prv_set_duty(target);
}

uint8_t Motor_GetDuty(void)      { return s_duty; }
motor_mode_t Motor_GetMode(void) { return s_mode; }
uint8_t Motor_ManualIsOn(void)   { return s_manual_on; }
void Motor_SetMode(motor_mode_t m){ s_mode = m; }
void Motor_SetManualOn(uint8_t on){ s_manual_on = (on ? 1 : 0); }
