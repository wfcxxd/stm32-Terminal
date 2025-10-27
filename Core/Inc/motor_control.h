#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_MODE_AUTO = 0,
    MOTOR_MODE_MANUAL = 1
} motor_mode_t;

/* 初始化：启动 TIM2_CH1 PWM，默认占空比 0% */
void Motor_Init(void);

/* 10ms 调一次，读取 PB11：短按 切换人工开/关；长按(≥1.5s) 回到自动 */
void Motor_ButtonPoll10ms(void);

/* 自动模式下：根据温度更新占空比；无有效温度时=0% */
void Motor_Update_FromTemp(int tempC, uint8_t has_valid_temp);

/* 获取当前占空比(0~100)、模式、以及手动开关状态（手动模式下是否“开”） */
uint8_t     Motor_GetDuty(void);
motor_mode_t Motor_GetMode(void);
uint8_t     Motor_ManualIsOn(void);

/* （可选）直接切换模式与手动状态 */
void Motor_SetMode(motor_mode_t m);
void Motor_SetManualOn(uint8_t on);

#ifdef __cplusplus
}
#endif
#endif
