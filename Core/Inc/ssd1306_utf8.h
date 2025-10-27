#ifndef __SSD1306_UTF8_H__
#define __SSD1306_UTF8_H__

#include <stdint.h>

/* 渲染到现有 ssd1306 帧缓冲（已由 ssd1306.c 暴露 DrawPixel / String 宏族） */
#ifdef __cplusplus
extern "C" {
#endif

/* 单字宽度：ASCII=6，CJK=16 */
int  OLED_UTF8_CharWidth(const char* s);

/* 计算一段 UTF8 文本的显示宽度（像素），遇 '\n' 停止 */
int  OLED_UTF8_LineWidth(const char* s);

/* 从 (x,y) 开始绘制一行 UTF8 文本（ASCII 6×8；中文 16×16）。超出右边界自动截断 */
void OLED_DrawUTF8_Line(int x, int y, const char* s);

/* 居中绘制一行 UTF8 文本 */
void OLED_DrawUTF8_Line_Centered(int y, const char* s);

/* 将 UTF8 文本切分成两行，宽度不超过屏宽；返回第二行起始指针（可能为 NULL） */
const char* OLED_UTF8_SplitTwoLines(const char* s);

#ifdef __cplusplus
}
#endif
#endif
