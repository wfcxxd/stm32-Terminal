#ifndef __SSD1306_H
#define __SSD1306_H

#include "stm32f1xx_hal.h"
#include "stm32_init.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT   64
#define SSD1306_PAGES   (SSD1306_HEIGHT/8)

#ifndef SSD1306_COL_OFFSET
#define SSD1306_COL_OFFSET 0
#endif

void SSD1306_Init(void);
void SSD1306_Update(void);
void SSD1306_Fill(uint8_t on);
void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t on);
void SSD1306_DrawChar(int x,int y,char c);
void SSD1306_DrawString(int x,int y,const char* s);
void SSD1306_DrawCharScaled(int x,int y,char c, uint8_t sx, uint8_t sy, uint8_t bold);
void SSD1306_DrawStringScaled(int x,int y,const char* s, uint8_t sx, uint8_t sy, uint8_t bold);
int  SSD1306_StringWidth6x8(const char* s); /* 以6x8基准测单行宽（像素） */
void SSD1306_DrawStringCenteredScaled(int y,const char* s, uint8_t sx, uint8_t sy, uint8_t bold);

#ifdef __cplusplus
}
#endif
#endif
