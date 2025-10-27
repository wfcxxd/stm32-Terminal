#include "ssd1306.h"
#include "ssd1306_utf8.h"
#include <string.h>

/* ============ 小型 16x16 汉字点阵（逐步扩充） ============ */
/* 每字 16x16，逐行自上而下，每行 16bit → 2 字节，共 32 字节。1=点亮 */
typedef struct { uint32_t ucs; const uint8_t bmp[32]; } CJK16_t;

/* 占位符 “□” */
static const uint8_t GLYPH_BOX_16[32] = {
  0xFF,0xFF, 0x80,0x01, 0x80,0x01, 0x80,0x01,
  0x80,0x01, 0x80,0x01, 0x80,0x01, 0x80,0x01,
  0x80,0x01, 0x80,0x01, 0x80,0x01, 0x80,0x01,
  0x80,0x01, 0x80,0x01, 0x80,0x01, 0xFF,0xFF
};

/* 下面示例性放少数几个常用字（“你 好 中 文 温 湿 度 报 警 蓝 牙 电 压 接 收 发 送 页 面”）
 * 点阵可逐步补齐；为了演示，这里给出简化笔画（不会很漂亮，但可读）。
 * 如需精准宋体/点阵，可用 PC 字库工具导出 16x16 位图后粘贴到此处。
 */
static const CJK16_t CJK16_TABLE[] = {
  /* “你” U+4F60（示意版） */
  {0x4F60,{0x00,0x10,0x00,0x10,0x7F,0xFE,0x00,0x10,0x00,0x10,0x1F,0xF0,0x00,0x20,0x00,0x40,
           0x7F,0xFE,0x00,0x80,0x01,0x00,0x06,0x00,0x18,0x00,0x60,0x00,0x01,0x80,0x00,0x60}},
  /* “好” U+597D */
  {0x597D,{0x00,0x08,0x3F,0xFC,0x20,0x08,0x20,0x08,0x3F,0xF8,0x20,0x08,0x20,0x08,0x3F,0xF8,
           0x04,0x00,0x04,0x00,0x24,0x00,0x24,0x00,0x44,0x00,0x84,0x00,0x04,0x00,0x04,0x00}},
  /* “中” U+4E2D */
  {0x4E2D,{0x00,0x00,0x01,0x00,0x01,0x00,0x7F,0xFE,0x41,0x02,0x41,0x02,0x41,0x02,0x7F,0xFE,
           0x41,0x02,0x41,0x02,0x41,0x02,0x7F,0xFE,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00}},
  /* “文” U+6587 */
  {0x6587,{0x00,0x00,0x00,0x80,0x40,0x80,0x20,0x80,0x10,0x80,0x0F,0xFE,0x00,0x80,0x01,0x80,
           0x02,0x80,0x0C,0x80,0x30,0x80,0xC0,0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,0x00}},
  /* ……此处省略若干：温 U+6E29、湿 U+6E7F、度 U+5EA6、报 U+62A5、警 U+8B66、
     蓝 U+84DD、牙 U+7259、电 U+7535、压 U+538B、接 U+63A5、收 U+6536、
     发 U+53D1、送 U+9001、页 U+9875、面 U+9762 */
};

/* 查找 16x16 点阵；找不到就返回占位符 */
static const uint8_t* find_cjk16(uint32_t ucs){
  for (unsigned i=0;i<sizeof(CJK16_TABLE)/sizeof(CJK16_TABLE[0]);++i){
    if (CJK16_TABLE[i].ucs == ucs) return CJK16_TABLE[i].bmp;
  }
  return GLYPH_BOX_16;
}

/* 取一个 UTF8 码点；返回码点和消耗的字节数（非法则按单字节回退） */
static int utf8_next(const char* s, uint32_t* out_ucs){
  const unsigned char* p=(const unsigned char*)s;
  if (p[0]<0x80){ *out_ucs=p[0]; return 1; }
  if ((p[0]&0xE0)==0xC0 && (p[1]&0xC0)==0x80){
    *out_ucs=((p[0]&0x1F)<<6) | (p[1]&0x3F); return 2;
  }
  if ((p[0]&0xF0)==0xE0 && (p[1]&0xC0)==0x80 && (p[2]&0xC0)==0x80){
    *out_ucs=((p[0]&0x0F)<<12) | ((p[1]&0x3F)<<6) | (p[2]&0x3F); return 3;
  }
  /* 更长的我们也先不支持，回退成单字节 */
  *out_ucs = p[0]; return 1;
}

int OLED_UTF8_CharWidth(const char* s){
  if(!s||!*s) return 0;
  uint32_t u; int n = utf8_next(s,&u);
  (void)n;
  if (u < 0x80) return 6;     /* ASCII 6x8 */
  return 16;                  /* CJK 16x16 */
}

int OLED_UTF8_LineWidth(const char* s){
  int w=0; if(!s) return 0;
  while(*s && *s!='\n'){
    int cw = OLED_UTF8_CharWidth(s);
    w += cw;
    uint32_t u; int n = utf8_next(s,&u);
    s += n;
  }
  return w;
}

static void draw_cjk16(int x,int y,const uint8_t bmp[32]){
  for(int row=0; row<16; ++row){
    uint8_t b0=bmp[row*2], b1=bmp[row*2+1];
    for(int col=0; col<8; ++col){
      if (b0 & (0x80>>col)) SSD1306_DrawPixel(x+col,   y+row, 1);
      if (b1 & (0x80>>col)) SSD1306_DrawPixel(x+8+col, y+row, 1);
    }
  }
}

void OLED_DrawUTF8_Line(int x, int y, const char* s){
  int cx = x; if(!s) return;
  while(*s && *s!='\n'){
    uint32_t u; int n = utf8_next(s,&u);
    if (u < 0x80){           /* ASCII：复用 6×8 字符 */
      SSD1306_DrawChar(cx, y, (char)u);
      cx += 6;
    }else{
      const uint8_t* bmp = find_cjk16(u);
      if (cx + 16 > SSD1306_WIDTH) break;  /* 右侧裁掉 */
      draw_cjk16(cx, y - 4, bmp);          /* 中文整体更高，向上提 4px 便于与 6×8 对齐 */
      cx += 16;
    }
    if (cx >= SSD1306_WIDTH) break;
    s += n;
  }
}

void OLED_DrawUTF8_Line_Centered(int y, const char* s){
  int w = OLED_UTF8_LineWidth(s);
  int x = (SSD1306_WIDTH - w)/2; if (x<0) x=0;
  OLED_DrawUTF8_Line(x, y, s);
}

/* 拆两行：保证第一行宽度不超过屏宽，第二行返回其起始指针（可能为 NULL） */
const char* OLED_UTF8_SplitTwoLines(const char* s){
  if(!s) return NULL;
  int w=0; const char* p=s; const char* last_fit = NULL;
  while (*p && *p!='\n'){
    int cw = OLED_UTF8_CharWidth(p);
    if (w + cw > SSD1306_WIDTH) break;
    uint32_t u; int n = utf8_next(p,&u);
    w += cw; last_fit = p + n; p += n;
  }
  return (*p) ? last_fit : NULL;
}
