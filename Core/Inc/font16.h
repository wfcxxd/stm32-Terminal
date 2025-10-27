#ifndef FONT16_H
#define FONT16_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 返回给定 Unicode 码点的 16×16 点阵（行优先，共 32 字节）。
 * 找不到则返回 NULL（由 main.c 用“□”占位显示）。 */
const uint8_t* Font16_FindGlyph(uint32_t ucs);

#ifdef __cplusplus
}
#endif
#endif
