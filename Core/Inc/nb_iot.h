#ifndef NB_IOT_H
#define NB_IOT_H

#include "main.h"
#include "usart.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 简单 NB 连接状态 */
typedef struct {
  uint8_t inited;   /* AT & PDP & UDP 是否完成 */
  uint8_t opened;   /* UDP socket 是否打开       */
} NB_State_t;

extern NB_State_t g_nb;

/* 初始化：上电后握手 + 附着 + 设置 APN + 打开 UDP
 *  apn  : 例如 "cmiot"（按你的 NB 卡运营商）
 *  ip   : 你的服务器公网 IP 或域名（建议先用 IP）
 *  port : 服务器 UDP 端口
 * 返回：0 成功；<0 失败
 */
int NB_Init(const char* apn, const char* ip, uint16_t port);

/* 发送一行文本到 UDP（自动在末尾追加 \r\n） */
int NB_SendLine(const char* line);

/* 非阻塞读取一行（\r 或 \n 结束）。
 *  用于调试或读取 URC。超时返回已读长度（可为 0）。
 */
int NB_ReadLine(char* out, int max, uint32_t tout_ms);

#ifdef __cplusplus
}
#endif

#endif /* NB_IOT_H */
