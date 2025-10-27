#include "nb_iot.h"
#include <string.h>
#include <stdio.h>

/* 使用 USART1 与 BC260Y-CN 通讯（与你原工程一致） */
NB_State_t g_nb = {0,0};

/* ---- 串口基础 ---- */
static int uart_send_str(const char* s){
  if(!s) return -1;
  return (HAL_UART_Transmit(&huart1,(uint8_t*)s,strlen(s),500)==HAL_OK)?0:-1;
}
static int uart_send_bytes(const uint8_t* b, uint16_t n){
  return (HAL_UART_Transmit(&huart1,(uint8_t*)b,n,1000)==HAL_OK)?0:-1;
}

/* 非阻塞读一行：以 \\r 或 \\n 结束，超时返回已读长度（可为 0） */
int NB_ReadLine(char* out, int max, uint32_t tout_ms){
  if(!out || max<=1) return -1;
  uint32_t t0 = HAL_GetTick();
  int i = 0;
  while ((HAL_GetTick() - t0) < tout_ms){
    uint8_t ch;
    if (HAL_UART_Receive(&huart1, &ch, 1, 10) == HAL_OK){
      if(ch=='\\r' || ch=='\\n'){
        if(i==0) continue; // 跳过连续\\r\\n
        break;
      }
      if(i < max-1) out[i++] = (char)ch;
    }
  }
  out[i] = 0;
  return i;
}

/* 等待 AT 返回中包含某子串（或 ERROR），并支持收集回显 */
static int at_wait(const char* expect, uint32_t tout_ms, char* echo, int echo_sz){
  char line[160]; line[0]=0;
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < tout_ms){
    int n = NB_ReadLine(line, sizeof(line), 200);
    if(n <= 0) continue;
    if(echo && echo_sz>0){
      strncat(echo, line, echo_sz-1);
      strncat(echo, "\\n",  echo_sz-1);
    }
    if (strstr(line, expect))  return 0;
    if (strstr(line, "ERROR")) return -2;
    if (strstr(line, "+CME ERROR")) return -3;
  }
  return -4; // timeout
}
static int at_cmd(const char* cmd, const char* expect, uint32_t tout_ms){
  static const char crlf[] = "\\r\\n";
  (void)uart_send_str(cmd);
  (void)uart_send_bytes((const uint8_t*)crlf,2);
  return at_wait(expect, tout_ms, NULL, 0);
}

/* 打开 UDP：Socket id=1，profile=1（BC260Y/Quectel 风格） */
static int nb_open_udp(const char* ip, uint16_t port){
  char cmd[112];
  (void)at_cmd("AT+QICLOSE=1","OK",1000);  // 先尝试关闭旧的，不影响
  snprintf(cmd,sizeof(cmd),"AT+QIOPEN=1,1,\\"UDP\\",\\"%s\\",%u,0,0,0", ip, (unsigned)port);
  if (at_cmd(cmd,"OK",3000) != 0) return -1;
  /* 等待 +QIOPEN: 1,0 表示 socket 1 打开成功 */
  if (at_wait("+QIOPEN: 1,0", 10000, NULL, 0) != 0) return -2;
  g_nb.opened = 1;
  return 0;
}

/* ---- 初始化（最小必需流程） ---- */
int NB_Init(const char* apn, const char* ip, uint16_t port){
  if(!apn || !*apn || !ip || !*ip) return -1;

  /* 1) 基础握手 */
  if (at_cmd("AT","OK",1000) != 0){
    (void)at_cmd("AT","OK",1500); // 部分固件第一次慢
  }

  /* 2) 全功能 + 网络附着 */
  (void)at_cmd("AT+CFUN=1","OK",2500);
  (void)at_cmd("AT+CGATT=1","OK",8000);

  /* 3) 设置 PDP（APN） */
  char cmd[96];
  snprintf(cmd,sizeof(cmd),"AT+CGDCONT=1,\\"IP\\",\\"%s\\"", apn);
  if (at_cmd(cmd,"OK",2000) != 0) return -2;

  /* 可选：查询注册与信号，便于调试 */
  (void)at_cmd("AT+CEREG?","OK",1000);
  (void)at_cmd("AT+CSQ","OK",1000);

  /* 4) 打开 UDP */
  if (nb_open_udp(ip, port) != 0) return -3;

  g_nb.inited = 1;
  return 0;
}

/* ---- 发送一行数据到 UDP ---- */
int NB_SendLine(const char* line){
  if(!line || !*line) return -1;
  if(!g_nb.inited || !g_nb.opened) return -2;

  char cmd[48];
  char payload[300];
  size_t L = strlen(line);
  if (L > sizeof(payload)-3) L = sizeof(payload)-3;
  memcpy(payload, line, L);
  payload[L++] = '\\r';
  payload[L++] = '\\n';
  payload[L]   = 0;

  snprintf(cmd,sizeof(cmd),"AT+QISEND=1,%u", (unsigned)L);
  if (at_cmd(cmd,">",2000) != 0) return -3;

  (void)uart_send_bytes((uint8_t*)payload, (uint16_t)L);
  uint8_t ctrlz = 0x1A;
  (void)uart_send_bytes(&ctrlz,1);

  if (at_wait("SEND OK", 5000, NULL, 0) != 0) return -4;
  return 0;
}
