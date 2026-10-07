#ifndef __DBUS_H__
#define __DBUS_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ DR16 / DT7 (DBUS) ============================
   接收机与遥控器对频成功后，每 14ms 通过 DBUS 发一帧 18 字节数据。
   通信参数（见接收机手册）：100000 bps、8 位数据、偶校验、1 位停止位、无流控
   （STM32 端配成 9 位字长 + 偶校验即可）。
   注意：DBUS 的电平和普通 UART 相反，需要取反；C 型开发板的 DBUS 接口已经
   在硬件上做了取反，软件不用再处理。

   18 字节帧结构（没有帧头字节）：
     通道 0~3：各 11 bit，占用前 6 个字节，范围 364~1684，中位 1024；
     S1 / S2 ：各 2 bit，在字节 5 的高 4 位里，1=上、2=下、3=中。
   解码后每个通道都减去中位 1024，得到 -660 ~ +660 的摇杆量。 */

/* 原始通道值范围（未减中位） */
#define DBUS_CH_MIN     364
#define DBUS_CH_MID     1024
#define DBUS_CH_MAX     1684

/* 拨杆位置（DR16 手册规定：1=上, 2=下, 3=中） */
#define DBUS_SW_UP      1u
#define DBUS_SW_DOWN    2u
#define DBUS_SW_MID     3u

/* 超过该时间没有收到新的完整帧，就判定为遥控掉线，执行失控保护 */
#define DBUS_OFFLINE_TIMEOUT_MS   50u

/* 解码后的遥控器数据（通道范围 -660 ~ +660，中位 0） */
typedef struct
{
  int16_t ch0;   /* 右摇杆 左右 */
  int16_t ch1;   /* 右摇杆 上下 */
  int16_t ch2;   /* 左摇杆 左右 */
  int16_t ch3;   /* 左摇杆 上下（本作业用作油门 -> 目标转速） */
  uint8_t s1;    /* 开关 S1：1=上 2=下 3=中 */
  uint8_t s2;    /* 开关 S2：1=上 2=下 3=中 */

  uint32_t frame_cnt;   /* 累计收到的完整帧数（判断是否真的收到过数据） */
  uint32_t last_tick;   /* 最近一帧的解码时刻，单位 HAL_GetTick */
} DBUS_t;

extern DBUS_t g_rc;

void DBUS_Init(void);

/* 下面两个函数由 main.c 里的 HAL 回调统一分发（USART3 = DBUS） */
void DBUS_UartRxCallback(UART_HandleTypeDef *huart);
void DBUS_UartErrorCallback(UART_HandleTypeDef *huart);

uint8_t  DBUS_IsNewData(void);
void     DBUS_ClearNewData(void);
/* 最近 DBUS_OFFLINE_TIMEOUT_MS 内收到过完整帧则返回 1 */
uint8_t  DBUS_IsOnline(void);

/* 调试用：USART3 上一共收到过多少个字节、以及最近 18 个原始字节。
   用途是把"一个字节都没收到"(接线/遥控器问题) 和 "有字节但解不出帧"
   (波特率/校验/取反问题) 区分开。 */
uint32_t DBUS_ByteCount(void);
void     DBUS_GetRawBytes(uint8_t *out, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __DBUS_H__ */
