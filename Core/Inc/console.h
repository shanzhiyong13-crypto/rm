#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONSOLE_BUF_LEN   64u
#define CONSOLE_RB_SIZE   256u

void Console_Init(void);
void Console_Task(void);
uint8_t Console_RxActivity(void);

/* 累计收到过多少字节（USART1 / USART6）。调"电脑->板子"方向时看这两个数：
   敲一串字符时数字跟着涨，说明接收方向是通的。 */
uint32_t Console_RxCount1(void);
uint32_t Console_RxCount6(void);

void Console_Send(const char *s);
void Console_SendLn(const char *s);

/* 由 main.c 里的 HAL 回调统一分发（USART1 / USART6） */
void Console_UartRxCallback(UART_HandleTypeDef *huart);
void Console_UartErrorCallback(UART_HandleTypeDef *huart);

/* 台架手动调速：
     bench <rpm>        -> 四轮同速，manual_idx = -1
     wheel <i> <rpm>    -> 只转第 i 个轮子（i = 0..3），manual_idx = i
     remote             -> 退回遥控
   进入台架模式后不受遥控影响（遥控掉线也照转），退出用 remote。 */
uint8_t Console_ManualActive(void);
int8_t  Console_ManualWheel(void);   /* -1 = 四轮同速；0..3 = 单轮 */
float   Console_ManualRpm(void);

/* 台架：直接用底盘速度指令验证麦轮解算（move 命令），同样不受遥控影响 */
uint8_t Console_MoveActive(void);
void    Console_MoveGet(float *vx, float *vy, float *wz);

#ifdef __cplusplus
}
#endif

#endif /* __CONSOLE_H__ */
