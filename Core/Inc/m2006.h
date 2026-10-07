/**
  ******************************************************************************
  * @file    m2006.h
  * @brief   M2006 P36 减速电机 + C610 电调，四电机 CAN 驱动
  ******************************************************************************
  * 底盘四个轮子各用一台 M2006 P36(减速比 36:1) + C610 电调，四个电调挂在
  * 同一条 CAN1 上（1Mbps）。
  *
  * 下发（板 -> 电调）：标准帧 ID = 0x200，DLC = 8，
  *   DATA[0..1] 1 号电调电流指令、DATA[2..3] 2 号、DATA[4..5] 3 号、
  *   DATA[6..7] 4 号，每个都是 int16、高字节在前。
  *   （ID 5~8 才用 0x1FF 那一帧，本车只有 4 个电机，只发 0x200。）
  *
  * 反馈（电调 -> 板）：标准帧 ID = 0x201 ~ 0x204，DLC = 8，
  *   DATA[0..1] 转子机械角度(0~8191)、DATA[2..3] 转子转速(转子 rpm)、
  *   DATA[4..5] 实际转矩电流、DATA[6] 温度、DATA[7] 保留。
  *   C610 默认 1kHz 回传。
  *
  * 注意：C610 的电流指令量程是 ±10000，C620 才是 ±16384，两者不要混用。
  ******************************************************************************
  */

#ifndef __M2006_H__
#define __M2006_H__

#include "main.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- CAN 报文 ---- */
#define M2006_MOTOR_NUM        4u
#define M2006_CAN_TX_ID        0x200u
#define M2006_CAN_FB_ID_BASE   0x201u

/* ---- 机械/电气参数 ---- */
#define M2006_GEAR_RATIO       36.0f      /* M2006 P36 减速比 36:1 */
#define M2006_ECD_PER_REV      8192u      /* 机械角度一圈 8192 个计数 */
#define M2006_ENCODER_HALF     4096       /* 解卷绕用（半圈） */

#define M2006_CURRENT_MAX      10000.0f   /* C610 电流指令量程 ±10000 */
#define M2006_CURRENT_LIMIT    7000.0f    /* 自己再加一道限幅，防误动作 */

/* 目标/反馈转速上限（转子 rpm）。M2006 输出轴空载 500rpm，折算到转子约
   18000rpm；这里先按 6000 保守限速，跑顺了再往上放。 */
#define M2006_ROTOR_RPM_MAX    6000.0f

/* 反馈超时：超过这个时间没收到该电调的帧，就认为它掉线 */
#define M2006_FB_TIMEOUT_MS    50u

typedef struct
{
  uint8_t  id;             /* 电调 ID：1~4 */
  uint8_t  online;         /* 反馈是否在超时窗口内 */

  uint16_t ecd_raw;        /* 最近一次机械角度原始值 */
  uint16_t last_ecd;       /* 上一次机械角度（解卷绕用） */
  int32_t  total_ecd;      /* 累计角度计数 */
  int16_t  speed_rpm;      /* 转子转速 rpm（电调反馈） */
  int16_t  torque_cur;     /* 实际转矩电流（电调反馈） */
  uint8_t  temp;           /* 温度 */
  float    output_deg;     /* 输出轴累计角度（度） */

  uint32_t last_rx_tick;   /* 最近一次收到反馈的时刻 */

  float    target_rpm;     /* 目标转子转速 rpm */
  int16_t  current_out;    /* 本次下发的电流指令 */

  Pid_t    speed_pid;      /* 速度环：转子 rpm -> 电流指令 */
} M2006Motor_t;

extern M2006Motor_t g_motor[M2006_MOTOR_NUM];

/* 初始化（含 CAN 过滤器/启动/中断），电调还没上电也不会卡住 */
void    M2006_Init(void);

/* 设置目标转速（转子 rpm，自动限幅到 ±M2006_ROTOR_RPM_MAX） */
void    M2006_SetTargetRpm(uint8_t idx, float rpm);
void    M2006_SetAllTargetRpm(const float rpm[M2006_MOTOR_NUM]);
void    M2006_StopAll(void);

/* 1kHz 调用：算速度环，得到四路电流指令 */
void    M2006_ControlLoop(void);

/* 1kHz 调用：把四路电流指令打包成 0x200 一帧发出去 */
void    M2006_SendCmd(void);

/* CAN 链路维护：掉线/总线 off 时重新拉起，正常时直接返回 */
void    M2006_CanRetry(void);
uint8_t M2006_IsReady(void);

/* 反馈正常的电机个数（0~4），用于指示灯/状态显示 */
uint8_t M2006_OnlineCount(void);

/* CAN 接收中断里调用 */
void    M2006_OnCanRx(CAN_HandleTypeDef *hcan);

/* 回环自测：把 CAN1 切到 loopback，自己发一帧 0x201（模拟电调反馈），
   检查能不能经过 过滤器 -> 中断 -> 解析 全链路回到 g_motor[0]。
   整个过程不依赖任何外部接线，用来判断问题在 MCU 侧还是在总线/电调侧。
   返回 1 = 通过。测试结束后会自动恢复成正常模式。 */
uint8_t M2006_LoopbackSelfTest(void);

/* 诊断：收到的 CAN 帧总数，以及最近 8 个帧的标识符。
   用来查"电调 ID 设错"这类问题——如果收到的是 0x205，就说明 1 号电调
   其实被设成了 ID 5。 */
uint32_t M2006_RxFrameCount(void);
void     M2006_GetRxIdLog(uint16_t *out, uint8_t n);

#ifdef __cplusplus
}
#endif

#endif /* __M2006_H__ */
