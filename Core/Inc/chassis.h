/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   麦轮底盘逆运动学：底盘速度 -> 四轮转速
  ******************************************************************************
  * 坐标系（俯视）：x 轴向前，y 轴向左，z 轴竖直向上；
  * 绕 z 轴逆时针旋转为 wz 正方向。
  *
  * 轮子编号与 C610 电调 ID 一一对应（也等于 g_motor[] 的下标）：
  *   index 0 = ID1 = 左前(LF)      index 1 = ID2 = 右前(RF)
  *   index 2 = ID3 = 左后(LB)      index 3 = ID4 = 右后(RB)
  *
  * O 型安装（1/3 左旋轮、2/4 右旋轮）的逆解算：
  *   v_LF = vx + vy + L*wz
  *   v_RF = vx - vy - L*wz
  *   v_LB = vx + vy - L*wz
  *   v_RB = vx - vy + L*wz
  * 其中 L 是"轮心到车体中心"的等效距离，先用 (半长 + 半宽)/2 近似，
  * 跑起来后再用原地旋转标定。
  ******************************************************************************
  */

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 机械参数：上车前按实际测量值改这几个数 ----
   RM 麦轮直径 152.5mm；HALF_LENGTH / HALF_WIDTH 是四条轮心构成矩形的
   半长 b 和半宽 a（前后轮心距除以 2、左右轮心距除以 2）。 */
#define CHASSIS_WHEEL_DIAMETER_M   0.1525f
#define CHASSIS_HALF_LENGTH_M      0.110f
#define CHASSIS_HALF_WIDTH_M       0.100f

/* ---- 速度上限：先给保守值，跑顺了再往上放 ---- */
#define CHASSIS_MAX_VX_MPS         1.2f    /* 前后 m/s */
#define CHASSIS_MAX_VY_MPS         1.0f    /* 左右平移 m/s */
#define CHASSIS_MAX_WZ_RPS         3.0f    /* 自转 rad/s */
#define CHASSIS_MAX_WHEEL_MPS      1.5f    /* 单轮线速度上限，超了就四轮一起等比缩小 */

void  Chassis_Init(void);

/* 设定底盘速度：vx/vy 单位 m/s，wz 单位 rad/s（正方向见文件头） */
void  Chassis_SetVelocity(float vx, float vy, float wz);

/* 解算四个轮子 -> 换算成转子 rpm -> 写入电机目标值（1kHz 调用） */
void  Chassis_Update(void);

void  Chassis_Stop(void);

/* 调试用：最近一次解算出的单轮线速度(m/s) 和方向修正系数 */
float Chassis_GetWheelMps(uint8_t idx);
float Chassis_GetWheelSign(uint8_t idx);

/* 逻辑轮位 -> 电调索引（实车标定出来的映射表） */
uint8_t Chassis_WheelToMotor(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H__ */
