/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   麦轮底盘逆运动学实现
  ******************************************************************************
  */

#include "chassis.h"
#include "m2006.h"

#ifndef M_PI
#define M_PI   3.14159265358979323846f
#endif

/* ---------------------------------------------------------------------------
   实车标定结果（用串口 wheel 命令逐个试出来的）

   逻辑轮位（本文件的编号）   实际车轮   电调 ID   给正转速时的表现
        0  左前 LF            左前         3       把车往前推  -> 方向正确
        1  右前 RF            右前         4       把车往后推  -> 要取反
        2  左后 LB            左后         1       把车往前推  -> 方向正确
        3  右后 RB            右后         2       把车往后推  -> 要取反

   所以这张表要做两件事：
     1) 逻辑轮位 -> 电调索引的映射：实车的电调顺序是 左后/右后/左前/右前，
        也就是比逻辑顺序"前后对调"，即 (i + 2) % 4。
     2) 方向修正：右侧两个轮子取 -1。

   换车/换电机后要重新标定这两张表。
   --------------------------------------------------------------------------- */
static const uint8_t s_wheel_to_motor[M2006_MOTOR_NUM] = { 2u, 3u, 0u, 1u };
static const float   s_wheel_sign[M2006_MOTOR_NUM] = { +1.0f, -1.0f, +1.0f, -1.0f };

/* 平移 / 自转方向的修正开关。
   四个轮子的"前进方向"已经标定好了，但平移和自转的方向取决于麦轮滚子的
   手性（左旋轮装在左边还是右边），这个靠公式推不出来，要实测。
   用 move 命令测：如果"左右平移"方向反了就把下面第一个改成 -1.0f，
   如果"自转"方向反了就把第二个改成 -1.0f。 */
#define CHASSIS_VY_SIGN   (+1.0f)
#define CHASSIS_WZ_SIGN   (+1.0f)

/* 逆解算（RM 麦轮标准 O 型安装）：
     左前 LF: vx - vy - L*wz
     右前 RF: vx + vy + L*wz
     左后 LB: vx + vy - L*wz
     右后 RB: vx - vy + L*wz
   关键点：vy 和 wz 两项在"左前/右前"上的符号是相反的。之前写成两边同号，
   结果平移指令退化成"单侧驱动"，车会原地打转——就是这么个错。 */

/* 当前底盘速度指令 */
static float s_vx = 0.0f;
static float s_vy = 0.0f;
static float s_wz = 0.0f;

/* 最近一次解算出的四轮线速度（m/s），调试用 */
static float s_wheel_mps[M2006_MOTOR_NUM];

static float clampf(float v, float lo, float hi)
{
  if (v > hi) return hi;
  if (v < lo) return lo;
  return v;
}

void Chassis_Init(void)
{
  uint8_t i;

  s_vx = 0.0f;
  s_vy = 0.0f;
  s_wz = 0.0f;

  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    s_wheel_mps[i] = 0.0f;
  }
}

void Chassis_SetVelocity(float vx, float vy, float wz)
{
  s_vx = clampf(vx, -CHASSIS_MAX_VX_MPS, CHASSIS_MAX_VX_MPS);
  s_vy = clampf(vy, -CHASSIS_MAX_VY_MPS, CHASSIS_MAX_VY_MPS);
  s_wz = clampf(wz, -CHASSIS_MAX_WZ_RPS, CHASSIS_MAX_WZ_RPS);
}

void Chassis_Stop(void)
{
  s_vx = 0.0f;
  s_vy = 0.0f;
  s_wz = 0.0f;
  M2006_StopAll();
}

/* 轮子线速度(m/s) -> 电机转子转速(rpm)
   线速度 / 轮周长 = 输出轴 转/秒 -> x60 = 输出轴 rpm -> x36 = 转子 rpm */
static float wheel_mps_to_rotor_rpm(float v_mps)
{
  float wheel_rps = v_mps / (M_PI * CHASSIS_WHEEL_DIAMETER_M);

  return wheel_rps * 60.0f * M2006_GEAR_RATIO;
}

void Chassis_Update(void)
{
  /* 自转项的力臂：轮心在 (±b, ±a) 时，逆解算里对应的系数正好是 (a+b)。
     （之前写成 (a+b)/2 偏小一半，自转的力度会不够，顺便修正。） */
  float L = CHASSIS_HALF_LENGTH_M + CHASSIS_HALF_WIDTH_M;
  float v[M2006_MOTOR_NUM];
  float motor_rpm[M2006_MOTOR_NUM] = { 0.0f, 0.0f, 0.0f, 0.0f };
  float max_abs = 0.0f;
  float scale   = 1.0f;
  uint8_t i;

  /* 1) 逆运动学：底盘三自由度 -> 四个轮子的线速度 */
  v[0] = s_vx - CHASSIS_VY_SIGN * s_vy - CHASSIS_WZ_SIGN * L * s_wz;   /* 左前 */
  v[1] = s_vx + CHASSIS_VY_SIGN * s_vy + CHASSIS_WZ_SIGN * L * s_wz;   /* 右前 */
  v[2] = s_vx + CHASSIS_VY_SIGN * s_vy - CHASSIS_WZ_SIGN * L * s_wz;   /* 左后 */
  v[3] = s_vx - CHASSIS_VY_SIGN * s_vy + CHASSIS_WZ_SIGN * L * s_wz;   /* 右后 */

  /* 2) 归一化：任一轮子超速就四个轮子一起等比缩小。这一步不能省——
        只对单个轮子限幅会改变运动方向，底盘就不走直线了。 */
  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    float a = (v[i] >= 0.0f) ? v[i] : -v[i];

    if (a > max_abs) max_abs = a;
  }
  if (max_abs > CHASSIS_MAX_WHEEL_MPS)
  {
    scale = CHASSIS_MAX_WHEEL_MPS / max_abs;
  }

  /* 3) 换算成转子 rpm，按标定表送到对应的电调，并加方向修正 */
  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    s_wheel_mps[i] = v[i] * scale;
    motor_rpm[s_wheel_to_motor[i]] =
        s_wheel_sign[i] * wheel_mps_to_rotor_rpm(s_wheel_mps[i]);
  }

  /* 4) 写给四个电机（内部还会再按 M2006_ROTOR_RPM_MAX 夹一次） */
  M2006_SetAllTargetRpm(motor_rpm);
}

float Chassis_GetWheelMps(uint8_t idx)
{
  if (idx >= M2006_MOTOR_NUM) return 0.0f;
  return s_wheel_mps[idx];
}

float Chassis_GetWheelSign(uint8_t idx)
{
  if (idx >= M2006_MOTOR_NUM) return 0.0f;
  return s_wheel_sign[idx];
}

/* 逻辑轮位对应的电调索引（0~3，加 1 就是电调 ID），串口显示用 */
uint8_t Chassis_WheelToMotor(uint8_t idx)
{
  if (idx >= M2006_MOTOR_NUM) return 0u;
  return s_wheel_to_motor[idx];
}
