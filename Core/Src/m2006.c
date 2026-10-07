/**
  ******************************************************************************
  * @file    m2006.c
  * @brief   M2006 P36 + C610 四电机 CAN 驱动（速度环）
  ******************************************************************************
  */

#include "m2006.h"
#include "can.h"

M2006Motor_t g_motor[M2006_MOTOR_NUM];

static uint8_t s_can_ready = 0u;

/* 诊断用：收到的 CAN 帧计数 + 最近 8 个标识符 */
static volatile uint32_t s_rx_frame_cnt = 0u;
static volatile uint16_t s_rx_id_log[8];
static volatile uint8_t  s_rx_id_idx = 0u;

/* ---- 速度环 PID 起始参数（转子 rpm -> 电流指令）-------------------------
   这三个数只是"能转起来"的起点，必须按实车整定：
   先把 ki、kd 置 0，调 kp 到跟手且不过冲；再加一点 ki 消除静差。 */
#define _KP_SPEED        3.0f
#define _KI_SPEED        0.10f
#define _KD_SPEED        0.0f
#define _IOUT_LIMIT      3000.0f   /* 增量式用不到，留作备用 */
#define _SPEED_DEADBAND  10.0f     /* rpm，抑制零漂引起的抖动 */

static void update_angle(M2006Motor_t *m);
static uint8_t m2006_can_setup(void);

void M2006_Init(void)
{
  uint8_t i;

  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    M2006Motor_t *m = &g_motor[i];

    m->id           = (uint8_t)(i + 1u);   /* 电调 ID 1~4 */
    m->online       = 0u;
    m->ecd_raw      = 0u;
    m->last_ecd     = 0u;
    m->total_ecd    = 0;
    m->speed_rpm    = 0;
    m->torque_cur   = 0;
    m->temp         = 0u;
    m->output_deg   = 0.0f;
    m->last_rx_tick = 0u;
    m->target_rpm   = 0.0f;
    m->current_out  = 0;

    PID_Init(&m->speed_pid, _KP_SPEED, _KI_SPEED, _KD_SPEED,
             M2006_CURRENT_LIMIT, _IOUT_LIMIT, _SPEED_DEADBAND, PID_INCREMENTAL);
  }

  /* 拉起 CAN1。总线还没空闲时（电调没上电 / CANH-CANL 接反 / 缺 120 欧
     终端电阻）HAL_CAN_Init 可能失败，这里不锁死，主循环会一直重试。 */
  M2006_CanRetry();

  /* 先发一帧 0 电流，让四个电调进入受控状态 */
  M2006_SendCmd();
}

/* 过滤器结构体必须整体清零：F407 上 CAN2SB 字段也在里面，不清零可能把
   所有过滤器组都分给 CAN2，导致 CAN1 一个报文都收不到。 */
static uint8_t m2006_can_setup(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterActivation     = ENABLE;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh         = 0x0000;
  filter.FilterIdLow          = 0x0000;
  filter.FilterMaskIdHigh     = 0x0000;
  filter.FilterMaskIdLow      = 0x0000;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;   /* 和 RX1 中断对应 */
  filter.FilterBank           = 0u;
  filter.SlaveStartFilterBank = 14u;                /* 0~13 组归 CAN1 */

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) return 0u;
  if (HAL_CAN_Start(&hcan1) != HAL_OK)                return 0u;
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
    return 0u;
  }

  return 1u;
}

uint8_t M2006_IsReady(void)
{
  return s_can_ready;
}

/* 可以反复调用：只在还没起来、或者总线 off 时才重新初始化 */
void M2006_CanRetry(void)
{
  /* 无论 can.c 是怎么生成的，都把自动总线关闭恢复打开 */
  hcan1.Init.AutoBusOff = ENABLE;

  if (s_can_ready != 0u)
  {
    if ((hcan1.Instance->ESR & CAN_ESR_BOFF) == 0u) return;   /* 一切正常 */

    (void)HAL_CAN_Stop(&hcan1);
    s_can_ready = 0u;
  }

  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    return;   /* 仍不可用，等下一次重试 */
  }

  if (m2006_can_setup() == 0u) return;

  if (hcan1.State == HAL_CAN_STATE_LISTENING)
  {
    s_can_ready = 1u;
  }
}

void M2006_SetTargetRpm(uint8_t idx, float rpm)
{
  if (idx >= M2006_MOTOR_NUM) return;

  if (rpm >  M2006_ROTOR_RPM_MAX) rpm =  M2006_ROTOR_RPM_MAX;
  if (rpm < -M2006_ROTOR_RPM_MAX) rpm = -M2006_ROTOR_RPM_MAX;

  g_motor[idx].target_rpm = rpm;
}

void M2006_SetAllTargetRpm(const float rpm[M2006_MOTOR_NUM])
{
  uint8_t i;

  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    M2006_SetTargetRpm(i, rpm[i]);
  }
}

void M2006_StopAll(void)
{
  uint8_t i;

  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    g_motor[i].target_rpm  = 0.0f;
    g_motor[i].current_out = 0;
    PID_Clear(&g_motor[i].speed_pid);
  }
}

void M2006_ControlLoop(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t  i;

  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    M2006Motor_t *m = &g_motor[i];

    /* 反馈超时判定：刚上电 last_rx_tick 还是 0，也按掉线处理 */
    if (m->last_rx_tick == 0u)
    {
      m->online = 0u;
    }
    else
    {
      m->online = ((uint32_t)(now - m->last_rx_tick) <= M2006_FB_TIMEOUT_MS) ? 1u : 0u;
    }

    if (m->online == 0u)
    {
      /* 这个电调没有反馈：电流给 0，并把 PID 内部状态清零，
         免得它一恢复就带着一堆积分冲出去。 */
      PID_Clear(&m->speed_pid);
      m->current_out = 0;
      continue;
    }

    /* 每一拍都必须刷新反馈，否则 PID 会退化成对着 0 算误差 */
    PID_SetFeedback(&m->speed_pid, (float)m->speed_rpm);
    PID_SetTarget(&m->speed_pid, m->target_rpm);
    PID_Calc(&m->speed_pid);

    /* 双保险：PID 内部已按 max_out 限过，这里再夹一次 */
    if (m->speed_pid.output >  M2006_CURRENT_LIMIT) m->speed_pid.output =  M2006_CURRENT_LIMIT;
    if (m->speed_pid.output < -M2006_CURRENT_LIMIT) m->speed_pid.output = -M2006_CURRENT_LIMIT;

    m->current_out = (int16_t)m->speed_pid.output;
  }
}

void M2006_SendCmd(void)
{
  CAN_TxHeaderTypeDef tx = {0};
  uint8_t  data[8];
  uint32_t mailbox = 0;
  uint8_t  i;

  if (s_can_ready == 0u) return;

  tx.StdId = M2006_CAN_TX_ID;
  tx.IDE   = CAN_ID_STD;
  tx.RTR   = CAN_RTR_DATA;
  tx.DLC   = 8;

  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    uint16_t cur = (uint16_t)g_motor[i].current_out;

    data[2u * i]      = (uint8_t)((cur >> 8) & 0xFFu);
    data[2u * i + 1u] = (uint8_t)(cur & 0xFFu);
  }

  (void)HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox);
}

void M2006_OnCanRx(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx;
  uint8_t data[8];
  uint8_t idx;
  M2006Motor_t *m;

  if (hcan->Instance != CAN1) return;

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx, data) != HAL_OK)
  {
    return;
  }

  /* 先把这一帧的标识符记下来（不管 ID 对不对都记），方便查 ID 设错 */
  s_rx_frame_cnt++;
  s_rx_id_log[s_rx_id_idx] = rx.StdId;
  s_rx_id_idx = (uint8_t)((s_rx_id_idx + 1u) % 8u);

  /* 只收 0x201 ~ 0x204（也就是 1~4 号电调） */
  if ((rx.StdId < M2006_CAN_FB_ID_BASE) ||
      (rx.StdId >= (M2006_CAN_FB_ID_BASE + M2006_MOTOR_NUM)))
  {
    return;
  }

  idx = (uint8_t)(rx.StdId - M2006_CAN_FB_ID_BASE);
  m   = &g_motor[idx];

  m->ecd_raw    = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
  m->speed_rpm  =  (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  m->torque_cur =  (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  m->temp       = data[6];
  m->last_rx_tick = HAL_GetTick();

  update_angle(m);
}

/* CAN1 RX1 中断回调（stm32f4xx_it.c 的 CAN1_RX1_IRQHandler -> HAL_CAN_IRQHandler） */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    M2006_OnCanRx(hcan);
  }
}

uint8_t M2006_OnlineCount(void)
{
  uint8_t i;
  uint8_t n = 0u;

  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    if (g_motor[i].online != 0u) n++;
  }

  return n;
}

uint32_t M2006_RxFrameCount(void)
{
  return s_rx_frame_cnt;
}

void M2006_GetRxIdLog(uint16_t *out, uint8_t n)
{
  uint8_t i;
  uint8_t cnt   = (n > 8u) ? 8u : n;
  uint8_t start = (uint8_t)((s_rx_id_idx + 8u - cnt) % 8u);

  for (i = 0u; i < cnt; i++)
  {
    out[i] = s_rx_id_log[(uint8_t)((start + i) % 8u)];
  }
}

/* ---------------------------------------------------------------------------
   回环自测（串口命令 cantest 调用）

   把 CAN1 临时切到 loopback 模式：这个模式下自己发出去的帧会被自己收回来，
   ACK 由控制器内部产生，所以**完全不需要外部接线和电调**。

   用它来分离两类问题：
     通过 -> 位时序、过滤器、中断、帧解析、驱动代码全都是好的，
             那"收不到电调反馈"就一定是总线/电调那侧（没上电、没接、接反）
     不通过 -> 是 MCU 侧的问题，得回来查代码/芯片

   注意：不管成功失败都要把模式还原成 NORMAL，否则车就开不了了。
   --------------------------------------------------------------------------- */
uint8_t M2006_LoopbackSelfTest(void)
{
  CAN_TxHeaderTypeDef tx = {0};
  uint8_t  data[8];
  uint32_t mailbox = 0;
  uint8_t  ok = 0u;
  int16_t  saved_speed = g_motor[0].speed_rpm;
  uint32_t saved_tick  = g_motor[0].last_rx_tick;

  /* ---- 1) 切到回环模式 ---- */
  (void)HAL_CAN_Stop(&hcan1);
  s_can_ready = 0u;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;

  if (HAL_CAN_Init(&hcan1) != HAL_OK) goto m2006_selftest_restore;
  if (m2006_can_setup() == 0u)        goto m2006_selftest_restore;

  /* ---- 2) 伪造一帧 1 号电调的反馈：转速 1234、温度 33 ---- */
  tx.StdId = M2006_CAN_FB_ID_BASE;      /* 0x201 */
  tx.IDE   = CAN_ID_STD;
  tx.RTR   = CAN_RTR_DATA;
  tx.DLC   = 8;

  data[0] = 0x10; data[1] = 0x00;       /* 机械角度 */
  data[2] = 0x04; data[3] = 0xD2;       /* 转子转速 = 1234 */
  data[4] = 0x00; data[5] = 0x64;       /* 转矩电流 = 100 */
  data[6] = 33u;                        /* 温度 = 33 */
  data[7] = 0x00;

  if (HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox) == HAL_OK)
  {
    /* 回环的帧也要经过接收中断才会进到 g_motor[]，等它处理完 */
    HAL_Delay(5);

    if ((g_motor[0].speed_rpm == 1234) && (g_motor[0].temp == 33u))
    {
      ok = 1u;
    }
  }

m2006_selftest_restore:
  /* ---- 3) 无论结果如何都还原成正常模式 ---- */
  (void)HAL_CAN_Stop(&hcan1);
  s_can_ready = 0u;
  hcan1.Init.Mode = CAN_MODE_NORMAL;

  if (HAL_CAN_Init(&hcan1) == HAL_OK)
  {
    if (m2006_can_setup() != 0u)
    {
      if (hcan1.State == HAL_CAN_STATE_LISTENING)
      {
        s_can_ready = 1u;
      }
    }
  }

  /* 把自测灌进去的假数据清掉，别让它冒充真实反馈 */
  g_motor[0].speed_rpm    = saved_speed;
  g_motor[0].last_rx_tick = saved_tick;
  g_motor[0].temp         = 0u;

  return ok;
}

/* 机械角度是 0~8191 的循环值，累加前要先解卷绕，否则过零点会跳变 */
static void update_angle(M2006Motor_t *m)
{
  int16_t delta = (int16_t)(m->ecd_raw - m->last_ecd);

  if (delta >  M2006_ENCODER_HALF) delta = (int16_t)(delta - (int16_t)M2006_ECD_PER_REV);
  if (delta < -M2006_ENCODER_HALF) delta = (int16_t)(delta + (int16_t)M2006_ECD_PER_REV);

  m->total_ecd += delta;
  m->last_ecd   = m->ecd_raw;

  /* 输出轴角度 = 累计计数 / (8192 x 36) x 360 度 */
  m->output_deg = (float)m->total_ecd * 360.0f
                  / ((float)M2006_ECD_PER_REV * M2006_GEAR_RATIO);
}
