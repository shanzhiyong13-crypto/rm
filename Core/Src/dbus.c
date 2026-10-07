#include "dbus.h"
#include "usart.h"
#include <string.h>

/* DR16 每 14ms 发一帧 18 字节，帧内字节连续发送（间隔约 0.11ms），
   帧与帧之间空闲约 12ms。利用"距上一个字节的时间间隔"就能判断帧边界，
   不需要帧头也能对齐；万一丢字节，下一帧会自动重新对齐。 */
#define DBUS_FRAME_LEN     18u
#define DBUS_FRAME_GAP_MS  3u    /* 帧间空闲判据(ms)，帧内字节间隔 <= 1ms */
#define DBUS_RAW_LEN       18u   /* 调试用：保存最近收到的原始字节数 */

static uint8_t  s_rx_byte;                 /* 单字节中断接收缓冲 */
static uint8_t  s_frame[DBUS_FRAME_LEN];
static uint8_t  s_frame_idx;
static uint32_t s_last_byte_tick;
static volatile uint8_t s_new_data;

/* 调试用：不做对齐，原样记录最近收到的字节，方便在串口上打印出来看 */
static uint8_t  s_raw[DBUS_RAW_LEN];
static uint8_t  s_raw_idx;
static uint32_t s_byte_cnt;

DBUS_t g_rc;

static uint8_t DBUS_Decode(const uint8_t *b);

void DBUS_Init(void)
{
  s_frame_idx      = 0u;
  s_last_byte_tick = HAL_GetTick();
  s_new_data       = 0u;
  s_raw_idx        = 0u;
  s_byte_cnt       = 0u;
  memset(&g_rc, 0, sizeof(g_rc));
  memset(s_raw, 0, sizeof(s_raw));

  /* 启动第一个字节的接收（之后每个字节都在中断回调里续上） */
  (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
}

uint8_t DBUS_IsNewData(void)
{
  return s_new_data;
}

void DBUS_ClearNewData(void)
{
  s_new_data = 0u;
}

/* 遥控在线判据：收到过至少一帧，且最近一帧距今不超过 50ms。
   刚上电时 frame_cnt 还是 0，这里会返回 0，主循环按"掉线"处理，是安全的。 */
uint8_t DBUS_IsOnline(void)
{
  if (g_rc.frame_cnt == 0u)
  {
    return 0u;
  }

  return ((uint32_t)(HAL_GetTick() - g_rc.last_tick) <= DBUS_OFFLINE_TIMEOUT_MS) ? 1u : 0u;
}

uint32_t DBUS_ByteCount(void)
{
  return s_byte_cnt;
}

/* 把最近 len(<=18) 个收到的字节按时间顺序拷到 out */
void DBUS_GetRawBytes(uint8_t *out, uint8_t len)
{
  uint8_t i;
  uint8_t n = (len > DBUS_RAW_LEN) ? DBUS_RAW_LEN : len;
  uint8_t start = (uint8_t)((s_raw_idx + DBUS_RAW_LEN - n) % DBUS_RAW_LEN);

  for (i = 0u; i < n; i++)
  {
    out[i] = s_raw[(uint8_t)((start + i) % DBUS_RAW_LEN)];
  }
}

/* 由 main.c 的 HAL_UART_RxCpltCallback 调用，每收到一个字节进入一次。 */
void DBUS_UartRxCallback(UART_HandleTypeDef *huart)
{
  uint32_t now;

  if (huart->Instance != USART3)
  {
    return;
  }

  /* 调试统计：先原样记下来，再去做帧对齐 */
  s_raw[s_raw_idx] = s_rx_byte;
  s_raw_idx = (uint8_t)((s_raw_idx + 1u) % DBUS_RAW_LEN);
  s_byte_cnt++;

  now = HAL_GetTick();

  /* 距上一字节超过 3ms => 新的一帧开始，重新对齐 */
  if ((uint32_t)(now - s_last_byte_tick) > DBUS_FRAME_GAP_MS)
  {
    s_frame_idx = 0u;
  }
  s_last_byte_tick = now;

  if (s_frame_idx < DBUS_FRAME_LEN)
  {
    s_frame[s_frame_idx++] = s_rx_byte;
  }

  /* 凑满 18 字节就解码，并通知主循环有新数据 */
  if (s_frame_idx >= DBUS_FRAME_LEN)
  {
    /* 只有通道值都在合法范围内的帧才算有效：万一对齐错了或者线上有噪声，
       解出来的通道值会越界，这时直接丢帧，绝不让电机跟着乱码转。 */
    if (DBUS_Decode(s_frame) != 0u)
    {
      g_rc.last_tick = now;
      g_rc.frame_cnt++;
      s_new_data = 1u;
    }
    s_frame_idx = 0u;
  }

  /* 续上下一个字节的接收 */
  (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
}

/* 奇偶校验错 / 帧错 / 噪声 / 溢出等异常：HAL 会中止中断接收，
   必须在这里重新启动，否则一次干扰之后 DBUS 就永远收不到数据了。 */
void DBUS_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3)
  {
    return;
  }

  __HAL_UART_CLEAR_PEFLAG(huart);
  huart->ErrorCode = HAL_UART_ERROR_NONE;

  if (huart->RxState == HAL_UART_STATE_READY)
  {
    (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
  }
}

/* ---- 18 字节 DBUS 帧解码（与接收机手册的位表一一对应）----
   b[0..5] 依次打包：ch0(11) ch1(11) ch2(11) ch3(11) S1(2) S2(2)，
   后 12 个字节是键鼠信息，本作业用不到。 */
static uint8_t DBUS_Decode(const uint8_t *b)
{
  uint16_t c0, c1, c2, c3;

  c0 = (uint16_t)((b[0] | (b[1] << 8)) & 0x07FF);
  c1 = (uint16_t)(((b[1] >> 3) | (b[2] << 5)) & 0x07FF);
  c2 = (uint16_t)(((b[2] >> 6) | (b[3] << 2) | (b[4] << 10)) & 0x07FF);
  c3 = (uint16_t)(((b[4] >> 1) | (b[5] << 7)) & 0x07FF);

  /* 合法通道值范围是 364 ~ 1684，越界说明这一帧不可信 */
  if ((c0 < DBUS_CH_MIN) || (c0 > DBUS_CH_MAX) ||
      (c1 < DBUS_CH_MIN) || (c1 > DBUS_CH_MAX) ||
      (c2 < DBUS_CH_MIN) || (c2 > DBUS_CH_MAX) ||
      (c3 < DBUS_CH_MIN) || (c3 > DBUS_CH_MAX))
  {
    return 0u;
  }

  g_rc.ch0 = (int16_t)((int16_t)c0 - DBUS_CH_MID);
  g_rc.ch1 = (int16_t)((int16_t)c1 - DBUS_CH_MID);
  g_rc.ch2 = (int16_t)((int16_t)c2 - DBUS_CH_MID);
  g_rc.ch3 = (int16_t)((int16_t)c3 - DBUS_CH_MID);

  /* S1 = b[5] 的 bit4~5，S2 = b[5] 的 bit6~7（手册位表：ch3 占 bit33~43，
     所以紧跟其后的 S1 落在低半字节） */
  g_rc.s1 = (uint8_t)((b[5] >> 4) & 0x03u);
  g_rc.s2 = (uint8_t)((b[5] >> 6) & 0x03u);

  return 1u;
}
