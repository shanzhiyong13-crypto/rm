/**
  ******************************************************************************
  * @file    console.c
  * @brief   调试串口命令（USART1 / USART6，115200 8-N-1）
  ******************************************************************************
  * C 型开发板的 4-pin 串口 = USART1(PA9 发 / PB7 收)，3-pin 串口 =
  * USART6(PG14 发 / PG9 收)。两个口都能收命令，状态同时发往两个口，
  * 插哪个都能看。注意外壳丝印和实际串口是对调的。
  ******************************************************************************
  */

#include "console.h"
#include "usart.h"
#include "can.h"
#include "m2006.h"
#include "chassis.h"
#include "dbus.h"
#include <string.h>
#include <stdlib.h>

static volatile uint8_t  rx_buf[CONSOLE_RB_SIZE];
static volatile uint16_t rx_head = 0u, rx_tail = 0u;
static uint8_t rx_byte1 = 0u;   /* USART1 单字节缓冲 */
static uint8_t rx_byte6 = 0u;   /* USART6 单字节缓冲 */

static uint8_t  line_buf[CONSOLE_BUF_LEN];
static uint8_t  line_pos = 0u;
static uint32_t line_last = 0u;

static uint8_t  mon_on   = 1u;   /* 每秒打印一行状态 */
static uint32_t mon_last = 0u;

static uint8_t  echo_on = 1u;
static volatile uint8_t rx_activity = 0u;

/* 累计接收字节数：区分"一个字节都没收到"和"收到了但解不出命令" */
static volatile uint32_t rx_cnt1 = 0u;
static volatile uint32_t rx_cnt6 = 0u;

/* 台架手动模式：bench / wheel 进入，remote 退出 */
static uint8_t manual_on  = 0u;
static int8_t  manual_idx = -1;    /* -1 = 四轮同速；0..3 = 单轮 */
static float   manual_rpm = 0.0f;

/* 台架运动学模式：move <vx> <vy> <wz> 进入，remote 退出 */
static uint8_t move_on = 0u;
static float   mv_vx = 0.0f;
static float   mv_vy = 0.0f;
static float   mv_wz = 0.0f;

/* 逻辑轮位名字，串口显示映射表用 */
static const char *const s_wheel_name[M2006_MOTOR_NUM] = { "LF", "RF", "LB", "RB" };

/* ------------------------- 整数/定点打印辅助 ------------------------- */
/* 直接用 Keil 的 %f 打印浮点很容易出问题，这里自己转成字符串 */
static void itoa_(int32_t val, char *out)
{
  char tmp[16];
  int i = 0;
  uint32_t v;

  if (val < 0) { *out++ = '-'; v = (uint32_t)(-val); }
  else         { v = (uint32_t)val; }

  if (v == 0) { tmp[i++] = '0'; }
  while (v) { tmp[i++] = (char)('0' + (v % 10)); v /= 10; }
  while (i) *out++ = tmp[--i];
  *out = 0;
}

static void f2s(char *buf, float val, int dec)
{
  uint32_t scale = (dec == 1) ? 10u : (dec == 2 ? 100u : 1000u);
  int neg = (val < 0.0f);
  float v = neg ? -val : val;
  uint32_t iv = (uint32_t)(v * (float)scale + 0.5f);
  uint32_t ip = iv / scale;
  uint32_t fp = iv % scale;
  char tmp[24];
  int idx = 0;
  char digs[16];
  int di = 0;

  if (neg) tmp[idx++] = '-';
  if (ip == 0) tmp[idx++] = '0';
  while (ip) { digs[di++] = (char)('0' + (ip % 10)); ip /= 10; }
  while (di) tmp[idx++] = digs[--di];

  tmp[idx++] = '.';
  if (dec == 1) tmp[idx++] = (char)('0' + (fp % 10));
  else if (dec == 2) { tmp[idx++] = (char)('0' + (fp / 10) % 10); tmp[idx++] = (char)('0' + (fp % 10)); }
  else { tmp[idx++] = (char)('0' + (fp / 100) % 10); tmp[idx++] = (char)('0' + (fp / 10) % 10); tmp[idx++] = (char)('0' + (fp % 10)); }

  tmp[idx] = 0;
  strcpy(buf, tmp);
}

/* 一个字节转成两个十六进制字符 */
static void hex8(uint8_t v, char *out)
{
  const char *d = "0123456789ABCDEF";

  out[0] = d[(v >> 4) & 0x0Fu];
  out[1] = d[v & 0x0Fu];
  out[2] = 0;
}

/* 32 位十六进制（打印 CAN 寄存器用） */
static void hex32(uint32_t v, char *out)
{
  const char *d = "0123456789ABCDEF";
  int i;

  for (i = 0; i < 8; i++)
  {
    out[i] = d[(v >> (28 - (4 * i))) & 0x0Fu];
  }
  out[8] = 0;
}

/* 16 位十六进制（打印 CAN 标识符用） */
static void hex16(uint16_t v, char *out)
{
  const char *d = "0123456789ABCDEF";

  out[0] = d[(v >> 12) & 0x0Fu];
  out[1] = d[(v >>  8) & 0x0Fu];
  out[2] = d[(v >>  4) & 0x0Fu];
  out[3] = d[v & 0x0Fu];
  out[4] = 0;
}

void Console_Send(const char *s)
{
  uint16_t len = (uint16_t)strlen(s);

  if (len == 0u) return;

  HAL_UART_Transmit(&huart1, (uint8_t *)s, len, 100);
  HAL_UART_Transmit(&huart6, (uint8_t *)s, len, 100);
}

void Console_SendLn(const char *s)
{
  Console_Send(s);
  Console_Send("\r\n");
}

/* ------------------------------ 命令实现 ------------------------------ */
static void cmd_help(void)
{
  /* 发送出去的文本一律用 ASCII，避免终端编码不一致时乱码 */
  Console_SendLn("=== mecanum chassis / M2006 x4 / DR16 ===");
  Console_SendLn("help            : show this help");
  Console_SendLn("rc              : print remote channels");
  Console_SendLn("state           : print all four motors");
  Console_SendLn("mon <0|1>       : 1 = one status line per second (default on)");
  Console_SendLn("echo <0|1>      : 1 = echo typed chars (link test)");
  Console_SendLn("bench <rpm>     : all four wheels at <rpm> (rotor rpm)");
  Console_SendLn("wheel <i> <rpm> : only wheel i at <rpm>, others 0");
  Console_SendLn("move <vx> <vy> <wz> : chassis velocity in m/s, m/s, rad/s");
  Console_SendLn("cantest         : CAN loopback self test (no wiring needed)");
  Console_SendLn("canid           : CAN frames received + their identifiers");
  Console_SendLn("remote          : back to stick control");
  Console_SendLn("");
  Console_SendLn("wheel <i> uses the ESC number (i+1 = esc id).");
  Console_SendLn("'state' prints which physical wheel each esc drives.");
  Console_SendLn("move example:  'move 0.5 0 0' = forward, 'move 0 0.5 0' = strafe left,");
  Console_SendLn("               'move 0 0 2' = spin counter-clockwise");
  Console_SendLn("NOTE: bench/wheel/move ignore the remote. Type 'remote' to exit.");
}

static void print_motor_row(uint8_t i)
{
  char b[24];

  Console_Send("  id"); itoa_((int32_t)g_motor[i].id, b);        Console_Send(b);
  Console_Send(" on");  itoa_((int32_t)g_motor[i].online, b);    Console_Send(b);
  Console_Send("  rpm "); itoa_((int32_t)g_motor[i].speed_rpm, b); Console_Send(b);
  Console_Send("  tgt "); itoa_((int32_t)g_motor[i].target_rpm, b); Console_Send(b);
  Console_Send("  cur "); itoa_((int32_t)g_motor[i].current_out, b); Console_Send(b);
  Console_Send("  T");   itoa_((int32_t)g_motor[i].temp, b);      Console_Send(b);
  Console_Send("  deg "); f2s(b, g_motor[i].output_deg, 1);       Console_SendLn(b);
}

static void cmd_state(void)
{
  char b[24];
  uint8_t i;

  Console_SendLn("--- motors ---");
  Console_Send("can ready : "); itoa_((int32_t)M2006_IsReady(), b); Console_SendLn(b);
  Console_Send("online    : "); itoa_((int32_t)M2006_OnlineCount(), b); Console_Send(b);
  Console_Send("/"); itoa_((int32_t)M2006_MOTOR_NUM, b); Console_SendLn(b);

  /* CAN 控制器自己的错误计数：用来区分"总线没接好"和"电调不理我" */
  {
    uint32_t esr = hcan1.Instance->ESR;

    Console_Send("CAN ESR   : 0x"); hex32(esr, b); Console_SendLn(b);
    Console_Send("TEC=");  itoa_((int32_t)((esr >> 16) & 0xFFu), b); Console_Send(b);
    Console_Send(" REC="); itoa_((int32_t)((esr >> 24) & 0xFFu), b); Console_Send(b);
    Console_Send(" LEC="); itoa_((int32_t)(esr & 0x07u), b); Console_Send(b);
    Console_Send(" BOFF="); itoa_((int32_t)((esr >> 2) & 0x01u), b); Console_Send(b);
    Console_SendLn("");
    Console_SendLn("LEC: 0=ok 3=ACK err(no answer) 6=CRC; TEC rising = frames not acked");
  }

  Console_Send("wheel m/s : ");
  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    f2s(b, Chassis_GetWheelMps(i), 2);
    Console_Send(b);
    Console_Send(" ");
  }
  Console_SendLn("");
  Console_Send("sign      : ");
  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    f2s(b, Chassis_GetWheelSign(i), 0);
    Console_Send(b);
    Console_Send(" ");
  }
  Console_SendLn("");

  /* 实车标定出来的轮位映射：逻辑轮位 -> 它实际接在哪个电调上 */
  Console_Send("map       : ");
  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    Console_Send(s_wheel_name[i]);
    Console_Send("->id");
    itoa_((int32_t)Chassis_WheelToMotor(i) + 1, b);
    Console_Send(b);
    Console_Send(" ");
  }
  Console_SendLn("");

  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    print_motor_row(i);
  }
}

static void cmd_rc(void)
{
  char b[32];
  char h[4];
  uint8_t raw[18];
  uint8_t i;

  Console_SendLn("--- remote ---");
  Console_Send("online      : "); itoa_((int32_t)DBUS_IsOnline(), b); Console_SendLn(b);
  Console_Send("frames      : "); itoa_((int32_t)g_rc.frame_cnt, b); Console_SendLn(b);
  /* bytes = USART3 一共收到过多少字节，排查的第一站 */
  Console_Send("bytes       : "); itoa_((int32_t)DBUS_ByteCount(), b); Console_SendLn(b);
  Console_Send("ch0 R-X     : "); itoa_((int32_t)g_rc.ch0, b); Console_SendLn(b);
  Console_Send("ch1 R-Y     : "); itoa_((int32_t)g_rc.ch1, b); Console_SendLn(b);
  Console_Send("ch2 L-X     : "); itoa_((int32_t)g_rc.ch2, b); Console_SendLn(b);
  Console_Send("ch3 L-Y     : "); itoa_((int32_t)g_rc.ch3, b); Console_SendLn(b);
  Console_Send("s1 s2       : "); itoa_((int32_t)g_rc.s1, b); Console_Send(b);
  Console_Send(" ");                 itoa_((int32_t)g_rc.s2, b); Console_SendLn(b);

  /* 最近 18 个原始字节（未做帧对齐），用来判断线上到底是什么 */
  DBUS_GetRawBytes(raw, 18u);
  Console_Send("last 18 hex : ");
  for (i = 0u; i < 18u; i++)
  {
    hex8(raw[i], h);
    Console_Send(h);
    Console_Send(" ");
  }
  Console_SendLn("");
  Console_SendLn("(FF..FF = line idle/no data, 00..00 = line stuck low)");
}

static void cmd_mon(const char **args, int argc)
{
  int m;

  if (argc < 2) { Console_SendLn("usage: mon <0|1>"); return; }
  m = atoi(args[1]);
  mon_on = (m != 0) ? 1u : 0u;
  Console_SendLn(mon_on ? "monitor on." : "monitor off.");
}

static void cmd_echo(const char **args, int argc)
{
  int m;

  if (argc < 2) { Console_SendLn("usage: echo <0|1>"); return; }
  m = atoi(args[1]);
  echo_on = (m != 0) ? 1u : 0u;
  Console_SendLn(echo_on ? "echo on." : "echo off.");
}

/* bench：四轮同速，用来快速确认四个电调都在响应 */
static void cmd_bench(const char **args, int argc)
{
  float v;
  char b[24];

  if (argc < 2) { Console_SendLn("usage: bench <rpm>"); return; }
  v = (float)atof(args[1]);
  manual_rpm = v;
  manual_idx = -1;
  manual_on  = 1u;
  move_on    = 0u;
  f2s(b, v, 1);
  Console_Send("bench all wheels, rpm = "); Console_SendLn(b);
  Console_SendLn("(remote ignored now; type 'remote' to go back)");
}

/* wheel：只转一个轮子，用来标定轮序/转向符号 */
static void cmd_wheel(const char **args, int argc)
{
  int idx;
  float v;
  char b[24];

  if (argc < 3) { Console_SendLn("usage: wheel <0..3> <rpm>"); return; }

  idx = atoi(args[1]);
  if ((idx < 0) || (idx >= (int)M2006_MOTOR_NUM))
  {
    Console_SendLn("wheel index must be 0..3 (0=LF 1=RF 2=LB 3=RB)");
    return;
  }

  v = (float)atof(args[2]);
  manual_idx = (int8_t)idx;
  manual_rpm = v;
  manual_on  = 1u;
  move_on    = 0u;

  Console_Send("wheel "); itoa_((int32_t)idx, b); Console_Send(b);
  Console_Send(" rpm = "); f2s(b, v, 1); Console_SendLn(b);
  Console_SendLn("(remote ignored now; type 'remote' to go back)");
}

/* CAN 回环自测：把 CAN1 切到 loopback 自己发自己收，
   不依赖电调、不依赖接线，用来判断问题在 MCU 侧还是总线侧 */
static void cmd_cantest(void)
{
  Console_SendLn("CAN loopback self test (no ESC, no wiring needed)...");

  if (M2006_LoopbackSelfTest() != 0u)
  {
    Console_SendLn("PASS: tx + filter + irq + decode all work.");
    Console_SendLn("  -> MCU side is OK, so the fault is on the bus:");
    Console_SendLn("     ESC power / CAN1 port / CANH-CANL / ESC id / 120ohm");
  }
  else
  {
    Console_SendLn("FAIL: the loopback frame never came back.");
    Console_SendLn("  -> problem is on the MCU side; tell me and I will dig in.");
  }
}

/* 看总线上到底收到了什么：帧数 + 最近 8 帧的标识符
   如果打出来是 0x205 之类，说明电调 ID 被设成了 5，而不是 1 */
static void cmd_canid(void)
{
  char b[24];
  uint16_t ids[8];
  uint8_t i;

  Console_Send("frames rx : "); itoa_((int32_t)M2006_RxFrameCount(), b); Console_SendLn(b);
  Console_Send("last 8 id : ");
  M2006_GetRxIdLog(ids, 8u);
  for (i = 0u; i < 8u; i++)
  {
    Console_Send("0x"); hex16(ids[i], b); Console_Send(b); Console_Send(" ");
  }
  Console_SendLn("");
  Console_SendLn("(we only use 0x201..0x204 = esc id 1..4)");
}

static void cmd_remote(const char **args, int argc)
{
  (void)args; (void)argc;
  manual_on = 0u;
  move_on   = 0u;
  Console_SendLn("back to remote stick control.");
}

/* 台架：直接给底盘速度指令，用来验证麦轮解算对不对（不用遥控器）。
   vx/vy 单位 m/s，wz 单位 rad/s。例：
     move 0.5 0 0   前进      move 0 0.5 0    向左平移
     move 0 0 2     逆时针自转  move 0 0 0      停 */
static void cmd_move(const char **args, int argc)
{
  char b[24];

  if (argc < 4)
  {
    Console_SendLn("usage: move <vx> <vy> <wz>   (m/s, m/s, rad/s)");
    return;
  }

  mv_vx   = (float)atof(args[1]);
  mv_vy   = (float)atof(args[2]);
  mv_wz   = (float)atof(args[3]);
  move_on = 1u;
  manual_on = 0u;

  Console_Send("move vx="); f2s(b, mv_vx, 2); Console_Send(b);
  Console_Send(" vy=");     f2s(b, mv_vy, 2); Console_Send(b);
  Console_Send(" wz=");     f2s(b, mv_wz, 2); Console_SendLn(b);
  Console_SendLn("(remote ignored now; type 'remote' to go back)");
}

/* 每秒一行，方便一边拨摇杆一边看数据 */
static void send_monitor_line(void)
{
  char b[24];
  uint8_t i;

  Console_Send("rc=");   itoa_((int32_t)DBUS_IsOnline(), b); Console_Send(b);
  Console_Send(" ch3="); itoa_((int32_t)g_rc.ch3, b); Console_Send(b);
  Console_Send(" ch2="); itoa_((int32_t)g_rc.ch2, b); Console_Send(b);
  Console_Send(" ch0="); itoa_((int32_t)g_rc.ch0, b); Console_Send(b);
  Console_Send(" s1=");  itoa_((int32_t)g_rc.s1, b); Console_Send(b);
  Console_Send(" s2=");  itoa_((int32_t)g_rc.s2, b); Console_Send(b);

  Console_Send(" | v(m/s)");
  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    Console_Send(" ");
    f2s(b, Chassis_GetWheelMps(i), 2);
    Console_Send(b);
  }

  Console_Send(" | rpm");
  for (i = 0u; i < M2006_MOTOR_NUM; i++)
  {
    Console_Send(" ");
    itoa_((int32_t)g_motor[i].speed_rpm, b);
    Console_Send(b);
  }

  Console_Send(" | can=");
  itoa_((int32_t)M2006_OnlineCount(), b); Console_Send(b);
  Console_Send("/");
  itoa_((int32_t)M2006_MOTOR_NUM, b); Console_Send(b);

  Console_Send(" | rx1="); itoa_((int32_t)rx_cnt1, b); Console_Send(b);
  Console_Send(" rx6=");   itoa_((int32_t)rx_cnt6, b); Console_Send(b);
  Console_SendLn("");
}

static int split(char *line, const char **args)
{
  int n = 0;
  char *tok = strtok(line, " \t\r\n");
  while (tok && n < 8) { args[n++] = tok; tok = strtok(NULL, " \t\r\n"); }
  return n;
}

static void process_line(char *line)
{
  const char *args[8];
  int argc = split(line, args);
  if (argc == 0) return;

  if      (strcmp(args[0], "help")   == 0) cmd_help();
  else if (strcmp(args[0], "rc")     == 0) cmd_rc();
  else if (strcmp(args[0], "state")  == 0) cmd_state();
  else if (strcmp(args[0], "mon")    == 0) cmd_mon(args, argc);
  else if (strcmp(args[0], "echo")   == 0) cmd_echo(args, argc);
  else if (strcmp(args[0], "bench")  == 0) cmd_bench(args, argc);
  else if (strcmp(args[0], "wheel")  == 0) cmd_wheel(args, argc);
  else if (strcmp(args[0], "move")   == 0) cmd_move(args, argc);
  else if (strcmp(args[0], "cantest") == 0) cmd_cantest();
  else if (strcmp(args[0], "canid")   == 0) cmd_canid();
  else if (strcmp(args[0], "remote") == 0) cmd_remote(args, argc);
  else
  {
    /* 认不出来的时候，把这一行的原始字节用十六进制打出来——
       是"根本没收到有效字符"还是"收到了别的东西"，一眼就能分清 */
    char h[4];
    size_t i;
    size_t n = strlen(line);

    Console_Send("unknown: "); Console_SendLn(line);
    Console_Send("raw hex : ");
    for (i = 0u; (i < n) && (i < 20u); i++)
    {
      hex8((uint8_t)line[i], h);
      Console_Send(h);
      Console_Send(" ");
    }
    Console_SendLn("");
  }
}

uint8_t Console_ManualActive(void)
{
  return manual_on;
}

int8_t Console_ManualWheel(void)
{
  return manual_idx;
}

float Console_ManualRpm(void)
{
  return manual_rpm;
}

uint32_t Console_RxCount1(void)
{
  return rx_cnt1;
}

uint32_t Console_RxCount6(void)
{
  return rx_cnt6;
}

uint8_t Console_MoveActive(void)
{
  return move_on;
}

void Console_MoveGet(float *vx, float *vy, float *wz)
{
  if (vx != 0) *vx = mv_vx;
  if (vy != 0) *vy = mv_vy;
  if (wz != 0) *wz = mv_wz;
}

/* ------------------------------ 接收部分 ------------------------------ */
static void console_rearm(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte1, 1);
  }
  else if (huart->Instance == USART6)
  {
    (void)HAL_UART_Receive_IT(&huart6, &rx_byte6, 1);
  }
}

/* 由 main.c 的 HAL_UART_RxCpltCallback 调用 */
void Console_UartRxCallback(UART_HandleTypeDef *huart)
{
  rx_activity = 1u;

  if (huart->Instance == USART1)
  {
    rx_buf[rx_head] = rx_byte1;
    rx_head = (uint16_t)((rx_head + 1) % CONSOLE_RB_SIZE);
    rx_cnt1++;
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte1, 1);
  }
  else if (huart->Instance == USART6)
  {
    rx_buf[rx_head] = rx_byte6;
    rx_head = (uint16_t)((rx_head + 1) % CONSOLE_RB_SIZE);
    rx_cnt6++;
    (void)HAL_UART_Receive_IT(&huart6, &rx_byte6, 1);
  }
}

/* 串口出错后 HAL 会中止接收，必须重新启动，否则收不到命令。
   这里无条件重新拉一次：HAL_UART_Receive_IT 内部会自己判断状态，
   正在接收时返回 BUSY 不会打断，已经停了才能重新武装。 */
void Console_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart->Instance != USART1) && (huart->Instance != USART6)) return;

  __HAL_UART_CLEAR_PEFLAG(huart);
  huart->ErrorCode = HAL_UART_ERROR_NONE;

  console_rearm(huart);
}

/* 最近是否收到过字节（main 用来闪灯，不依赖发送方向） */
uint8_t Console_RxActivity(void)
{
  uint8_t a = rx_activity;

  rx_activity = 0u;
  return a;
}

void Console_Init(void)
{
  (void)HAL_UART_Receive_IT(&huart1, &rx_byte1, 1);
  (void)HAL_UART_Receive_IT(&huart6, &rx_byte6, 1);

  Console_SendLn("");
  Console_SendLn("mecanum chassis + M2006 x4 + DR16 (C board)");
  Console_SendLn("Console ready. Type help.");
  cmd_help();
}

void Console_Task(void)
{
  if (mon_on != 0u)
  {
    uint32_t now = HAL_GetTick();

    if ((now - mon_last) >= 1000u)
    {
      mon_last = now;
      send_monitor_line();
    }
  }

  while (rx_head != rx_tail)
  {
    uint8_t c = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1) % CONSOLE_RB_SIZE);
    rx_activity = 1u;

    if (echo_on != 0u)
    {
      char e[2];

      if ((c == '\r') || (c == '\n'))
      {
        Console_SendLn("");
      }
      else
      {
        e[0] = (char)c;
        e[1] = 0;
        Console_Send(e);
      }
    }

    if (c == '\n' || c == '\r')
    {
      if (line_pos > 0)
      {
        line_buf[line_pos] = 0;
        process_line((char *)line_buf);
        line_pos = 0;
      }
    }
    else if (line_pos < CONSOLE_BUF_LEN - 1)
    {
      line_buf[line_pos++] = c;
      line_last = HAL_GetTick();
    }
  }

  /* 兼容不发回车换行的串口助手：停手 150ms 就把这一行执行掉 */
  if ((line_pos > 0u) && ((HAL_GetTick() - line_last) >= 150u))
  {
    line_buf[line_pos] = 0;
    process_line((char *)line_buf);
    line_pos = 0;
  }
}
