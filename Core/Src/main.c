/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dbus.h"
#include "m2006.h"
#include "chassis.h"
#include "console.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 板载三色 LED 的引脚宏由 CubeMX 生成在 main.h 里：LED_B / LED_G / LED_R */
#define LED_ALL_PIN   (LED_B_Pin | LED_G_Pin | LED_R_Pin)

/* 摇杆死区（原始通道值，满量程 ±660），抑制回中时的零漂抖动 */
#define STICK_DEADBAND   15

/* 摇杆方向：三个都是 ±1，实测出来的。想改方向就把对应那个符号翻过来。
     期望效果：
       ch3 左摇杆上下  推上  -> 车前进
       ch2 左摇杆左右  推左  -> 车向左平移
       ch0 右摇杆左右  推右  -> 车顺时针转
     （vy 的正方向是"向左"、wz 的正方向是"逆时针"，这个是底盘解算约定好的，
       已经在 move 命令里验证过；这里只负责把摇杆翻译成它。） */
#define STICK_SIGN_VX   (+1.0f)
#define STICK_SIGN_VY   (+1.0f)
#define STICK_SIGN_WZ   (-1.0f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* CAN 链路维护 / 状态提示用（文件作用域，避免在 statements 之后再声明变量） */
static uint32_t s_last_can_try   = 0u;
static uint8_t  s_can_ready_seen = 0u;
static uint8_t  s_rc_online_seen = 0u;
static uint32_t s_led_tick       = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void  Led_Update(void);
static float Stick_Normalize(int16_t v);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 摇杆归一化：原始通道值(-660 ~ +660) -> -1.0 ~ +1.0。
   先扣掉死区再除以剩余行程，这样摇杆推到底刚好是 ±1。 */
static float Stick_Normalize(int16_t v)
{
  const float span = (float)(DBUS_CH_MID - DBUS_CH_MIN);   /* 660 */
  float n;

  if (v > STICK_DEADBAND)
  {
    v = (int16_t)(v - STICK_DEADBAND);
  }
  else if (v < -STICK_DEADBAND)
  {
    v = (int16_t)(v + STICK_DEADBAND);
  }
  else
  {
    return 0.0f;
  }

  n = (float)v / (span - (float)STICK_DEADBAND);

  if (n >  1.0f) n =  1.0f;
  if (n < -1.0f) n = -1.0f;

  return n;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 上电自检：板载三色 LED 快闪 3 次，说明固件在跑（不依赖串口/上位机）。
     LED 的 GPIO 已经由 MX_GPIO_Init 初始化过，这里不用再配寄存器。 */
  {
    uint8_t i;

    for (i = 0u; i < 3u; i++)
    {
      HAL_GPIO_TogglePin(GPIOH, LED_ALL_PIN);
      HAL_Delay(120);
      HAL_GPIO_TogglePin(GPIOH, LED_ALL_PIN);
      HAL_Delay(120);
    }
  }

  /* 四个 M2006 + C610：初始化速度环和 CAN1（电调没上电也不会卡死） */
  M2006_Init();
  Chassis_Init();

  /* DR16 遥控(USART3) 和 调试串口(USART1/USART6) */
  DBUS_Init();
  Console_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ---- 1) 决定四个轮子的目标转速 ---- */
    if (DBUS_IsNewData() != 0u)
    {
      DBUS_ClearNewData();
    }

    if (Console_MoveActive() != 0u)
    {
      /* 台架运动学模式（move 命令）：直接给底盘速度，用来验证麦轮解算 */
      float mvx, mvy, mwz;

      Console_MoveGet(&mvx, &mvy, &mwz);
      Chassis_SetVelocity(mvx, mvy, mwz);
      Chassis_Update();
    }
    else if (Console_ManualActive() != 0u)
    {
      /* 台架模式（bench / wheel）：直接给电机转速，跳过麦轮解算，也不受
         遥控影响，方便架空标定轮序和转向；退出用 remote 命令。 */
      int8_t  wi = Console_ManualWheel();
      float   rpm = Console_ManualRpm();
      float   tgt[M2006_MOTOR_NUM];
      uint8_t k;

      for (k = 0u; k < M2006_MOTOR_NUM; k++)
      {
        tgt[k] = ((wi < 0) || ((int8_t)k == wi)) ? rpm : 0.0f;
      }
      M2006_SetAllTargetRpm(tgt);
    }
    else if (DBUS_IsOnline() == 0u)
    {
      /* 遥控掉线（接收机没数据 / 遥控器没开）-> 失控保护，四个轮子都停 */
      Chassis_Stop();
    }
    else if ((g_rc.s1 == DBUS_SW_DOWN) || (g_rc.s2 == DBUS_SW_DOWN))
    {
      /* 任一拨杆打到最下 -> 停机 */
      Chassis_Stop();
    }
    else
    {
      /* 正常遥控：左摇杆前后/左右 + 右摇杆左右 -> 底盘三自由度速度，
         再经麦轮逆解算分到四个轮子 */
      Chassis_SetVelocity(STICK_SIGN_VX * Stick_Normalize(g_rc.ch3) * CHASSIS_MAX_VX_MPS,
                          STICK_SIGN_VY * Stick_Normalize(g_rc.ch2) * CHASSIS_MAX_VY_MPS,
                          STICK_SIGN_WZ * Stick_Normalize(g_rc.ch0) * CHASSIS_MAX_WZ_RPS);
      Chassis_Update();
    }

    /* ---- 2) 四个速度环 -> 四路电流指令 -> 一帧 0x200 发出去 ---- */
    M2006_ControlLoop();
    M2006_SendCmd();

    /* ---- 3) CAN 链路维护：每 100ms 检查一次（正常时函数内部直接返回）。
             电调后上电、接线松动、总线 off 都能自动恢复；状态变化只在
             串口打印一次，方便排查。 ---- */
    if ((uint32_t)(HAL_GetTick() - s_last_can_try) >= 100u)
    {
      s_last_can_try = HAL_GetTick();
      M2006_CanRetry();

      if (M2006_IsReady() != s_can_ready_seen)
      {
        s_can_ready_seen = M2006_IsReady();
        Console_SendLn((s_can_ready_seen != 0u)
                       ? "CAN1 ready (1Mbps)."
                       : "CAN1 not ready - check ESC power / CANH-CANL / 120ohm.");
      }
    }

    /* ---- 4) 遥控上下线提示 ---- */
    if (DBUS_IsOnline() != s_rc_online_seen)
    {
      s_rc_online_seen = DBUS_IsOnline();
      Console_SendLn((s_rc_online_seen != 0u)
                     ? "remote online."
                     : "remote lost -> motor stopped.");
    }

    /* ---- 5) 调试串口 + 状态指示灯 ---- */
    Console_Task();
    Led_Update();

    HAL_Delay(1);   /* 控制周期约 1ms（1kHz） */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* 板载三色 LED 状态指示（高电平点亮，见用户手册）：
     蓝 : 500ms 翻转一次的心跳，一直在闪 = 固件活着
     绿 : 遥控在线常亮，掉线熄灭
     红 : CAN 没起来、或者有电调反馈断了 -> 快闪；四个电调都正常时熄灭 */
static void Led_Update(void)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - s_led_tick) >= 500u)
  {
    s_led_tick = now;
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
  }

  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,
                    (DBUS_IsOnline() != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  if ((M2006_IsReady() == 0u) || (M2006_OnlineCount() < M2006_MOTOR_NUM))
  {
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin,
                      (((now / 50u) & 1u) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  }
}

/* ---- HAL 串口回调统一入口：USART3 = DBUS 遥控，USART1/USART6 = 调试串口 ----
   整个工程里只在这一处定义这两个回调，避免和别的模块重复定义。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    DBUS_UartRxCallback(huart);
  }
  else
  {
    Console_UartRxCallback(huart);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    DBUS_UartErrorCallback(huart);
  }
  else
  {
    Console_UartErrorCallback(huart);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 致命错误（时钟或串口初始化失败）：让板载三色 LED 快闪，
     不用调试器也能看出是初始化失败。CAN 初始化失败不会走到这里
     （见 can.c 里的说明）。注意这里可能是在 MX_GPIO_Init 之前出错，
     所以自己把 PH10~12 配成输出。 */
  uint32_t t;

  __HAL_RCC_GPIOH_CLK_ENABLE();
  GPIOH->MODER   = (GPIOH->MODER & ~(0x3Fu << 20)) | (0x15u << 20);
  GPIOH->OTYPER &= ~(uint32_t)LED_ALL_PIN;
  GPIOH->OSPEEDR |= (0x3Fu << 20);

  while (1)
  {
    GPIOH->ODR ^= (uint32_t)LED_ALL_PIN;
    t = HAL_GetTick();
    while ((HAL_GetTick() - t) < 100u)
    {
    }
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
