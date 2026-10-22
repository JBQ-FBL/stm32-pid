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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "speed-loop.h"
#include "ui.h"
#include "vofa.h"
#include "uart-link.h"
#include "cmd.h"
#include "motor.h"      /* MOTOR_EmergencyStop() */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t s_t_vofa;   /* 上次发 VOFA 的时刻（ms） */
static uint32_t s_t_ui;     /* 上次刷 OLED 的时刻（ms） */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* ── 应用层初始化（都在 App/ 里，CubeMX 重新生成不会动）──
     顺序：先执行器/控制器，再显示，最后串口和心跳 —— 这样"心跳一响就开始算" */
  SpeedLoop_Init();              /* MOTOR_Init(&htim1) + PID_Init */
  UI_Init();                     /* OLED；没插屏也照样往下跑 */
  UART_Link_Init(&huart1);       /* 开 USART1 中断，开始收命令 */
  Vofa_Init(UART_Link_Write);    /* VOFA 的输出接到串口 */
  SpeedLoop_Start();             /* PWM 起振 + TIM2 的 5 ms 心跳 */

  UART_Link_Printf("\r\nready v7. S200/S-100 | C=calib | KP/KI/KD/KV=n | V=vofa | A=autostep | X=stop | ?=status\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    SpeedLoop_Telemetry_t tel;
    uint32_t now = HAL_GetTick();

    /* 急停请求（串口 X）：立刻断电，目标已被 cmd 清成 0。
       ⚠️ 同时清故障计数：否则 ENC_DIR / NO_PULSE 一旦触发，就只能复位单片机才能恢复。
       安全：cmd 里 X 已经把目标和自动阶跃都置 0，所以清完不会自己转起来。*/
    if (CMD_TakeStopRequest() != 0U)
    {
      SpeedLoop_ClearFault();
      UART_Link_Printf("STOP (fault cleared)\r\n");
    }

    /* '?' 状态查询 */
    if (CMD_TakeStatusRequest() != 0U)
    {
      SpeedLoop_GetTelemetry(&tel);
      /* 增益用 1 位小数打印：调参步长 0.5，用 (int)(x+0.5) 会把 12.5 显示成 13 */
      int kp10 = (int)(tel.kp * 10.0f + 0.5f);
      int ki10 = (int)(tel.ki * 10.0f + 0.5f);
      int kd100 = (int)(tel.kd * 100.0f + 0.5f);
      int kv10 = (int)(tel.kv * 10.0f + 0.5f);

      /* f = 故障码（0 正常），给上位机 / 自动调参判「还能不能继续跑」。
       * 字段固定、以空格分隔，解析器（../tools/pid_autotune.py）按 key=value 取。*/
      UART_Link_Printf("tgt=%d act=%d out=%d kp=%d.%d ki=%d.%d kd=%d.%02d kv=%d.%d tick=%lu rx=%lu err=%lu f=%u\r\n",
                       (int)(tel.target_rpm + 0.5f),
                       (int)(tel.actual_rpm + 0.5f),
                       (int)(tel.out_permille + 0.5f),
                       kp10 / 10, kp10 % 10,
                       ki10 / 10, ki10 % 10,
                       kd100 / 100, kd100 % 100,
                       kv10 / 10, kv10 % 10,
                       (unsigned long)tel.tick,
                       (unsigned long)UART_Link_RxCount(),
                       (unsigned long)UART_Link_ErrCount(),
                       (unsigned)SpeedLoop_FaultCode());

      /* 标定模式额外打一行（不打印浮点：用 ×10 的整数避开 newlib printf 浮点支持）*/
      if (tel.calib != 0U)
      {
        UART_Link_Printf("CALIB enc=%ld revx10=%ld\r\n",
                         (long)tel.enc_total,
                         (long)(tel.enc_revs * 10.0f));
      }
    }

    SpeedLoop_GetTelemetry(&tel);

    /* 目标一变就回显：终端里能立刻看到命令生效（以前 S 命令是"静默生效"）*/
    {
      static int32_t s_last_tgt;
      if ((int32_t)(tel.target_rpm + 0.5f) != s_last_tgt)
      {
        s_last_tgt = (int32_t)(tel.target_rpm + 0.5f);
        UART_Link_Printf("TGT %d  OUT %d\r\n",
                         (int)s_last_tgt,
                         (int)(tel.out_permille + 0.5f));
      }
    }

    /* `C` 命令：进/出编码器标定模式 */
    if (CMD_TakeCalibToggle() != 0U)
    {
      uint8_t on = (SpeedLoop_GetCalib() != 0U) ? 0U : 1U;
      SpeedLoop_SetCalib(on);
      if (on != 0U)
      {
        UART_Link_Printf("CALIB ON  - 手转输出轴正好 10 圈，然后发 ? 或看 OLED\r\n");
      }
      else
      {
        UART_Link_Printf("CALIB OFF\r\n");
      }
    }

    /* `V` 命令：开/关 VOFA 波形流（默认关，否则纯文本串口满屏二进制"乱码"）*/
    if (CMD_TakeVofaToggle() != 0U)
    {
      CMD_SetVofa((CMD_GetVofa() != 0U) ? 0U : 1U);
      UART_Link_Printf("VOFA %s\r\n", (CMD_GetVofa() != 0U) ? "ON" : "OFF");
    }

    /* VOFA 100 Hz（三条线：目标 / 实际 / 输出）—— 只在打开时才发 */
    if ((CMD_GetVofa() != 0U) && ((now - s_t_vofa) >= 5U))   /* 200 Hz：提高上升时间测量分辨率 */
    {
      s_t_vofa = now;
      Vofa_Send3(tel.target_rpm, tel.actual_rpm, tel.out_permille);
    }

    /* OLED 刷新间隔：软件 I²C 刷一帧要几十 ms，**会阻塞主循环**。
     * 原来 100 ms 时，VOFA 的 100 Hz 被拖到实测 **6.6 Hz**（150 ms 才一个点），
     * 而考核要求「上升时间 < 50 ms」—— 根本采不到上升沿。
     * 调参期间用 1 Hz（VOFA 能跑到 90 Hz 左右）；只想看屏上数字时改回 100U。*/
    if ((now - s_t_ui) >= 1000U)
    {
      s_t_ui = now;
      UI_Refresh(&tel);
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
