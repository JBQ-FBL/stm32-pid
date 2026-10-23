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

  if (CMD_GetVofa() == 0U)
  {
    UART_Link_Printf("ready. S200 | X=stop | A=step | C=calib | V=vofa | ?=status\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    SpeedLoop_Telemetry_t tel;
    uint32_t now = HAL_GetTick();

    /* Serialize only read/clear operations; never print inside a critical section. */
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    uint8_t status_req = CMD_TakeStatusRequest();
    uint8_t vofa_toggle = CMD_TakeVofaToggle();
    if (vofa_toggle != 0U) { CMD_SetVofa(!CMD_GetVofa()); }
    uint8_t vofa_on = CMD_GetVofa();
    __set_PRIMASK(irq_state);
    if ((vofa_toggle != 0U) && (vofa_on == 0U)) { UART_Link_Printf("VOFA OFF\r\n"); }

    /* '?' 状态查询 */
    if (status_req != 0U)
    {
      SpeedLoop_GetTelemetry(&tel);
      /* 增益用 1 位小数打印：调参步长 0.5，用 (int)(x+0.5) 会把 12.5 显示成 13 */
      int kp10 = (int)(tel.kp * 10.0f + 0.5f);
      int ki10 = (int)(tel.ki * 10.0f + 0.5f);
      int kd100 = (int)(tel.kd * 100.0f + 0.5f);
      int kv10 = (int)(tel.kv * 10.0f + 0.5f);

      /* f = 故障码（0 正常），给上位机 / 自动调参判「还能不能继续跑」。
       * 字段固定、以空格分隔，解析器（../tools/pid_autotune.py）按 key=value 取。*/
      UART_Link_Printf("tgt=%d act=%d out=%d kp=%d.%d ki=%d.%d kd=%d.%02d kv=%d.%d tick=%lu rx=%lu err=%lu f=%u miss=%lu\r\n",
                       (int)(tel.target_rpm + (tel.target_rpm >= 0 ? 0.5f : -0.5f)),
                       (int)(tel.actual_rpm + (tel.actual_rpm >= 0 ? 0.5f : -0.5f)),
                       (int)(tel.out_permille + (tel.out_permille >= 0 ? 0.5f : -0.5f)),
                       kp10 / 10, kp10 % 10,
                       ki10 / 10, ki10 % 10,
                       kd100 / 100, kd100 % 100,
                       kv10 / 10, kv10 % 10,
                       (unsigned long)tel.tick,
                       (unsigned long)UART_Link_RxCount(),
                       (unsigned long)UART_Link_ErrCount(),
                       (unsigned)SpeedLoop_FaultCode(),
                       (unsigned long)Vofa_MissedTicks());

      /* 标定模式额外打一行（不打印浮点：用 ×10 的整数避开 newlib printf 浮点支持）*/
      if (tel.calib != 0U)
      {
        UART_Link_Printf("CALIB enc=%ld revx10=%ld\r\n",
                         (long)tel.enc_total,
                         (long)(tel.enc_revs * 10.0f));
      }
    }

    SpeedLoop_GetTelemetry(&tel);

    /* Live VOFA keeps its three-channel format and never replays stale samples. */
    Vofa_Poll(tel.tick, vofa_on, tel.target_rpm, tel.actual_rpm, tel.out_permille);

    /* Text feedback is useful in terminal mode, but silent while streaming. */
    {
      static int32_t last_target;
      static uint8_t last_calib, last_fault;
      uint8_t fault = SpeedLoop_FaultCode();
      if (vofa_on == 0U)
      {
        if ((int32_t)tel.target_rpm != last_target)
        {
          UART_Link_Printf("TGT %d OUT %d\r\n", (int)tel.target_rpm, (int)tel.out_permille);
        }
        if (tel.calib != last_calib) { UART_Link_Printf("CALIB %s\r\n", tel.calib ? "ON" : "OFF"); }
        if (fault != last_fault) { UART_Link_Printf("FAULT %u\r\n", (unsigned)fault); }
      }
      last_target = (int32_t)tel.target_rpm;
      last_calib = tel.calib;
      last_fault = fault;
    }
    if ((now - s_t_ui) >= 1000U)
    {
      s_t_ui = now;
      UI_Refresh(&tel); /* Format only: pads each row, no I2C transaction. */
    }
    UI_Task(tel.tick); /* One bounded display slice, after wave transmission. */
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
  MOTOR_EmergencyStop();
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
