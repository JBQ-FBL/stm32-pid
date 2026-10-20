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

  UART_Link_Printf("\r\nready. S200 / S-100 / X / V(vofa) / A / ?\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    SpeedLoop_Telemetry_t tel;
    uint32_t now = HAL_GetTick();

    /* 急停请求（串口 X）：立刻断电，目标已被 cmd 清成 0 */
    if (CMD_TakeStopRequest() != 0U)
    {
      MOTOR_EmergencyStop();
      UART_Link_Printf("STOP\r\n");
    }

    /* '?' 状态查询 */
    if (CMD_TakeStatusRequest() != 0U)
    {
      SpeedLoop_GetTelemetry(&tel);
      UART_Link_Printf("tgt=%d act=%d out=%d kp=%d ki=%d tick=%lu\r\n",
                       (int)(tel.target_rpm + 0.5f),
                       (int)(tel.actual_rpm + 0.5f),
                       (int)(tel.out_permille + 0.5f),
                       (int)(tel.kp + 0.5f),
                       (int)(tel.ki + 0.5f),
                       (unsigned long)tel.tick);
    }

    SpeedLoop_GetTelemetry(&tel);

    /* `V` 命令：开/关 VOFA 波形流（默认关，否则纯文本串口满屏二进制"乱码"）*/
    if (CMD_TakeVofaToggle() != 0U)
    {
      CMD_SetVofa((CMD_GetVofa() != 0U) ? 0U : 1U);
      UART_Link_Printf("VOFA %s\r\n", (CMD_GetVofa() != 0U) ? "ON" : "OFF");
    }

    /* VOFA 100 Hz（三条线：目标 / 实际 / 输出）—— 只在打开时才发 */
    if ((CMD_GetVofa() != 0U) && ((now - s_t_vofa) >= 10U))
    {
      s_t_vofa = now;
      Vofa_Send3(tel.target_rpm, tel.actual_rpm, tel.out_permille);
    }

    /* OLED 10 Hz（软件 I²C 刷屏慢，别更快）*/
    if ((now - s_t_ui) >= 100U)
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
