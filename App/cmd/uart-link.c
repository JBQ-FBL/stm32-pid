/**
  ******************************************************************************
  * @file    uart-link.c
  * @brief   USART1：中断收单字节 → CMD_FeedByte()；阻塞发送给 VOFA/打印
  ******************************************************************************
  */

#include "uart-link.h"
#include "cmd.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *s_uart;
static uint8_t             s_rx_byte;

/* 诊断计数：区分"字节没进来"(rx 不涨) 和 "进来了但命令没生效"(rx 涨) */
static volatile uint32_t   s_rx_count;
static volatile uint32_t   s_err_count;

void UART_Link_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  s_uart = huart;
  s_rx_count = 0U;
  s_err_count = 0U;

  /* 串口中断优先级 2 → 低于 TIM2 的 1，控制中断永远能抢占它 */
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
}

void UART_Link_Write(const uint8_t *buf, uint16_t len)
{
  USART_TypeDef *u;
  uint16_t i;

  if ((s_uart == NULL) || (buf == NULL) || (len == 0U))
  {
    return;
  }

  u = s_uart->Instance;

  /* ★ 为什么不用 HAL_UART_Transmit：
   *   HAL 的 Transmit 和 Receive_IT **共用 huart->Lock**。主循环正发数据时，
   *   接收完成中断里再调 HAL_UART_Receive_IT() 就会返回 HAL_BUSY，
   *   而那一刻如果返回值没人管，**接收就永远关掉了** ——
   *   症状正是"发过一次数据之后串口再也不响应，复位才恢复"。
   *
   *   所以这里直接写寄存器：不碰锁、不碰 HAL 状态机，和 Receive_IT 完全不冲突。
   *   （发送仍可能让 CPU 忙 1~2 ms，接收靠中断，不会丢。）*/
  for (i = 0U; i < len; i++)
  {
    uint32_t guard = 200000UL;
    while (((u->SR & USART_SR_TXE) == 0U) && (guard-- != 0U))
    {
    }
    u->DR = (uint32_t)buf[i];
  }

  /* 等最后一个字节真正移出移位寄存器（否则紧接着复位/掉电会截断尾巴）*/
  {
    uint32_t guard = 200000UL;
    while (((u->SR & USART_SR_TC) == 0U) && (guard-- != 0U))
    {
    }
  }
}

void UART_Link_Printf(const char *fmt, ...)
{
  char    buf[96];
  int     n;
  va_list args;

  if (fmt == NULL)
  {
    return;
  }

  va_start(args, fmt);
  n = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (n <= 0)
  {
    return;
  }
  if (n > (int)sizeof(buf) - 1)
  {
    n = (int)sizeof(buf) - 1;
  }

  UART_Link_Write((const uint8_t *)buf, (uint16_t)n);
}

uint32_t UART_Link_RxCount(void)  { return s_rx_count; }
uint32_t UART_Link_ErrCount(void) { return s_err_count; }

/* ==========================================================================
 * HAL 回调：收满 1 字节 → 喂给命令状态机 → 立刻续订下一字节
 * ========================================================================== */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != s_uart)
  {
    return;
  }

  s_rx_count++;
  CMD_FeedByte((char)s_rx_byte);                       /* 中断里只做状态机，不打印 */

  /* 重新武装：万一返回 HAL_BUSY（理论上现在不会了），再试一次 */
  if (HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) != HAL_OK)
  {
    (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
  }
}

/* 溢出/噪声/帧错误：清了标志再重新武装，否则接收会永久停摆
 * （ORE 是 115200 下最常见的"串口突然死了"元凶）*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != s_uart)
  {
    return;
  }

  s_err_count++;
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);

  huart->RxState = HAL_UART_STATE_READY;
  (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
}
