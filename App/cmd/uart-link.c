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

void UART_Link_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  s_uart = huart;

  /* 串口中断优先级 2 → 低于 TIM2 的 1，控制中断永远能抢占它 */
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
}

void UART_Link_Write(const uint8_t *buf, uint16_t len)
{
  if ((s_uart == NULL) || (buf == NULL) || (len == 0U))
  {
    return;
  }

  /* 阻塞发送。16 字节 @115200 ≈ 1.4 ms，主循环里发 100 Hz 完全来得及；
     接收方向有中断兜着，所以不会丢字节。*/
  (void)HAL_UART_Transmit(s_uart, (uint8_t *)buf, len, 20U);
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

/* ==========================================================================
 * HAL 回调：收满 1 字节 → 喂给命令状态机 → 立刻续订下一字节
 * ========================================================================== */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != s_uart)
  {
    return;
  }

  CMD_FeedByte((char)s_rx_byte);                       /* 中断里只做状态机，不打印 */
  (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);   /* 重新武装 */
}
