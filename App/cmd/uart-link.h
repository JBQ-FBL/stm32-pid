/**
  ******************************************************************************
  * @file    uart-link.h
  * @brief   串口这一条线的"门面"：收命令（中断）+ 发数据（主循环）
  ******************************************************************************
  * @attention
  *
  * 为什么单独一层：
  *   · App/cmd 是纯 C（不认 HAL），它只想要"来一个字节了 → CMD_FeedByte()"
  *   · App/vofa 也只是拼帧，它只想要"把这 16 字节发出去"
  *   把 HAL 的 UART 细节关在本文件里，上面两层就都能在 PC 上测。
  *
  * ⚠️ 前置条件（**当前工程还没做这一步**）：
  *   CubeMX → Connectivity → USART1 → NVIC Settings → 勾 `USART1 global interrupt`，
  *   抢占优先级 2（低于 TIM2 的 1），然后 GENERATE CODE。
  *   没勾就会出现"一收数据就卡死"：`stm32f1xx_it.c` 里没有 USART1_IRQHandler，
  *   启动文件的 weak 桩是个死循环。
  *
  ******************************************************************************
  */
#ifndef __UART_LINK_H
#define __UART_LINK_H

#include "main.h"

/** 初始化：开 USART1 中断 + 启动 1 字节接收。放在 USER CODE 2 */
void UART_Link_Init(UART_HandleTypeDef *huart);

/** 写一串字节（阻塞发送，一帧 16 字节约 1.4 ms @115200）。
  * 同时也作为 Vofa_Init() 的回调 —— 发一帧就调它一次。*/
void UART_Link_Write(const uint8_t *buf, uint16_t len);

/** 诊断：收到过多少字节 / 发生过多少次串口错误（ORE 等）*/
uint32_t UART_Link_RxCount(void);
uint32_t UART_Link_ErrCount(void);

/** 格式化打印（给 '?' 状态查询用；主循环里调，不要在中断里用）*/
void UART_Link_Printf(const char *fmt, ...);

#endif /* __UART_LINK_H */
