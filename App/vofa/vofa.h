/**
  ******************************************************************************
  * @file    vofa.h
  * @brief   VOFA+ 的 JustFloat 帧格式（只拼帧，不发字节）
  ******************************************************************************
  * @attention
  *
  * 为什么单独成一个模块：**它不 include 任何 HAL 头**，只依赖一个"把字节写出去"
  * 的函数指针。所以在 PC 上也能测（回调里把字节攒进数组，断言帧长 16、帧尾正确）。
  *
  * 一帧（JustFloat，小端 float）：
  *
  *     [ch0 float][ch1 float][ch2 float][0x00 0x00 0x80 0x7F]
  *      4 字节     4 字节     4 字节      ↑ 固定帧尾 = float 的 +inf
  *
  * 本工程三条线：目标 rpm / 实际 rpm / 输出 ‰。
  * 16 字节/帧，200 Hz → 3200 B/s，115200 8N1 线速占用约 28%。
  *
  ******************************************************************************
  */
#ifndef __VOFA_H
#define __VOFA_H

#include <stdint.h>

/** 初始化：传入"怎么写出去"的回调（固件里是 UART，PC 测试里是收集数组）*/
void Vofa_Init(void (*write)(const uint8_t *buf, uint16_t len));

/* Call each main-loop iteration with a coherent control snapshot.
 * One latest frame per tick, no replay; disabled ticks do not count as missed. */
void Vofa_Poll(uint32_t tick, uint8_t enabled, float target, float actual, float out);
uint32_t Vofa_MissedTicks(void);

/** Send one three-channel frame immediately. */
void Vofa_Send3(float ch0, float ch1, float ch2);

#endif /* __VOFA_H */
