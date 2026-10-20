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
  * 16 字节/帧，100 Hz → 1600 B/s，只占 115200 的 14%。
  *
  ******************************************************************************
  */
#ifndef __VOFA_H
#define __VOFA_H

#include <stdint.h>

/** 初始化：传入"怎么写出去"的回调（固件里是 UART，PC 测试里是收集数组）*/
void Vofa_Init(void (*write)(const uint8_t *buf, uint16_t len));

/** 发一帧三通道数据 */
void Vofa_Send3(float ch0, float ch1, float ch2);

#endif /* __VOFA_H */
