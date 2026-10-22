/**
  ******************************************************************************
  * @file    ui.h
  * @brief   OLED 显示（主循环里刷，**默认 1 Hz**；绝不在中断里刷）
  *
  * 为什么这么慢：OLED 是软件 I²C，刷一帧要 20~40 ms，会阻塞主循环，
  * 而 VOFA 的发送也在主循环 —— 以前 100 ms 刷一次把 VOFA 拖到 6.6 Hz，
  * 导致 50 ms 的上升沿采不到。现改为 1 Hz（VOFA ~180 Hz）。
  * 改刷新率：Core/Src/main.c 里的 `(now - s_t_ui) >= 1000U`。
  ******************************************************************************
  */
#ifndef __UI_H
#define __UI_H

#include "speed-loop.h"

/** 初始化屏幕。失败重试几次就放弃（不 while(1) 卡死，方便没有屏时也能跑）*/
void UI_Init(void);

/** 刷新四行内容 */
void UI_Refresh(const SpeedLoop_Telemetry_t *tel);

#endif /* __UI_H */
