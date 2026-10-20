/**
  ******************************************************************************
  * @file    ui.h
  * @brief   OLED 显示（主循环里刷，10 Hz；**绝不在中断里刷**）
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
