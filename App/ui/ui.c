/**
  ******************************************************************************
  * @file    ui.c
  * @brief   OLED 四行显示：目标 / 输出 / 实际 / 状态
  ******************************************************************************
  * @attention
  *
  * 只显示整数：vsnprintf 里的 %f 要开浮点打印支持（-u _printf_float），
  * 为了不为了显示去动链接参数，这里把 float 四舍五入成 int 再打印。
  *
  * 刷新节奏：主循环 10 Hz（软件 I²C 全屏一帧几十 ms，放太快会拖慢主循环）。
  *
  ******************************************************************************
  */

#include "ui.h"
#include "oled.h"

#include <stdio.h>

#define UI_RETRY_MAX   3U

void UI_Init(void)
{
  uint8_t i;

  for (i = 0U; i < UI_RETRY_MAX; i++)
  {
    if (OLED_Init() == 0U)      /* 0 = 屏有应答 */
    {
      OLED_ShowString(1, 1, "PID  speed loop");
      return;
    }
  }

  /* 没有屏也继续跑：所有信息都还能从串口（VOFA / '?'）看 */
}

void UI_Refresh(const SpeedLoop_Telemetry_t *tel)
{
  static uint8_t s_have_screen = 1U;

  if (tel == NULL)
  {
    return;
  }

  if (s_have_screen == 0U)
  {
    return;
  }

  /* ---- 标定模式：四行全给计数，不显示 rpm（此时本来就没在控速）---- */
  if (tel->calib != 0U)
  {
    int whole = (int)tel->enc_revs;
    int frac  = (int)((tel->enc_revs - (float)whole) * 10.0f);
    if (frac < 0) { frac = -frac; }

    OLED_ShowString(1, 1, "CALIB  (C=exit)");
    OLED_Printf(2, 1, "ENC %8ld", (long)tel->enc_total);
    OLED_Printf(3, 1, "REV %d.%d", whole, frac);
    OLED_ShowString(4, 1, "turn 10 revs");
    return;
  }

  OLED_Printf(1, 1, "TGT %4d rpm", (int)(tel->target_rpm + 0.5f));

  if (tel->feedback != 0U)
  {
    OLED_Printf(2, 1, "ACT %4d rpm", (int)(tel->actual_rpm + 0.5f));
  }
  else
  {
    OLED_ShowString(2, 1, "ACT   --  open");   /* 开环：没有反馈，不编数据 */
  }

  OLED_Printf(3, 1, "OUT %4d pcm", (int)(tel->out_permille));

  if (SpeedLoop_Fault() != NULL)
  {
    OLED_Printf(4, 1, "FAULT %s", SpeedLoop_Fault());
  }
  else
  {
    /* 三个增益都要看得见：Kp/Ki 留 1 位小数（调参步长 0.5），Kd 留 2 位（常用 0.05）*/
    int kp10  = (int)(tel->kp * 10.0f + 0.5f);
    int ki10  = (int)(tel->ki * 10.0f + 0.5f);
    int kd100 = (int)(tel->kd * 100.0f + 0.5f);

    OLED_Printf(4, 1, "K%d.%d I%d.%d D.%02d",
                kp10 / 10, kp10 % 10,
                ki10 / 10, ki10 % 10,
                kd100 / 100, kd100 % 100);
  }
}
