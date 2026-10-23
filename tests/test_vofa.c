/**
  ******************************************************************************
  * @file    test_vofa.c
  * @brief   JustFloat 帧格式的主机测试（不需要硬件）
  ******************************************************************************
  *   gcc -std=c11 -Wall -Wextra -I App/vofa tests/test_vofa.c App/vofa/vofa.c -o /tmp/t && /tmp/t
  ******************************************************************************
  */

#include "vofa.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static uint8_t  g_buf[64];
static uint16_t g_len;
static int      g_calls;

static void capture(const uint8_t *buf, uint16_t len)
{
  /* 只留最后一帧 */
  memcpy(g_buf, buf, len);
  g_len = len;
  g_calls++;
}

static int  g_fail;
static void check(int cond, const char *what)
{
  printf("  %s %s\n", cond ? "✓" : "✗", what);
  if (!cond) { g_fail++; }
}

int main(void)
{
  float ch[3];
  float out[3];

  printf("=== JustFloat 帧格式 ===\n");

  /* 1) 没 init 时不能崩 */
  Vofa_Send3(1.0f, 2.0f, 3.0f);
  check(g_calls == 0, "Vofa_Init 之前调用，安全忽略");

  /* 2) 帧长 / 帧尾 */
  Vofa_Init(capture);
  Vofa_Send3(150.0f, 148.5f, -540.0f);

  check(g_calls == 1, "调一次发一帧");
  check(g_len == 16U, "帧长 = 16 字节（3×float + 4 字节帧尾）");
  check(g_buf[12] == 0x00 && g_buf[13] == 0x00 && g_buf[14] == 0x80 && g_buf[15] == 0x7F,
        "帧尾 = 00 00 80 7F");

  /* 3) 三个 float 按小端、按顺序放进去 */
  memcpy(&out[0], &g_buf[0], 4);
  memcpy(&out[1], &g_buf[4], 4);
  memcpy(&out[2], &g_buf[8], 4);
  ch[0] = 150.0f; ch[1] = 148.5f; ch[2] = -540.0f;
  check(memcmp(out, ch, sizeof(ch)) == 0, "三个通道顺序与数值正确（含负数）");

  /* 4) 负零 / 特殊值不破坏帧尾 */
  Vofa_Send3(0.0f, -0.0f, 0.0f);
  check(g_len == 16U && g_buf[15] == 0x7F, "值为 0 时帧尾不受影响");

  Vofa_Init(capture); g_calls = 0;
  Vofa_Poll(100, 0, 1, 2, 3);
  Vofa_Poll(200, 1, 1, 2, 3);
  Vofa_Poll(200, 1, 9, 9, 9);
  check(g_calls == 1 && Vofa_MissedTicks() == 0, "enable sends latest once; same tick does not duplicate");
  Vofa_Poll(201, 1, 4, 5, 6);
  check(g_calls == 2 && Vofa_MissedTicks() == 0, "consecutive control ticks send one frame each");
  Vofa_Poll(205, 1, 7, 8, 9);
  memcpy(out, g_buf, sizeof(out));
  check(g_calls == 3 && Vofa_MissedTicks() == 3 && out[0] == 7, "gap counts missing ticks and sends latest without replay");
  Vofa_Poll(206, 0, 0, 0, 0); Vofa_Poll(300, 1, 1, 2, 3);
  check(g_calls == 4 && Vofa_MissedTicks() == 3, "disabled interval is excluded from missed ticks");
  Vofa_Init(capture); g_calls = 0;
  Vofa_Poll(UINT32_MAX, 1, 0, 0, 0); Vofa_Poll(0, 1, 0, 0, 0);
  check(g_calls == 2 && Vofa_MissedTicks() == 0, "tick wrap is handled without false gap");

  printf("=== 结果：%s ===\n", g_fail == 0 ? "全部通过" : "有失败");
  return g_fail == 0 ? 0 : 1;
}
