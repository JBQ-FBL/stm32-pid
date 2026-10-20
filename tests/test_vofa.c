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

  printf("=== 结果：%s ===\n", g_fail == 0 ? "全部通过" : "有失败");
  return g_fail == 0 ? 0 : 1;
}
