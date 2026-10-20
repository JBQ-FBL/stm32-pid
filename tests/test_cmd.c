/**
  ******************************************************************************
  * @file    test_cmd.c
  * @brief   cmd.c 的主机端单测（不需要单片机）
  ******************************************************************************
  * 跑法（在 ~/stm32/PID 目录下）：
  *
  *   gcc -std=c11 -Wall -Wextra -O2 -Isrc/cmd src/cmd/test_cmd.c src/cmd/cmd.c \
  *       -o /tmp/test_cmd && /tmp/test_cmd
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include "cmd.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, fmt, ...)                                                 \
  do {                                                                        \
    if (cond) { g_pass++; printf("  \xE2\x9C\x93 " fmt "\n", ##__VA_ARGS__); } \
    else      { g_fail++; printf("  \xE2\x9C\x97 " fmt "   <-- 失败\n", ##__VA_ARGS__); } \
  } while (0)

static void feed(const char *s)
{
  while (*s != '\0')
  {
    CMD_FeedByte(*s++);
  }
}

int main(void)
{
  printf("=== cmd.c 主机端单测 ===\n");

  printf("[1] 基本给速\n");
  CMD_Reset();
  feed("S200\n");
  CHECK(CMD_GetTargetRpm() == 200, "S200 → 200（实际 %d）", (int)CMD_GetTargetRpm());
  CHECK(CMD_GetAutoStep() == 0U, "给了目标后自动阶跃自动关闭");

  printf("[2] 大小写、\\r 结尾、负号\n");
  CMD_Reset();
  feed("s50\r");
  CHECK(CMD_GetTargetRpm() == 50, "小写 s50 → 50（实际 %d）", (int)CMD_GetTargetRpm());
  feed("S-100\n");
  CHECK(CMD_GetTargetRpm() == -100, "S-100 → -100（实际 %d）", (int)CMD_GetTargetRpm());

  printf("[3] 限幅（两侧都要）\n");
  CMD_Reset();
  feed("S999\n");
  CHECK(CMD_GetTargetRpm() == CMD_TARGET_MAX, "S999 超上限 → 限到 %d（实际 %d）",
        CMD_TARGET_MAX, (int)CMD_GetTargetRpm());
  feed("S-999\n");
  CHECK(CMD_GetTargetRpm() == -CMD_TARGET_MAX, "S-999 超下限 → 限到 -%d（实际 %d）",
        CMD_TARGET_MAX, (int)CMD_GetTargetRpm());

  printf("[4] 垃圾输入不能改变目标\n");
  CMD_Reset();
  feed("S100\n");
  feed("hello world\n");
  CHECK(CMD_GetTargetRpm() == 100, "乱码行被忽略（仍是 100，实际 %d）", (int)CMD_GetTargetRpm());

  feed("200\n");
  CHECK(CMD_GetTargetRpm() == 100, "不带 S 的数字不生效（仍是 100，实际 %d）", (int)CMD_GetTargetRpm());

  feed("S12x3\n");
  CHECK(CMD_GetTargetRpm() == 100, "数字里混字母 → 整行作废（仍是 100，实际 %d）", (int)CMD_GetTargetRpm());

  feed("S\n");
  CHECK(CMD_GetTargetRpm() == 100, "只有 S 没有数字 → 不生效（仍是 100，实际 %d）", (int)CMD_GetTargetRpm());

  printf("[5] 超长行（灌 40 个字符）\n");
  CMD_Reset();
  feed("S1\n");
  feed("S1234567890123456789012345678901234567890\n");
  CHECK(CMD_GetTargetRpm() == 1, "超长行整行作废，目标没被改动（实际 %d）", (int)CMD_GetTargetRpm());
  feed("S999999999\n");                      /* 11 字符，行内但数值溢出 */
  CHECK(CMD_GetTargetRpm() == CMD_TARGET_MAX, "行内但数值超大 → 限到 %d（实际 %d）",
        CMD_TARGET_MAX, (int)CMD_GetTargetRpm());

  printf("[6] 分片到达 / 一包多条\n");
  CMD_Reset();
  feed("S1");
  feed("50\n");
  CHECK(CMD_GetTargetRpm() == 150, "分三次到达也能拼成 S150（实际 %d）", (int)CMD_GetTargetRpm());

  feed("S100\nS200\n");
  CHECK(CMD_GetTargetRpm() == 200, "一包里两条命令，以后一条为准（实际 %d）", (int)CMD_GetTargetRpm());

  printf("[7] 急停 X\n");
  CMD_Reset();
  feed("S200\n");
  feed("X\n");
  CHECK(CMD_GetTargetRpm() == 0, "X 之后目标归零（实际 %d）", (int)CMD_GetTargetRpm());
  CHECK(CMD_TakeStopRequest() == 1U, "第一次取到急停请求");
  CHECK(CMD_TakeStopRequest() == 0U, "第二次取不到了（读后清零，不会重复触发）");

  printf("[8] A / ? 请求\n");
  CMD_Reset();
  feed("A\n");
  CHECK(CMD_TakeAutoStepToggle() == 1U, "A 置位自动阶跃切换请求");
  CHECK(CMD_TakeAutoStepToggle() == 0U, "读后清零");
  feed("?\n");
  CHECK(CMD_TakeStatusRequest() == 1U, "? 置位状态打印请求");

  printf("[9] 复位\n");
  CMD_Reset();
  CHECK(CMD_GetTargetRpm() == 0 && CMD_GetAutoStep() == 1U, "Reset 后目标 0、自动阶跃重新开启");

  printf("\n=== 结果：%d 通过，%d 失败 ===\n", g_pass, g_fail);
  return (g_fail == 0) ? 0 : 1;
}
