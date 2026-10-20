/**
  ******************************************************************************
  * @file    vofa.c
  * @brief   JustFloat 组帧（无 HAL 依赖，可在 PC 上单测）
  ******************************************************************************
  */

#include "vofa.h"
#include <string.h>

#define VOFA_CHANNELS    3U
#define VOFA_TAIL_LEN    4U
#define VOFA_FRAME_LEN   ((VOFA_CHANNELS * 4U) + VOFA_TAIL_LEN)   /* 16 */

/* float 的 +inf（0x7F800000）小端展开 = 00 00 80 7F，VOFA+ 靠它判断帧尾 */
static const uint8_t s_tail[VOFA_TAIL_LEN] = { 0x00U, 0x00U, 0x80U, 0x7FU };

static void (*s_write)(const uint8_t *buf, uint16_t len);

void Vofa_Init(void (*write)(const uint8_t *buf, uint16_t len))
{
  s_write = write;
}

void Vofa_Send3(float ch0, float ch1, float ch2)
{
  uint8_t  frame[VOFA_FRAME_LEN];

  if (s_write == 0)
  {
    return;
  }

  /* memcpy 而不是强制转换/指针别名：既避免对齐问题，也天然按"本机字节序"排布
     （Cortex-M3 和 x86 都是小端，和 VOFA+ 的约定一致）*/
  memcpy(&frame[0],  &ch0, 4U);
  memcpy(&frame[4],  &ch1, 4U);
  memcpy(&frame[8],  &ch2, 4U);
  memcpy(&frame[12], s_tail, VOFA_TAIL_LEN);

  s_write(frame, (uint16_t)VOFA_FRAME_LEN);
}
