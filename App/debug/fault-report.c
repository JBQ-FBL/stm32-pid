/**
  ******************************************************************************
  * @file    fault-report.c
  * @brief   HardFault 现场打印（寄存器直写 USART1，115200 8N1）
  ******************************************************************************
  */

#include "fault-report.h"

/* SCB（System Control Block）里和故障有关的寄存器 */
#define SCB_CFSR    (*(volatile uint32_t *)0xE000ED28UL)  /* 可配置故障状态 */
#define SCB_HFSR    (*(volatile uint32_t *)0xE000ED2CUL)  /* 硬故障状态 */
#define SCB_MMFAR   (*(volatile uint32_t *)0xE000ED34UL)  /* MemManage 出错地址 */
#define SCB_BFAR    (*(volatile uint32_t *)0xE000ED38UL)  /* BusFault 出错地址 */

/* USART1 寄存器直写（PA9 已在 CubeMX 里配成 AF_PP，这里只发不收）*/
#define U1_SR       (*(volatile uint32_t *)0x40013800UL)
#define U1_DR       (*(volatile uint32_t *)0x40013804UL)
#define U1_SR_TXE   (1UL << 7)

static void putc_raw(char c)
{
  /* 等发送寄存器空；加个上限，防止 USART 没初始化时死等 */
  uint32_t guard = 1000000UL;
  while (((U1_SR & U1_SR_TXE) == 0U) && (guard-- != 0U))
  {
  }
  U1_DR = (uint32_t)(uint8_t)c;
}

static void puts_raw(const char *s)
{
  while (*s != '\0')
  {
    putc_raw(*s++);
  }
}

static void put_hex32(uint32_t v)
{
  static const char hex[] = "0123456789ABCDEF";
  int i;

  puts_raw("0x");
  for (i = 28; i >= 0; i -= 4)
  {
    putc_raw(hex[(v >> (uint32_t)i) & 0xFU]);
  }
}

/** 把 CFSR 的位翻成人话，直接指出是哪类错 */
static void explain_cfsr(uint32_t cfsr)
{
  if ((cfsr & (1UL << 25)) != 0U) { puts_raw("  DIVBYZERO  (整除零)\r\n"); }
  if ((cfsr & (1UL << 24)) != 0U) { puts_raw("  UNALIGNED  (非对齐访问)\r\n"); }
  if ((cfsr & (1UL << 19)) != 0U) { puts_raw("  LSPERR     (FP 惰性保存出错)\r\n"); }
  if ((cfsr & (1UL << 18)) != 0U) { puts_raw("  STKOF      (栈溢出!)\r\n"); }
  if ((cfsr & (1UL << 17)) != 0U) { puts_raw("  NOCP       (没开协处理器)\r\n"); }
  if ((cfsr & (1UL << 16)) != 0U) { puts_raw("  INVSTATE   (非法状态/指令编码)\r\n"); }
  if ((cfsr & (1UL << 15)) != 0U) { puts_raw("  UNDEFINSTR (未定义指令)\r\n"); }
  if ((cfsr & (1UL << 10)) != 0U) { puts_raw("  IMPRECISERR(非精确总线错误)\r\n"); }
  if ((cfsr & (1UL <<  9)) != 0U) { puts_raw("  PRECISERR  (精确总线错误, 看 BFAR)\r\n"); }
  if ((cfsr & (1UL <<  8)) != 0U) { puts_raw("  IBUSERR    (取指总线错误)\r\n"); }
  if ((cfsr & (1UL <<  1)) != 0U) { puts_raw("  DACCVIOL   (数据访问被 MPU 拒绝)\r\n"); }
  if ((cfsr & (1UL <<  0)) != 0U) { puts_raw("  IACCVIOL   (取指被 MPU 拒绝)\r\n"); }
}

void Fault_Report(uint32_t sp)
{
  /* 异常入口时硬件压栈顺序：R0 R1 R2 R3 R12 LR PC xPSR */
  const uint32_t *f = (const uint32_t *)sp;

  puts_raw("\r\n\r\n!!! HARDFAULT !!!\r\n");

  puts_raw("CFSR   = "); put_hex32(SCB_CFSR); puts_raw("\r\n");
  explain_cfsr(SCB_CFSR);
  puts_raw("HFSR   = "); put_hex32(SCB_HFSR); puts_raw("\r\n");
  puts_raw("MMFAR  = "); put_hex32(SCB_MMFAR); puts_raw("\r\n");
  puts_raw("BFAR   = "); put_hex32(SCB_BFAR); puts_raw("\r\n");

  puts_raw("R0     = "); put_hex32(f[0]); puts_raw("\r\n");
  puts_raw("R1     = "); put_hex32(f[1]); puts_raw("\r\n");
  puts_raw("R2     = "); put_hex32(f[2]); puts_raw("\r\n");
  puts_raw("R3     = "); put_hex32(f[3]); puts_raw("\r\n");
  puts_raw("R12    = "); put_hex32(f[4]); puts_raw("\r\n");
  puts_raw("LR     = "); put_hex32(f[5]); puts_raw("\r\n");
  puts_raw("PC     = "); put_hex32(f[6]); puts_raw("   <- 出错的那条指令\r\n");
  puts_raw("xPSR   = "); put_hex32(f[7]); puts_raw("\r\n");

  puts_raw("把 PC 和上面几行贴给 agent，用 objdump 就能定位到源码行\r\n");
  puts_raw("提示：STKOF = 栈溢出（本工程默认 1 KB，printf 用得多时可加到 2 KB）\r\n");
}
