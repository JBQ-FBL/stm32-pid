/**
  * @file    oled.h
  * @brief   0.96 寸 OLED（SSD1306，128x64，I2C）驱动 —— 软件模拟 I2C
  *
  * ── 来源 ─────────────────────────────────────────────────
  * 本项目自研：niko 提出需求、pi coding agent 于 2026-10-13 生成，未拷贝任何第三方代码。
  * 参考：SSD1306 数据手册；江协科技《STM32入门教程-2023版》的接口约定（函数命名、
  *       line/column 坐标系、字库字节排布）—— 只是为了方便对照教程看代码。
  * 详见同目录 README「来源与许可」。
  * ────────────────────────────────────────────────────────
  *
  * ============ 接线（江协科技那种 4 针 I2C 模块）============
  *
  *   OLED  VDD / VCC  ──→  板子 3V3     ← 是【电源】，千万别接到 GPIO 上！
  *   OLED  GND        ──→  板子 GND     ← 是【地】，同理
  *   OLED  SCK / SCL  ──→  PB8
  *   OLED  SDA        ──→  PB9
  *
  *   模块板子上自带 SCL/SDA 上拉电阻，不用外接。
  *   （BluePill 右侧排针上 B9 B8 B7 B6 是连着的四个孔，但 B7/B6 是普通 GPIO，
  *     没有电压也不会接地 —— VDD/GND 必须单独接到 3V3 和 GND。）
  *
  * ============ 用法 ============
  *
  *   if (OLED_Init() == 0) {                 // 返回 0 = 屏有应答
  *       OLED_ShowString(1, 1, "Hello");     // 立刻上屏，不需要 Update()
  *       OLED_ShowNum(2, 1, 1234, 4);
  *       OLED_Printf(3, 1, "spd=%d", 100);
  *   }
  *
  * 坐标系：line 1~4（每行 16 像素高），column 1~16（每列 8 像素宽）。
  * 所有显示函数都是「算完立刻通过 I2C 发到屏」，跟江协 2023 版教程一致。
  */
#ifndef __OLED_H
#define __OLED_H

#include "main.h"

/* ==========================================================================
 * 硬件：引脚（**不需要改本文件**）
 *
 * 查找顺序（和 motor-tb6612 驱动同一套风格）：
 *   1. 工程侧 -D 覆盖（CMakeLists.txt 的 target_compile_definitions）
 *   2. CubeMX 的 User Label：把 PB8/PB9 的 User Label 设成 OLED_SCL / OLED_SDA，
 *      main.h 会自动生成 OLED_SCL_Pin / OLED_SCL_GPIO_Port，本驱动直接用
 *   3. 都没有 → 默认 PB8 = SCL，PB9 = SDA
 *
 * 例（换到 PB10/PB11，只改工程，不动库）：
 *   target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
 *       OLED_SCL_GPIO_PORT=GPIOB  OLED_SCL_GPIO_PIN=GPIO_PIN_10
 *       OLED_SDA_GPIO_PORT=GPIOB  OLED_SDA_GPIO_PIN=GPIO_PIN_11)
 * ========================================================================== */
#ifndef OLED_SCL_GPIO_PORT
#  if defined(OLED_SCL_GPIO_Port)
#    define OLED_SCL_GPIO_PORT   OLED_SCL_GPIO_Port
#  else
#    define OLED_SCL_GPIO_PORT   GPIOB
#  endif
#endif

#ifndef OLED_SCL_GPIO_PIN
#  if defined(OLED_SCL_Pin)
#    define OLED_SCL_GPIO_PIN    OLED_SCL_Pin
#  else
#    define OLED_SCL_GPIO_PIN    GPIO_PIN_8     /* PB8 → 模块 SCK/SCL */
#  endif
#endif

#ifndef OLED_SDA_GPIO_PORT
#  if defined(OLED_SDA_GPIO_Port)
#    define OLED_SDA_GPIO_PORT   OLED_SDA_GPIO_Port
#  else
#    define OLED_SDA_GPIO_PORT   GPIOB
#  endif
#endif

#ifndef OLED_SDA_GPIO_PIN
#  if defined(OLED_SDA_Pin)
#    define OLED_SDA_GPIO_PIN    OLED_SDA_Pin
#  else
#    define OLED_SDA_GPIO_PIN    GPIO_PIN_9     /* PB9 → 模块 SDA */
#  endif
#endif

/* 端口时钟：按端口指针打开（F1 的 GPIOA~D，更大的 F1 若有 GPIOE 也能用）。

   为什么用函数而不是宏拼接：`GPIOB` 本身是 `((GPIO_TypeDef *) GPIOB_BASE)` 这种宏，
   直接 `__HAL_RCC_##port##_CLK_ENABLE` 拼接会炸（实测报
   `pasting "__HAL_RCC_" and "(" does not give a valid preprocessing token`）。

   好处：换引脚只改 PORT/PIN，**时钟自动跟着走**，不会出现「忘了改时钟 → 引脚完全不工作
   且编译不报错」那个坑。重复开同一路时钟无害。 */
static inline void OLED_GPIO_PortClockEnable(GPIO_TypeDef *port)
{
	if      (port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
	else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
	else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
	else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
#ifdef __HAL_RCC_GPIOE_CLK_ENABLE
	else if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
#endif
}

#ifndef OLED_SCL_GPIO_CLK_ENABLE
#  define OLED_SCL_GPIO_CLK_ENABLE()  OLED_GPIO_PortClockEnable(OLED_SCL_GPIO_PORT)
#endif

#ifndef OLED_SDA_GPIO_CLK_ENABLE
#  define OLED_SDA_GPIO_CLK_ENABLE()  OLED_GPIO_PortClockEnable(OLED_SDA_GPIO_PORT)
#endif

/* I2C 从机地址：由模块决定，常见 0x78（7 位 0x3C）。
   万一屏死活不亮，改用 -DOLED_I2C_ADDR=0x7A（7 位 0x3D）再试，**不用改本文件**。 */
#ifndef OLED_I2C_ADDR
#define OLED_I2C_ADDR        0x78
#endif

/* ---------- 屏幕尺寸（128x64 是 0.96" 的；128x32 的小屏用 -D 覆盖）---------- */
#ifndef OLED_WIDTH
#define OLED_WIDTH           128
#endif
#ifndef OLED_HEIGHT
#define OLED_HEIGHT          64
#endif

/* ---------- 对外接口 ---------- */

/* 初始化。返回 0 = 屏应答正常；非 0 = 屏没应答（查接线 / 查上拉 / 换 OLED_I2C_ADDR） */
uint8_t OLED_Init(void);

/* Bounded page write: count 1..4, no page crossing; 0=ACK, 1=invalid/NACK. */
uint8_t OLED_WriteSpan(uint8_t page, uint8_t x, const uint8_t *data, uint8_t count);

/* 清屏（全黑） */
void OLED_Clear(void);

/* 在 (line, column) 显示一个字符，字库 8x16，字符范围 0x20~0x7E */
void OLED_ShowChar(uint8_t line, uint8_t column, char ch);

/* 显示字符串，写满一行会自动换到下一行 */
void OLED_ShowString(uint8_t line, uint8_t column, const char *str);

/* 格式化显示，例如 OLED_Printf(1, 1, "spd=%d", speed);（最长一行 16 个字符，超出会被截断） */
void OLED_Printf(uint8_t line, uint8_t column, const char *format, ...);

/* 显示无符号十进制数，定长 length 位（前面补 0），例如 (1,1,12,3) → "012" */
void OLED_ShowNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);

/* 显示带符号十进制数，length 不含符号位，例如 (1,1,-12,3) → "-012" */
void OLED_ShowSignedNum(uint8_t line, uint8_t column, int32_t number, uint8_t length);

/* 显示十六进制数（大写），定长 length 位 */
void OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);

/* 显示二进制数，定长 length 位 */
void OLED_ShowBinNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);

/* 显示浮点数，如 (1,1,3.14,1,2) → "3.14" */
void OLED_ShowFloatNum(uint8_t line, uint8_t column, float number, uint8_t intLength, uint8_t fracLength);

#endif /* __OLED_H */
