/**
  * @file    oled_font.h
  * @brief   SSD1306 8x16 ASCII 点阵字模（自动生成的，别手改）
  *
  * 数据由 ~/stm32/drivers/oled-ssd1306/tools/gen-oled-font.py 从 Linux 系统控制台字体
  * （PSF 格式）程序化提取，非手工抄录、非拷贝自其它 OLED 字库；
  * 每字符 16 字节：[0..7] 上半页，[8..15] 下半页，字节排布用 SSD1306 页寻址的通用约定
  * （与江协教程的 F8X16 相同，便于对照）。详见同目录 README「来源与许可」。
  */
#ifndef __OLED_FONT_H
#define __OLED_FONT_H

#include <stdint.h>

/* ASCII 0x20(空格) ~ 0x7E(~)，共 95 个字符 */
extern const uint8_t OLED_F8x16[95][16];

#endif /* __OLED_FONT_H */
