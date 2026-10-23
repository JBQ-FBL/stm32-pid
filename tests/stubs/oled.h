#ifndef TEST_OLED_H
#define TEST_OLED_H
#include <stdint.h>
uint8_t OLED_Init(void);
uint8_t OLED_WriteSpan(uint8_t page, uint8_t x, const uint8_t *data, uint8_t count);
#endif
