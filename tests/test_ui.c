#include "ui.h"
#include "oled_font.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t pixels[8][128];
static unsigned writes, init_calls, checks;
static uint32_t tick;
static uint8_t init_fail, write_fail;
static const char *fault;
#define CHECK(c) do { checks++; if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); exit(1); } } while (0)
const char *SpeedLoop_Fault(void) { return fault; }
uint8_t OLED_Init(void) { init_calls++; memset(pixels, 0, sizeof(pixels)); return init_fail; }
uint8_t OLED_WriteSpan(uint8_t page, uint8_t x, const uint8_t *data, uint8_t n)
{
  CHECK(page < 8 && x + n <= 128 && n <= 4);
  writes++;
  if (write_fail) { return 1; }
  memcpy(&pixels[page][x], data, n);
  return 0;
}
static void advance(unsigned n)
{
  while (n--)
  {
    unsigned before = writes;
    UI_Task(++tick);
    CHECK(writes - before <= 1); /* one slice per tick, including dirty mode switches */
    UI_Task(tick); CHECK(writes - before <= 1);
  }
}
static void row_is(unsigned row, const char *text)
{
  for (unsigned c = 0; c < 16; c++)
  {
    char ch = c < strlen(text) ? text[c] : ' ';
    CHECK(memcmp(&pixels[row * 2][c * 8], &OLED_F8x16[ch - ' '][0], 8) == 0);
    CHECK(memcmp(&pixels[row * 2 + 1][c * 8], &OLED_F8x16[ch - ' '][8], 8) == 0);
  }
}
int main(void)
{
  SpeedLoop_Telemetry_t t = {0};
  UI_Init(); CHECK(init_calls == 1);
  t.target_rpm = 200; t.actual_rpm = -100; t.kp = 14; t.ki = 5;
  UI_Refresh(&t); CHECK(writes == 0); /* formatting never touches I2C */
  advance(260); row_is(0, "TGT  200 rpm"); row_is(1, "ACT -100 rpm");
  unsigned before = writes; UI_Refresh(&t); advance(100); CHECK(writes == before);
  t.target_rpm = 5; UI_Refresh(&t); advance(260); row_is(0, "TGT    5 rpm");
  fault = "NO_PULSE"; UI_Refresh(&t); advance(260); row_is(3, "FAULT NO_PULSE");
  fault = NULL; UI_Refresh(&t); advance(260); row_is(3, "K14.0 I5.0 D.00");
  t.calib = 1; t.enc_total = -13200; t.enc_revs = -10;
  UI_Refresh(&t); advance(260); row_is(1, "ENC -13200"); row_is(2, "REV -10.0");
  t.calib = 0; UI_Refresh(&t); advance(1); /* desired changes while a character is in flight */
  t.target_rpm = 123; UI_Refresh(&t); advance(300); row_is(0, "TGT  123 rpm");
  write_fail = 1; t.target_rpm = 222; UI_Refresh(&t); advance(1);
  before = writes; advance(100); CHECK(writes == before);
  write_fail = 0; init_fail = 1; before = init_calls; UI_Init(); CHECK(init_calls == before + 3);
  before = writes; UI_Refresh(&t); advance(300); CHECK(writes == before);
  printf("UI: %u checks passed (includes transfer bounds and per-tick budgets)\n", checks);
  return 0;
}
