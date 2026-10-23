/* Main-loop UI: format a desired screen, then transfer at most four bytes per tick. */
#include "ui.h"
#include "oled.h"
#include "oled_font.h"
#include <stdio.h>
#include <string.h>

#define UI_RETRY_MAX 3U
static char s_wanted[64], s_shown[64];
static uint8_t s_have_screen, s_scan, s_part, s_active;
static char s_char;
static uint32_t s_task_tick;

static int round_signed(float v) { return (int)(v >= 0.0f ? v + 0.5f : v - 0.5f); }

static void set_line(uint8_t row, const char *text)
{
  size_t n = strlen(text);
  if (n > 16U) { n = 16U; }
  memset(&s_wanted[row * 16U], ' ', 16U);
  memcpy(&s_wanted[row * 16U], text, n);
}

void UI_Init(void)
{
  s_have_screen = s_scan = s_part = s_active = 0U;
  s_task_tick = 0U;
  memset(s_wanted, ' ', sizeof(s_wanted));
  memset(s_shown, ' ', sizeof(s_shown)); /* OLED_Init clears physical RAM. */
  for (uint8_t i = 0U; i < UI_RETRY_MAX; i++)
  {
    if (OLED_Init() == 0U) { s_have_screen = 1U; break; }
  }
}

void UI_Refresh(const SpeedLoop_Telemetry_t *tel)
{
  char line[40];
  if ((tel == NULL) || (s_have_screen == 0U)) { return; }
  if (tel->calib != 0U)
  {
    set_line(0U, "CALIB (C=exit)");
    snprintf(line, sizeof(line), "ENC %ld", (long)tel->enc_total);
    set_line(1U, line);
    int rev10 = round_signed(tel->enc_revs * 10.0f);
    unsigned mag = (unsigned)(rev10 < 0 ? -rev10 : rev10);
    snprintf(line, sizeof(line), "REV %s%u.%u", rev10 < 0 ? "-" : "", mag / 10U, mag % 10U);
    set_line(2U, line);
    set_line(3U, "turn 10 revs");
    return;
  }
  snprintf(line, sizeof(line), "TGT %4d rpm", round_signed(tel->target_rpm));
  set_line(0U, line);
  snprintf(line, sizeof(line), "ACT %4d rpm", round_signed(tel->actual_rpm));
  set_line(1U, line);
  snprintf(line, sizeof(line), "OUT %4d pcm", round_signed(tel->out_permille));
  set_line(2U, line);
  const char *fault = SpeedLoop_Fault();
  if (fault != NULL) { snprintf(line, sizeof(line), "FAULT %s", fault); }
  else
  {
    int kp10 = round_signed(tel->kp * 10.0f);
    int ki10 = round_signed(tel->ki * 10.0f);
    int kd100 = round_signed(tel->kd * 100.0f);
    snprintf(line, sizeof(line), "K%d.%d I%d.%d D.%02d", kp10 / 10, kp10 % 10,
             ki10 / 10, ki10 % 10, kd100);
  }
  set_line(3U, line);
}

void UI_Task(uint32_t tick)
{
  if ((s_have_screen == 0U) || (tick == s_task_tick)) { return; }
  s_task_tick = tick;
  if (s_active == 0U)
  {
    for (uint8_t n = 0U; n < 64U; n++)
    {
      if (s_wanted[s_scan] != s_shown[s_scan])
      {
        s_char = s_wanted[s_scan];
        s_part = 0U;
        s_active = 1U;
        break;
      }
      s_scan = (s_scan + 1U) % 64U;
    }
    if (s_active == 0U) { return; }
  }
  unsigned char ch = (unsigned char)s_char;
  if ((ch < ' ') || (ch > '~')) { ch = '?'; }
  uint8_t page = (s_scan / 16U) * 2U + s_part / 2U;
  uint8_t x = (s_scan % 16U) * 8U + (s_part % 2U) * 4U;
  if (OLED_WriteSpan(page, x, &OLED_F8x16[ch - ' '][s_part * 4U], 4U) != 0U)
  {
    s_have_screen = 0U; /* Disconnect: stop wasting time until reboot. */
    return;
  }
  if (++s_part == 4U)
  {
    s_shown[s_scan] = s_char;
    s_active = 0U;
    s_scan = (s_scan + 1U) % 64U;
  }
}
