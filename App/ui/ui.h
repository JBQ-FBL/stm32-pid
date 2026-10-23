#ifndef __UI_H
#define __UI_H
#include "speed-loop.h"
/* Boot-only initialization; failure disables all runtime display work. */
void UI_Init(void);
/* Format the desired four-line screen; performs no bus writes. */
void UI_Refresh(const SpeedLoop_Telemetry_t *tel);
/* Main loop after VOFA: at most one four-byte transfer per control tick. */
void UI_Task(uint32_t tick);
#endif
