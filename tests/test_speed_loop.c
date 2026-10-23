/* Exercise the real command parser, PID and control loop against a fake encoder/actuator. */
#include "main.h"
#include "cmd.h"
#include "speed-loop.h"
#include <stdio.h>
#include <stdlib.h>

uint32_t test_primask;
TIM_HandleTypeDef htim1 = {(void *)1}, htim2 = {TIM2}, htim3 = {(void *)3};
static float measured;
static int16_t output, delta;
static unsigned resets, checks;
#define CHECK(c) do { checks++; if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); exit(1); } } while (0)
HAL_StatusTypeDef MOTOR_Init(TIM_HandleTypeDef *h) { (void)h; output = 0; return HAL_OK; }
HAL_StatusTypeDef MOTOR_ENC_Init(TIM_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef MOTOR_SetDuty(int16_t v) { output = v; return HAL_OK; }
void MOTOR_EmergencyStop(void) { output = 0; }
void MOTOR_ENC_Reset(void) { resets++; }
int16_t MOTOR_ENC_ReadDelta(void) { return delta; }
float MOTOR_ENC_UpdateSpeed(uint32_t dt) { CHECK(dt == 5U); return measured; }
float MOTOR_ENC_GetCountsPerRev(void) { return 1320.0f; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t c) { (void)h; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *h, uint32_t c) { (void)h; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h) { (void)h; return HAL_OK; }
void Error_Handler(void) { abort(); }
static void feed(const char *s) { while (*s) { CMD_FeedByte(*s++); } }
static void ticks(unsigned n) { while (n--) { SpeedLoop_Tick(); } }
static SpeedLoop_Telemetry_t tel(void) { SpeedLoop_Telemetry_t t; SpeedLoop_GetTelemetry(&t); return t; }
static void init(float rpm) { measured = rpm; delta = 7; test_primask = 0; SpeedLoop_Init(); }

int main(void)
{
  init(40.0f); SpeedLoop_Start(); ticks(1);
  CHECK(output == 0 && tel().target_rpm == 0);
  feed("S100\n"); measured = 80; ticks(1); CHECK(output > 0);
  feed("S20X"); ticks(1); CHECK(output == 0 && tel().target_rpm == 0);
  CMD_SetTargetRpm(100); ticks(5); CHECK(output == 0); /* no explicit run */
  feed("S100\n"); ticks(1); CHECK(output > 0);
  feed("S200\nX"); ticks(1); CHECK(output == 0);
  feed("XS100\n"); ticks(1); CHECK(output > 0); /* ordering within one tick */
  feed("S0\n"); ticks(1); CHECK(output == 0);

  for (unsigned negative = 0; negative < 2; negative++)
  {
    init(0); feed(negative ? "S-150\n" : "S150\n");
    ticks(39); CHECK(SpeedLoop_FaultCode() == 0 && output != 0);
    ticks(1); CHECK(SpeedLoop_FaultCode() == 2 && output == 0 && tel().out_permille == 0);
    feed("S100\n"); measured = 80; ticks(1); CHECK(output == 0); /* S cannot clear fault */
    feed("X"); ticks(1); CHECK(SpeedLoop_FaultCode() == 0 && output == 0);
  }
  init(100); feed("S-150\n"); ticks(19); CHECK(SpeedLoop_FaultCode() == 0);
  ticks(1); CHECK(SpeedLoop_FaultCode() == 1 && output == 0);
  init(-100); feed("S150\n"); ticks(20); CHECK(SpeedLoop_FaultCode() == 1 && output == 0);

  init(150); feed("S150\n"); ticks(1); feed("S-150\n");
  ticks(60); CHECK(SpeedLoop_FaultCode() == 0); /* reversal grace */
  ticks(19); CHECK(SpeedLoop_FaultCode() == 0);
  ticks(1); CHECK(SpeedLoop_FaultCode() == 1 && output == 0);
  init(150); feed("S150\n"); ticks(1); feed("S-150\n"); ticks(59);
  measured = -150; ticks(25); CHECK(SpeedLoop_FaultCode() == 0 && output < 0);

  init(150); feed("S150\n"); ticks(1); feed("S0\n"); ticks(1);
  feed("S-150\n"); ticks(60); CHECK(SpeedLoop_FaultCode() == 0);
  measured = -150; ticks(1); CHECK(SpeedLoop_FaultCode() == 0);

  for (unsigned negative = 0; negative < 2; negative++)
  {
    init(negative ? -5 : 5); feed(negative ? "S-200\n" : "S200\n");
    ticks(99); CHECK(SpeedLoop_FaultCode() == 0);
    ticks(1); CHECK(SpeedLoop_FaultCode() == 3 && output == 0);
  }
  SpeedLoop_ClearFault(); CHECK(SpeedLoop_FaultCode() == 3); /* deferred */
  ticks(1); CHECK(SpeedLoop_FaultCode() == 0 && output == 0);

  init(0); ticks(123); feed("A"); ticks(1);
  CHECK(CMD_GetAutoStep() && tel().target_rpm == 0);
  ticks(399); CHECK(tel().target_rpm == 0);
  measured = 150; ticks(1); CHECK(tel().target_rpm == 150);
  ticks(599); CHECK(tel().target_rpm == 150);
  ticks(1); CHECK(tel().target_rpm == 0);
  ticks(1400); CHECK(tel().target_rpm == 150); /* next period, delay included */
  feed("A"); ticks(1); CHECK(!CMD_GetAutoStep() && output == 0);
  feed("S80\nA"); ticks(1); CHECK(CMD_GetAutoStep() && tel().target_rpm == 0);
  feed("S80\n"); measured = 80; ticks(1); CHECK(!CMD_GetAutoStep() && tel().target_rpm == 80);

  feed("A"); ticks(1); feed("C"); ticks(2);
  CHECK(tel().calib && output == 0 && !CMD_GetAutoStep() && tel().enc_total == 14);
  feed("S200\nA"); ticks(1); CHECK(output == 0 && !CMD_GetAutoStep());
  feed("C"); ticks(1); CHECK(!tel().calib && tel().enc_total == 0 && output == 0);
  ticks(10); CHECK(output == 0);
  feed("S80\n"); ticks(1); CHECK(output > 0);
  CHECK(resets >= 2);

  test_primask = 1;
  (void)tel(); CHECK(test_primask == 1);
  SpeedLoop_SetCalib(1); CHECK(test_primask == 1);
  SpeedLoop_ClearFault(); CHECK(test_primask == 1);
  SpeedLoop_SetTarget(80); CHECK(test_primask == 1);
  test_primask = 0; ticks(1); CHECK(tel().calib && output == 0);
  printf("speed loop: %u checks passed (includes each sampling-period check)\n", checks);
  return 0;
}
