/* 5 ms control owner: command requests -> encoder -> PID + feedforward -> protection. */
#include "main.h"
#include "tim.h"
#include "motor.h"
#include "motor_encoder.h"
#include "pid.h"
#include "cmd.h"
#include "speed-loop.h"

float g_pid_kp = SPEED_LOOP_KP;
float g_pid_ki = SPEED_LOOP_KI;
float g_pid_kd = SPEED_LOOP_KD;
float g_pid_kv = SPEED_LOOP_KV;

static PID_t s_pid;
static volatile SpeedLoop_Telemetry_t s_tel;
static uint32_t s_tick;
enum { FAULT_NONE = 0, FAULT_ENC_DIR, FAULT_NO_PULSE, FAULT_STALL };
static volatile uint8_t s_fault;
static const char *const s_fault_name[] = { NULL, "ENC_DIR", "NO_PULSE", "STALL" };
static volatile uint8_t s_calib;
static int32_t s_enc_total;
static uint8_t s_stopped;
static volatile uint8_t s_clear_req;
/* 0 = no request, 1 = exit, 2 = enter. Written only inside caller critical section. */
static volatile uint8_t s_calib_req;
static uint16_t s_nopulse_cnt, s_stall_cnt, s_dirbad_cnt;
static uint16_t s_settle_ticks, s_reverse_ticks;
static float s_deadband_target;
static int8_t s_last_direction;
static uint32_t s_step_start;

static float absf(float v) { return v < 0.0f ? -v : v; }
static float clampf(float v, float lim)
{
  if (v > lim) { return lim; }
  if (v < -lim) { return -lim; }
  return v;
}

static void reset_control(void)
{
  PID_Reset(&s_pid);
  s_nopulse_cnt = s_stall_cnt = s_dirbad_cnt = 0U;
  s_settle_ticks = s_reverse_ticks = 0U;
  s_deadband_target = -1.0f;
}

static void stop_control(void)
{
  s_stopped = 1U;
  CMD_SetTargetRpm(0);
  CMD_SetAutoStep(0U);
  reset_control();
  MOTOR_EmergencyStop();
}

static void set_calib(uint8_t on)
{
  stop_control();
  s_calib = on;
  s_enc_total = 0;
  MOTOR_ENC_Reset();
}

static int32_t autostep_value(uint32_t elapsed)
{
  uint32_t t = elapsed % SPEED_LOOP_STEP_PERIOD;
  if (t < SPEED_LOOP_STEP_DELAY) { return 0; }
  if (t < SPEED_LOOP_STEP_DELAY + SPEED_LOOP_STEP_HOLD) { return SPEED_LOOP_STEP_RPM; }
  return 0;
}

void SpeedLoop_Init(void)
{
  CMD_Reset();
  s_tick = s_step_start = 0U;
  s_last_direction = 0;
  s_fault = FAULT_NONE;
  s_calib = s_clear_req = s_calib_req = 0U;
  s_enc_total = 0;
  s_stopped = (CMD_AUTOSTEP_DEFAULT == 0);
  g_pid_kp = SPEED_LOOP_KP;
  g_pid_ki = SPEED_LOOP_KI;
  g_pid_kd = SPEED_LOOP_KD;
  g_pid_kv = SPEED_LOOP_KV;
  s_tel = (SpeedLoop_Telemetry_t){0};
  s_tel.kp = g_pid_kp;
  s_tel.ki = g_pid_ki;
  s_tel.kd = g_pid_kd;
  s_tel.kv = g_pid_kv;
  if (MOTOR_Init(&htim1) != HAL_OK) { Error_Handler(); }
  if (MOTOR_ENC_Init(&htim3) != HAL_OK) { Error_Handler(); }
  PID_Init(&s_pid, g_pid_kp, g_pid_ki, g_pid_kd,
           SPEED_LOOP_OUT_MAX, SPEED_LOOP_INTEG_MAX);
  reset_control();
}

void SpeedLoop_Start(void)
{
  if ((HAL_TIM_PWM_Start(&htim1, MOTOR_PWM_TIM_CHANNEL) != HAL_OK) ||
      (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) ||
      (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK))
  {
    Error_Handler();
  }
}

void SpeedLoop_Tick(void)
{
  int32_t requested_target;
  uint8_t run_req, stop_req, calib_toggle, auto_toggle;
  float target, meas, duty = 0.0f;
  int16_t out;
  s_tick++;
  CMD_PollTimeout();
  /* UART cannot interrupt TIM2; UART masks IRQs during its parser transition. */
  run_req = CMD_TakeTargetRequest(&requested_target);
  stop_req = CMD_TakeStopRequest();
  calib_toggle = CMD_TakeCalibToggle();
  auto_toggle = CMD_TakeAutoStepToggle();
  if ((stop_req != 0U) || (s_clear_req != 0U))
  {
    uint8_t explicit_clear = s_clear_req;
    s_clear_req = 0U;
    s_fault = FAULT_NONE;
    stop_control();
    /* An explicit API clear means stop, irrespective of queued run requests. */
    if (explicit_clear != 0U) { run_req = auto_toggle = 0U; }
  }
  if ((calib_toggle != 0U) || (s_calib_req != 0U))
  {
    uint8_t on = (s_calib_req != 0U) ? (s_calib_req == 2U) : !s_calib;
    s_calib_req = 0U;
    set_calib(on);
    run_req = auto_toggle = 0U;
  }
  if (s_calib != 0U)
  {
    s_enc_total += MOTOR_ENC_ReadDelta();
    CMD_SetTargetRpm(0);
    CMD_SetAutoStep(0U);
    MOTOR_EmergencyStop();
    target = meas = 0.0f;
  }
  else
  {
    if ((auto_toggle != 0U) && (s_fault == FAULT_NONE))
    {
      uint8_t on = !CMD_GetAutoStep();
      stop_control();
      CMD_SetAutoStep(on);
      if (on != 0U) { s_step_start = s_tick; s_stopped = 0U; }
    }
    if (run_req != 0U)
    {
      CMD_SetTargetRpm(requested_target);
      CMD_SetAutoStep(0U);
      s_stopped = (requested_target == 0) || (s_fault != FAULT_NONE);
      if (s_stopped != 0U) { reset_control(); }
    }
    if (CMD_GetAutoStep() != 0U)
    {
      CMD_SetTargetRpm(autostep_value(s_tick - s_step_start));
    }
    target = (float)CMD_GetTargetRpm();
    meas = MOTOR_ENC_UpdateSpeed(5U);
    if (CMD_TakeGains(&g_pid_kp, &g_pid_ki, &g_pid_kd, &g_pid_kv) != 0U ||
        s_pid.kp != g_pid_kp || s_pid.ki != g_pid_ki || s_pid.kd != g_pid_kd)
    {
      PID_SetGains(&s_pid, g_pid_kp, g_pid_ki, g_pid_kd);
    }
    if (target != s_deadband_target)
    {
      int8_t direction = target > 0.0f ? 1 : target < 0.0f ? -1 : 0;
      if ((direction != 0) && (s_last_direction != 0) && (direction != s_last_direction))
      {
        s_reverse_ticks = SPEED_LOOP_REVERSE_GRACE_MS / 5U;
        s_dirbad_cnt = 0U;
      }
      if (direction != 0) { s_last_direction = direction; }
      s_deadband_target = target;
      s_settle_ticks = 0U;
    }
    else if (s_settle_ticks < SPEED_LOOP_DEADBAND_DELAY_MS / 5U)
    {
      s_settle_ticks++;
    }
    if ((s_stopped == 0U) && (s_fault == FAULT_NONE))
    {
      float p_dead = s_settle_ticks >= SPEED_LOOP_DEADBAND_DELAY_MS / 5U
                     ? SPEED_LOOP_DEADBAND_RPM : 0.0f;
      duty = clampf(g_pid_kv * target +
                    PID_UpdateEx(&s_pid, target, meas, SPEED_LOOP_DT_S, p_dead),
                    SPEED_LOOP_OUT_MAX);
      if (s_reverse_ticks != 0U)
      {
        s_reverse_ticks--;
        s_dirbad_cnt = 0U;
      }
      else if ((absf(target) > 50.0f) && (absf(meas) > 5.0f) &&
               ((target > 0.0f) != (meas > 0.0f)))
      {
        if (++s_dirbad_cnt >= SPEED_LOOP_DIRBAD_MS / 5U) { s_fault = FAULT_ENC_DIR; }
      }
      else { s_dirbad_cnt = 0U; }
      if ((absf(target) > 50.0f) && (absf(meas) < 3.0f))
      {
        if (++s_nopulse_cnt >= SPEED_LOOP_NOPULSE_MS / 5U) { s_fault = FAULT_NO_PULSE; }
      }
      else { s_nopulse_cnt = 0U; }
      if ((absf(duty) > SPEED_LOOP_OUT_MAX * 0.85f) && (absf(meas) < 10.0f))
      {
        if (++s_stall_cnt >= SPEED_LOOP_STALL_MS / 5U) { s_fault = FAULT_STALL; }
      }
      else { s_stall_cnt = 0U; }
    }
  }
  /* Protection runs before the single actuator write: faulting tick has zero output. */
  if ((s_fault != FAULT_NONE) || (s_stopped != 0U) || (s_calib != 0U))
  {
    duty = 0.0f;
    MOTOR_EmergencyStop();
    if (s_fault != FAULT_NONE) { s_stopped = 1U; CMD_SetAutoStep(0U); }
  }
  else { (void)MOTOR_SetDuty((int16_t)duty); }
  out = (int16_t)duty;
  s_tel.target_rpm = target;
  s_tel.actual_rpm = meas;
  s_tel.out_permille = (float)out;
  s_tel.tick = s_tick;
  s_tel.enc_total = s_enc_total;
  float cpr = MOTOR_ENC_GetCountsPerRev();
  s_tel.enc_revs = cpr > 0.0f ? (float)s_enc_total / cpr : 0.0f;
  s_tel.calib = s_calib;
  s_tel.kp = g_pid_kp;
  s_tel.ki = g_pid_ki;
  s_tel.kd = g_pid_kd;
  s_tel.kv = g_pid_kv;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) { SpeedLoop_Tick(); }
}

void SpeedLoop_SetTarget(float rpm)
{
  uint32_t irq_state = __get_PRIMASK();
  __disable_irq();
  CMD_RequestTargetRpm((int32_t)rpm);
  __set_PRIMASK(irq_state);
}

void SpeedLoop_GetTelemetry(SpeedLoop_Telemetry_t *out)
{
  if (out == NULL) { return; }
  uint32_t irq_state = __get_PRIMASK();
  __disable_irq();
  *out = s_tel;
  __set_PRIMASK(irq_state);
}

const char *SpeedLoop_Fault(void)
{
  uint8_t f = s_fault;
  return s_fault_name[f];
}

uint8_t SpeedLoop_FaultCode(void) { return s_fault; }

void SpeedLoop_ClearFault(void)
{
  uint32_t irq_state = __get_PRIMASK();
  __disable_irq();
  s_clear_req = 1U;
  __set_PRIMASK(irq_state);
}

void SpeedLoop_SetCalib(uint8_t on)
{
  uint32_t irq_state = __get_PRIMASK();
  __disable_irq();
  s_calib_req = on != 0U ? 2U : 1U;
  __set_PRIMASK(irq_state);
}

uint8_t SpeedLoop_GetCalib(void) { return s_calib; }
