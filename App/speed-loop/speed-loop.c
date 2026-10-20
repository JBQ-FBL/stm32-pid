/**
  ******************************************************************************
  * @file    speed-loop.c
  * @brief   速度环：5 ms 一拍，读编码器 → PID → 前馈 → 限幅 → MOTOR_SetDuty
  ******************************************************************************
  * @attention
  *
  * 这一版（SPEED_LOOP_FEEDBACK=0）是**开环**的：
  *
  *     目标 200 rpm  ──×Kv(2.70‰/rpm)──►  540‰  ──►  TB6612
  *
  * 没有反馈，所以转速会随电池电压/负载漂 —— 这**正是**闭环要解决的问题，
  * 也是答辩时"为什么闭环"最好的现场证据：同一目标，手捏一下轮子转速就掉。
  *
  * 编码器到货后把 SPEED_LOOP_FEEDBACK 改成 1，同一条链就变成：
  *
  *     目标 200 rpm ──►(+)──► PID ──►(+)──► 限幅 ──► 电机 ──► 编码器 ──┐
  *                     ▲                                            │
  *                     └──────────────── rpm ────────────────────────┘
  *                     ▲
  *                     └── 前馈 Kv×target（不走 PID，直接加到输出上）
  *
  * ⚠️ 中断里只做四件事，禁止 printf / OLED / HAL_Delay。
  *
  ******************************************************************************
  */

#include "main.h"
#include "tim.h"        /* htim1 / htim2 / htim3 */
#include "motor.h"
#include "pid.h"
#include "cmd.h"
#include "speed-loop.h"

/* ==========================================================================
 * 私有状态
 * ========================================================================== */

static PID_t    s_pid;
static volatile SpeedLoop_Telemetry_t s_tel;
static volatile uint32_t s_tick;

/* 故障：0 = 正常 */
enum { FAULT_NONE = 0, FAULT_ENC_DIR, FAULT_NO_PULSE, FAULT_STALL };
static volatile uint8_t s_fault;

static const char *s_fault_name[] = {
  NULL, "ENC_DIR", "NO_PULSE", "STALL"
};

#if SPEED_LOOP_SIM_PLANT
/* 一阶惯性模型：v' = (K·u − v)/T。K、T 先按估算值，实测后再改 */
#define SIM_K_RPM_PER_PERMILLE   (333.0f / 900.0f)
#define SIM_T_S                  0.08f
static float s_sim_rpm;
#endif

/* ==========================================================================
 * 内部小工具
 * ========================================================================== */

static float clampf(float v, float lim)
{
  if (v >  lim) { return  lim; }
  if (v < -lim) { return -lim; }
  return v;
}

/** 自动阶跃：返回本拍应该给的目标（开着时才用）*/
static float autostep_value(uint32_t tick)
{
  uint32_t t = tick % SPEED_LOOP_STEP_PERIOD;

  if (t < SPEED_LOOP_STEP_DELAY)                                  { return 0.0f; }
  if (t < SPEED_LOOP_STEP_DELAY + SPEED_LOOP_STEP_HOLD)           { return (float)SPEED_LOOP_STEP_RPM; }
  return 0.0f;
}

/* ==========================================================================
 * 初始化 / 启动
 * ========================================================================== */

void SpeedLoop_Init(void)
{
  s_tick  = 0U;
  s_fault = FAULT_NONE;

  s_tel.target_rpm   = 0.0f;
  s_tel.actual_rpm   = 0.0f;
  s_tel.out_permille = 0.0f;
  s_tel.kp = SPEED_LOOP_KP;
  s_tel.ki = SPEED_LOOP_KI;
  s_tel.kd = SPEED_LOOP_KD;
  s_tel.tick = 0U;
  s_tel.feedback = (SPEED_LOOP_FEEDBACK != 0) ? 1U : 0U;

  /* 执行器：PWM 句柄 = TIM1（PA8 = TIM1_CH1，通道由 CMake 的 -D 指定）*/
  if (MOTOR_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

#if SPEED_LOOP_FEEDBACK
  /* 反馈：编码器句柄 = TIM3（PA6/PA7）*/
  if (MOTOR_ENC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
#endif

  PID_Init(&s_pid,
           SPEED_LOOP_KP, SPEED_LOOP_KI, SPEED_LOOP_KD,
           SPEED_LOOP_OUT_MAX, SPEED_LOOP_INTEG_MAX);

#if SPEED_LOOP_SIM_PLANT
  s_sim_rpm = 0.0f;
#endif
}

void SpeedLoop_Start(void)
{
  /* CubeMX 只把外设配好，计数器/PWM 不会自己跑 —— 这里才启动 */
  (void)HAL_TIM_PWM_Start(&htim1, MOTOR_PWM_TIM_CHANNEL);

#if SPEED_LOOP_FEEDBACK
  (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
#endif

  (void)HAL_TIM_Base_Start_IT(&htim2);   /* 5 ms 心跳，从此每拍进 Tick */
}

/* ==========================================================================
 * 5 ms 中断
 * ========================================================================== */

void SpeedLoop_Tick(void)
{
  float target;
  float meas = 0.0f;
  float pid_out = 0.0f;
  float duty;
  int16_t out;

  s_tick++;

  /* ---- 1) 目标：自动阶跃开着就本地生成，否则跟串口给的 ---- */
  if (CMD_GetAutoStep() != 0U)
  {
    CMD_SetTargetRpm((int32_t)autostep_value(s_tick));
  }
  target = (float)CMD_GetTargetRpm();

  /* ---- 2) 实测转速 ---- */
#if SPEED_LOOP_SIM_PLANT
  meas = s_sim_rpm;                      /* 虚拟被控对象的上一次状态 */
#elif SPEED_LOOP_FEEDBACK
  meas = MOTOR_ENC_UpdateSpeed(5U);      /* dt = 5 ms，和本中断同频 */
#endif

  /* ---- 3) PID 修正（开环时整段不需要）---- */
#if SPEED_LOOP_FEEDBACK || SPEED_LOOP_SIM_PLANT
  if (s_fault == FAULT_NONE)
  {
    pid_out = PID_Update(&s_pid, target, meas, SPEED_LOOP_DT_S);

    /* 方向自检：给了正向目标、转速却持续为负 → 编码器 A/B 接反，PID 会变正反馈 */
    if ((target > 50.0f) && (meas < -5.0f))
    {
      s_fault = FAULT_ENC_DIR;
    }
  }
#endif

  /* ---- 4) 前馈 + PID → 限幅 → 输出 ---- */
  duty = (SPEED_LOOP_KV * target) + pid_out;
  duty = clampf(duty, SPEED_LOOP_OUT_MAX);

  if (s_fault != FAULT_NONE)
  {
    duty = 0.0f;
    MOTOR_EmergencyStop();
  }

  out = (int16_t)duty;
  (void)MOTOR_SetDuty(out);

#if SPEED_LOOP_SIM_PLANT
  /* 用"真正写出去的占空比"推进模型，下一拍读到的新转速才有意义 */
  s_sim_rpm += (((SIM_K_RPM_PER_PERMILLE * (float)out) - s_sim_rpm)
                * (SPEED_LOOP_DT_S / SIM_T_S));
#endif

#if SPEED_LOOP_FEEDBACK
  /* ---- 保护（只在闭环时有意义）---- */
  if (s_fault == FAULT_NONE)
  {
    static uint16_t nopulse_cnt;
    static uint16_t stall_cnt;

    /* 无脉冲：要它转却不转 → 编码器没信号/线松 */
    if ((target > 50.0f) && (meas < 3.0f) && (meas > -3.0f))
    {
      if (++nopulse_cnt >= (SPEED_LOOP_NOPULSE_MS / 5)) { s_fault = FAULT_NO_PULSE; }
    }
    else { nopulse_cnt = 0U; }

    /* 堵转：输出顶到很大、转速还上不去 */
    if ((out > 800) && (meas < 10.0f) && (meas > -10.0f))
    {
      if (++stall_cnt >= (SPEED_LOOP_STALL_MS / 5)) { s_fault = FAULT_STALL; }
    }
    else { stall_cnt = 0U; }
  }
#endif

  /* ---- 遥测（只写结构体，主循环自己去读）---- */
  s_tel.target_rpm   = target;
  s_tel.actual_rpm   = meas;
  s_tel.out_permille = (float)out;
  s_tel.tick         = s_tick;
}

/* ==========================================================================
 * 对外查询
 * ========================================================================== */

void SpeedLoop_SetTarget(float rpm)
{
  CMD_SetTargetRpm((int32_t)rpm);
}

void SpeedLoop_GetTelemetry(SpeedLoop_Telemetry_t *out)
{
  if (out == NULL) { return; }

  /* 5 ms 中断可能随时改这些字段；这里只做一次结构体拷贝（几个字，够快） */
  __disable_irq();
  *out = s_tel;
  __enable_irq();
}

const char *SpeedLoop_Fault(void)
{
  uint8_t f = s_fault;
  return (f == FAULT_NONE) ? NULL : s_fault_name[f];
}
