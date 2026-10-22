/**
  ******************************************************************************
  * @file    speed-loop.c
  * @brief   速度环：5 ms 一拍，读编码器 → PID → 前馈 → 限幅 → MOTOR_SetDuty
  ******************************************************************************
  * @attention
  *
  * **闭环：5 ms 一拍**
  *
  *     目标 ──►(+)──► PID ──►(+)──► 限幅 ──► 电机 ──► 编码器 ──┐
  *            ▲                                              │
  *            │            └──────── rpm ─────────────────────────┘
  *            │
  *            └── 前馈 Kv×target（不走 PID，直接加到输出上）
  *
  * 实测参数（2026-10-21，见 ../notes/tuning-log.md）：
  *     Kv=3.75‰/rpm（开环扫点标定）、Kp=10、Ki=5、Kd=0、限幅 1000‰
  *     上升 50.2ms / 静差 -0.1% / 扰动恢复 302ms
  *
  * 两个运行时模式（串口命令切，不影响编译）：
  *     正常     闭环控制
  *     'C'      编码器标定：PWM 强制 0，只累计脉冲（不跑 PID）
  *
  * 历史上还有「纯开环」和「一阶惯性虚拟被控对象」两个编译期开关，
  * 它们是无硬件时的调试手段；2026-10-21 真机调参完成后已删除（git 历史里还能找到）。
  *
  * ⚠️ 中断里只做四件事，禁止 printf / OLED / HAL_Delay。
  *
  ******************************************************************************
  */

#include "main.h"
#include "tim.h"        /* htim1 / htim2 / htim3 */
#include "motor.h"
#include "motor_encoder.h"   /* 编码器（TIM3）：测速 + 标定计数 */
#include "pid.h"
#include "cmd.h"
#include "speed-loop.h"

/* ==========================================================================
 * 私有状态
 * ========================================================================== */

static PID_t    s_pid;
static volatile SpeedLoop_Telemetry_t s_tel;
static volatile uint32_t s_tick;

/* ==========================================================================
 * 调参旋钮：全局变量（**故意不是常量**）
 * ==========================================================================
 * 为什么不用宏直接传：宏是编译期的，改一次要编译 + 烧录 ≈ 10 秒，
 * 一轮调参十几组就是十几分钟，而且实验中间喝口水回来就忘了刚才烧的是哪组。
 *
 * 用法（F5 调试，不用停程序）：
 *   1. VS Code F5 → 停在 main
 *   2. 左侧「运行和调试」→ 变量 → Globals → 找 g_pid_kp
 *   3. 直接改数值，回车 —— **下一拍（5 ms 后）就生效**
 *   4. OLED 第 4 行的 KP/KI 会立刻跟着变，不用怀疑到底有没有生效
 *
 * ⚠️ 注意：调试器改的是 RAM，**复位/重新上电就回到宏的初值**。
 *    调出满意的一组后，把数值写回 speed-loop.h 的宏里再烧一次，才算真的定下来。
 */
float g_pid_kp = SPEED_LOOP_KP;
float g_pid_ki = SPEED_LOOP_KI;
float g_pid_kd = SPEED_LOOP_KD;
float g_pid_kv = SPEED_LOOP_KV;   /* 前馈系数 ‰/rpm */

/* 故障：0 = 正常 */
enum { FAULT_NONE = 0, FAULT_ENC_DIR, FAULT_NO_PULSE, FAULT_STALL };
static volatile uint8_t s_fault;

static const char *s_fault_name[] = {
  NULL, "ENC_DIR", "NO_PULSE", "STALL"
};

/* ---- 标定模式（串口 'C'）----
 * on 时：不跑 PID、PWM 恒 0（coast，手能转动），只把编码器差值累加起来。
 * 用法：进标定 → 手转输出轴正好 10 圈 → 看 enc_total → 退出。 */
static volatile uint8_t s_calib;
static volatile int32_t s_enc_total;

/* ---- 三个保护计时器（拍数，非 ms）----
 * 提到文件级（而不是 Tick 内的 static）是为了让 ClearFault() 能把它们一起清掉：
 * 否则清完故障后计数还留着，下一拍立刻又触发，看上去“清不掉”。 */
static uint16_t s_nopulse_cnt;
static uint16_t s_stall_cnt;
static uint16_t s_dirbad_cnt;

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
  s_tick        = 0U;
  s_fault       = FAULT_NONE;
  s_calib       = 0U;
  s_enc_total   = 0;
  s_nopulse_cnt = 0U;
  s_stall_cnt   = 0U;
  s_dirbad_cnt  = 0U;

  s_tel.target_rpm   = 0.0f;
  s_tel.actual_rpm   = 0.0f;
  s_tel.out_permille = 0.0f;
  s_tel.kp = g_pid_kp;
  s_tel.ki = g_pid_ki;
  s_tel.kd = g_pid_kd;
  s_tel.tick = 0U;
  s_tel.enc_total = 0;
  s_tel.enc_revs  = 0.0f;
  s_tel.calib     = 0U;

  /* 执行器：PWM 句柄 = TIM1（PA8 = TIM1_CH1，通道由 CMake 的 -D 指定）*/
  if (MOTOR_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  /* 反馈：编码器句柄 = TIM3（PA6/PA7）*/
  if (MOTOR_ENC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  PID_Init(&s_pid,
           SPEED_LOOP_KP, SPEED_LOOP_KI, SPEED_LOOP_KD,
           SPEED_LOOP_OUT_MAX, SPEED_LOOP_INTEG_MAX);
}

void SpeedLoop_Start(void)
{
  /* CubeMX 只把外设配好，计数器/PWM 不会自己跑 —— 这里才启动 */
  (void)HAL_TIM_PWM_Start(&htim1, MOTOR_PWM_TIM_CHANNEL);

  (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

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

  /* 串口兜底：半截命令停太久就当打完（防止串口工具不给换行时整机"失联"）*/
  CMD_PollTimeout();

  /* ---- 标定模式：不控制，只数脉冲（PWM 恒 0，手能自由转输出轴）---- */
  if (s_calib != 0U)
  {
    int16_t d   = MOTOR_ENC_ReadDelta();
    float   cpr = MOTOR_ENC_GetCountsPerRev();

    s_enc_total += (int32_t)d;

    (void)MOTOR_SetDuty(0);          /* 强制滑行：避免电机绕组阻力干扰手转 */

    s_tel.target_rpm   = 0.0f;
    s_tel.actual_rpm   = 0.0f;
    s_tel.out_permille = 0.0f;
    s_tel.enc_total    = s_enc_total;
    s_tel.enc_revs     = (cpr > 0.0f) ? ((float)s_enc_total / cpr) : 0.0f;
    s_tel.calib        = 1U;
    s_tel.tick         = s_tick;
    return;
  }

  /* ---- 1) 目标：自动阶跃开着就本地生成，否则跟串口给的 ---- */
  if (CMD_GetAutoStep() != 0U)
  {
    CMD_SetTargetRpm((int32_t)autostep_value(s_tick));
  }
  target = (float)CMD_GetTargetRpm();

  /* ---- 2) 实测转速 ---- */
  meas = MOTOR_ENC_UpdateSpeed(5U);      /* dt = 5 ms，和本中断同频 */

  /* ---- 3) PID 修正 ---- */
  /* VOFA+ 滑块（串口 KP/KI/KD）：有新值就覆盖宏初值，下一拍生效。
   * 先拿当前值打底：TakeGains 只改被命令改过的那一个，不会把另两个刷成 0。*/
  {
    float kp_new = g_pid_kp;
    float ki_new = g_pid_ki;
    float kd_new = g_pid_kd;

    if (CMD_TakeGains(&kp_new, &ki_new, &kd_new) != 0U)
    {
      g_pid_kp = kp_new;
      g_pid_ki = ki_new;
      g_pid_kd = kd_new;
    }
  }

  /* 调试器里改了 g_* → 下一拍就生效（只在变了的时候写，开销可忽略）*/
  if ((s_pid.kp != g_pid_kp) || (s_pid.ki != g_pid_ki) || (s_pid.kd != g_pid_kd))
  {
    PID_SetGains(&s_pid, g_pid_kp, g_pid_ki, g_pid_kd);
  }

  if (s_fault == FAULT_NONE)
  {
    pid_out = PID_Update(&s_pid, target, meas, SPEED_LOOP_DT_S);

    /* 方向自检：给了正向目标、转速却持续为负 → 编码器 A/B 接反，PID 会变正反馈。
     * 必须持续 SPEED_LOOP_DIRBAD_MS 才判死：单拍的负值可能是量化噪声或回绕，
     * 一出现就急停会让"其实正常"的系统反复误停。*/
    if ((target > 50.0f) && (meas < -5.0f))
    {
      if (++s_dirbad_cnt >= (SPEED_LOOP_DIRBAD_MS / 5U)) { s_fault = FAULT_ENC_DIR; }
    }
    else { s_dirbad_cnt = 0U; }
  }

  /* ---- 4) 前馈 + PID → 限幅 → 输出 ---- */
  duty = (g_pid_kv * target) + pid_out;
  duty = clampf(duty, SPEED_LOOP_OUT_MAX);

  if (s_fault != FAULT_NONE)
  {
    duty = 0.0f;
    MOTOR_EmergencyStop();
  }

  out = (int16_t)duty;
  (void)MOTOR_SetDuty(out);

  /* ---- 保护 ---- */
  if (s_fault == FAULT_NONE)
  {
    /* 无脉冲：要它转却不转 → 编码器没信号/线松 */
    if ((target > 50.0f) && (meas < 3.0f) && (meas > -3.0f))
    {
      if (++s_nopulse_cnt >= (SPEED_LOOP_NOPULSE_MS / 5U)) { s_fault = FAULT_NO_PULSE; }
    }
    else { s_nopulse_cnt = 0U; }

    /* 堵转：输出顶到接近限幅、转速还上不去。
     * 阈值跟着 OUT_MAX 走（写死的 800 在首轮限幅 300‰ 时永远不会触发）。 */
    if (((float)out > (SPEED_LOOP_OUT_MAX * 0.85f)) && (meas < 10.0f) && (meas > -10.0f))
    {
      if (++s_stall_cnt >= (SPEED_LOOP_STALL_MS / 5U)) { s_fault = FAULT_STALL; }
    }
    else { s_stall_cnt = 0U; }
  }

  /* ---- 遥测（只写结构体，主循环自己去读）---- */
  s_tel.target_rpm   = target;
  s_tel.actual_rpm   = meas;
  s_tel.out_permille = (float)out;
  s_tel.tick         = s_tick;
  s_tel.enc_total    = s_enc_total;
  s_tel.enc_revs     = (MOTOR_ENC_GetCountsPerRev() > 0.0f)
                       ? ((float)s_enc_total / MOTOR_ENC_GetCountsPerRev()) : 0.0f;
  s_tel.calib        = s_calib;

  /* 参数快照：调试器里改了 g_* 以后，OLED / '?' 立刻就能看到新值 */
  s_tel.kp = g_pid_kp;
  s_tel.ki = g_pid_ki;
  s_tel.kd = g_pid_kd;
}

/* ==========================================================================
 * HAL 回调：5 ms 定时器到了 → 跑一拍
 * ==========================================================================
 * ⚠️ 这一步很容易漏：只调用 HAL_TIM_Base_Start_IT() 只是让中断"能进来"，
 *    真正把控制算一次的是这个回调。漏了它的症状是：
 *      编译通过、串口能收命令、但 tick 不涨、OUT 永远是 0、电机不转 ——
 *    更隐蔽的是链接器会把没人引用的 SpeedLoop_Tick() 直接丢掉
 *    （--gc-sections），所以 `nm` 里都看不到这个符号。
 *    判据：arm-none-eabi-nm build/Debug/Pid.elf | grep " T SpeedLoop_Tick"
 * ------------------------------------------------------------------------ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    SpeedLoop_Tick();
  }
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

void SpeedLoop_ClearFault(void)
{
  s_fault       = FAULT_NONE;
  s_nopulse_cnt = 0U;
  s_stall_cnt   = 0U;
  s_dirbad_cnt  = 0U;

  /* 积分清零：故障期间误差一直在累，恢复瞬间会把这个"欠账"一次性推出去 */
  PID_Reset(&s_pid);

  MOTOR_EmergencyStop();
}

void SpeedLoop_SetCalib(uint8_t on)
{
  s_calib     = (on != 0U) ? 1U : 0U;
  s_enc_total = 0;

  if (s_calib != 0U)
  {
    /* 进标定先彻底停机：目标置 0、积分清零、输出强制 0 */
    CMD_SetTargetRpm(0);
    PID_Reset(&s_pid);
    MOTOR_EmergencyStop();
  }

  s_tel.calib     = s_calib;
  s_tel.enc_total = 0;
  s_tel.enc_revs  = 0.0f;
}

uint8_t SpeedLoop_GetCalib(void)
{
  return s_calib;
}
