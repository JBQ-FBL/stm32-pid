/**
  ******************************************************************************
  * @file    pid.c
  * @author  niko（需求） / pi coding agent（实现）
  * @version V1.0.0
  * @date    2026-10-20
  * @brief   速度环 PID：位置式 + 条件积分抗饱和 + 对测量值微分
  ******************************************************************************
  * @attention
  *
  * 一拍的顺序（顺序本身就是设计，别随手调换）：
  *
  *   1) e = target − meas
  *   2) 积分：只有"没饱和"或"误差能把输出往回拉"时才累加，然后限幅
  *   3) 微分：Δmeas/dt，再做一阶低通
  *   4) 三项相加 → 输出限幅 → 记录
  *
  * 为什么先积分再算输出：条件积分要判断"上一拍的输出是否已经饱和"，
  * 用 p->out（上一拍保存的）而不是本拍还没算出来的值 —— 这是标准做法，
  * 也让这段代码可以在中断里安心跑（没有隐藏状态）。
  *
  ******************************************************************************
  */

#include "pid.h"

/* ==========================================================================
 * 小工具：不依赖 math.h（主机单测和单片机都省事，也不引入 libm）
 * ========================================================================== */

static float PID_Abs(float x)
{
  return (x < 0.0f) ? -x : x;
}

static float PID_Clamp(float x, float limit)
{
  if (limit <= 0.0f)
  {
    return x;                 /* 传 0/负数 = 不限幅 */
  }
  if (x >  limit) { return  limit; }
  if (x < -limit) { return -limit; }
  return x;
}

/* ==========================================================================
 * 初始化 / 复位
 * ========================================================================== */

void PID_Init(PID_t *p, float kp, float ki, float kd, float out_max, float integ_max)
{
  if (p == 0) { return; }

  p->kp        = kp;
  p->ki        = ki;
  p->kd        = kd;
  p->out_max   = out_max;
  p->integ_max = integ_max;
  p->d_alpha   = PID_DEFAULT_D_ALPHA;

  p->integral  = 0.0f;
  p->last_meas = 0.0f;
  p->d_filt    = 0.0f;
  p->out       = 0.0f;
  p->saturated = 0U;
}

void PID_SetGains(PID_t *p, float kp, float ki, float kd)
{
  if (p == 0) { return; }

  p->kp = kp;
  p->ki = ki;
  p->kd = kd;
}

void PID_Reset(PID_t *p)
{
  if (p == 0) { return; }

  p->integral  = 0.0f;
  p->last_meas = 0.0f;
  p->d_filt    = 0.0f;
  p->out       = 0.0f;
  p->saturated = 0U;
}

/* ==========================================================================
 * 一拍计算
 * ========================================================================== */

float PID_Update(PID_t *p, float target, float meas, float dt)
{
  float err;
  float p_term;
  float i_term;
  float d_term;
  float raw_d;
  float out;
  uint8_t was_saturated;

  /* ---- dt 保护：中断里万一算出 0 或负数，不许让 D 项变成 inf/NaN ---- */
  if ((p == 0) || (dt <= 0.0f))
  {
    return (p == 0) ? 0.0f : p->out;
  }

  err = target - meas;

  /* ---- 1) 条件积分 -------------------------------------------------------
   * 只在下面三种情况累加：
   *   a) 上一拍没饱和                      → 正常累加
   *   b) 上一拍顶在上限，但误差是负的      → 误差在把输出往回拉，允许累加
   *   c) 上一拍顶在下限，但误差是正的      → 同上
   * 其余情况（饱和且误差还在往同方向推）就冻结积分。
   * ---------------------------------------------------------------------- */
  was_saturated = p->saturated;

  if ((was_saturated == 0U) ||
      ((p->out >=  p->out_max) && (err < 0.0f)) ||
      ((p->out <= -p->out_max) && (err > 0.0f)))
  {
    p->integral += err * dt;

    /* 积分状态再限幅一道：即使误差长期同号，也不会把 float 堆到天文数字 */
    p->integral = PID_Clamp(p->integral, p->integ_max);
  }

  /* ---- 2) 对测量值微分 + 一阶低通 ---------------------------------------
   * d_alpha = 1.0  → d_filt 直接等于 raw_d（不滤波）
   * d_alpha = 0.3  → 平滑，但滞后约 (1-alpha)/alpha 拍
   * ---------------------------------------------------------------------- */
  raw_d = (meas - p->last_meas) / dt;
  p->d_filt += p->d_alpha * (raw_d - p->d_filt);

  /* ---- 3) 三项叠加 ------------------------------------------------------
   * D 项带负号：转速在上涨（raw_d > 0）时提前减小输出，抑制超调。
   * ---------------------------------------------------------------------- */
  p_term = p->kp * err;
  i_term = p->ki * p->integral;
  d_term = -p->kd * p->d_filt;

  out = p_term + i_term + d_term;

  /* ---- 4) 输出限幅 ------------------------------------------------------ */
  out = PID_Clamp(out, p->out_max);

  p->out       = out;
  p->saturated = (uint8_t)((p->out_max > 0.0f) &&
                           ((PID_Abs(out) >= p->out_max) ? 1 : 0));
  p->last_meas = meas;

  return out;
}
