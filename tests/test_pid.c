/**
  ******************************************************************************
  * @file    test_pid.c
  * @brief   pid.c 的主机端单测（不需要单片机、不需要 HAL）
  ******************************************************************************
  * 怎么跑（在本仓库根目录下）：
  *
  *   gcc -std=c11 -Wall -Wextra -O2 -IApp/pid tests/test_pid.c App/pid/pid.c -lm \
  *       -o /tmp/test_pid && /tmp/test_pid
  *
  * 为什么要有它：控制器是纯数学，**没有硬件也能证明它是对的**。
  * 面试被问"你怎么知道代码没问题"，这份文件和它的输出就是答案。
  ******************************************************************************
  */

#include <stdio.h>
#include <math.h>
#include "pid.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, fmt, ...)                                         \
  do {                                                                \
    if (cond) { g_pass++; printf("  \xE2\x9C\x93 " fmt "\n", ##__VA_ARGS__); } \
    else      { g_fail++; printf("  \xE2\x9C\x97 " fmt "   <-- 失败\n", ##__VA_ARGS__); } \
  } while (0)

#define NEAR(a, b, tol) (fabsf((a) - (b)) <= (tol))

/* ---------------------------------------------------------------- 1. 纯比例 */
static void test_proportional(void)
{
  PID_t p;
  float out;

  printf("[1] 纯比例：输出 = Kp*e，且被限幅\n");
  PID_Init(&p, 10.0f, 0.0f, 0.0f, 900.0f, 800.0f);

  out = PID_Update(&p, 50.0f, 0.0f, 0.005f);        /* e=50 → 500‰ */
  CHECK(NEAR(out, 500.0f, 0.01f), "e=50, Kp=10 → 500‰（实际 %.2f）", out);

  out = PID_Update(&p, 200.0f, 0.0f, 0.005f);       /* e=200 → 2000‰ → 限到 900 */
  CHECK(NEAR(out, 900.0f, 0.01f), "e=200 → 限幅到 900‰（实际 %.2f）", out);
  CHECK(p.saturated == 1U, "saturated 标志应为 1");

  out = PID_Update(&p, 0.0f, 50.0f, 0.005f);        /* e=-50 → -500‰ */
  CHECK(NEAR(out, -500.0f, 0.01f), "e=-50 → -500‰（方向符号正确，实际 %.2f）", out);
}

/* ---------------------------------------------------------------- 2. 纯积分 */
static void test_integral(void)
{
  PID_t p;
  float out;

  printf("[2] 纯积分：I 随 dt 累加，dt 翻倍则 I 翻倍\n");
  PID_Init(&p, 0.0f, 100.0f, 0.0f, 9000.0f, 1.0e9f);

  out = PID_Update(&p, 10.0f, 0.0f, 0.005f);        /* ∫=10*0.005=0.05 → 5‰ */
  CHECK(NEAR(out, 5.0f, 0.01f), "e=10, dt=5ms → 5‰（实际 %.3f）", out);

  PID_Reset(&p);
  out = PID_Update(&p, 10.0f, 0.0f, 0.010f);        /* dt 翻倍 → 10‰ */
  CHECK(NEAR(out, 10.0f, 0.01f), "同一误差 dt=10ms → 10‰（说明 dt 用对了，实际 %.3f）", out);
}

/* ------------------------------------------------- 3. 条件积分抗饱和（关键） */
static void test_anti_windup(void)
{
  PID_t p;
  int i;

  printf("[3] 条件积分抗饱和：饱和后积分必须冻结\n");
  PID_Init(&p, 1.0f, 100.0f, 0.0f, 100.0f, 1.0e9f);

  /* 误差 100，第一拍就顶到限幅；之后每拍误差还是 100（目标够不着） */
  for (i = 0; i < 1000; i++)
  {
    (void)PID_Update(&p, 100.0f, 0.0f, 0.005f);
  }

  /* 第一拍累加 100*0.005 = 0.5，之后就冻结在这 —— 绝对不会涨到 50 */
  CHECK(NEAR(p.integral, 0.5f, 0.001f),
        "1000 拍后积分仍停在 0.5（若没有抗饱和会累加到 ~50）实际 %.4f", p.integral);

  /* 反向误差时允许积分往回退（否则永远解除不了饱和） */
  (void)PID_Update(&p, 0.0f, 100.0f, 0.005f);       /* e = -100 → 顶部饱和时允许累加 */
  CHECK(p.integral < 0.5f, "反向误差能把积分往回拉（实际 %.4f）", p.integral);
}

/* ------------------------------------------------------- 4. 微分冲击（关键） */
static void test_no_derivative_kick(void)
{
  PID_t p;
  float out;

  printf("[4] 目标值跳变时 D 项不许打尖峰（对测量值微分）\n");
  PID_Init(&p, 2.0f, 0.0f, 500.0f, 9000.0f, 800.0f);
  p.d_alpha = 1.0f;

  (void)PID_Update(&p, 0.0f, 0.0f, 0.005f);         /* 静止起步 */
  out = PID_Update(&p, 100.0f, 0.0f, 0.005f);       /* 目标 0 → 100，测量没变 */

  CHECK(NEAR(out, 200.0f, 0.01f),
        "目标跳变 100 时输出只有 Kp*e=200（没有 D 尖峰，实际 %.1f）", out);
}

/* ---------------------------------------------------- 5. D 项方向与低通滤波 */
static void test_derivative_direction(void)
{
  PID_t p;
  float out = 0.0f;
  int i;

  printf("[5] D 项：转速在上涨时要**减小**输出\n");
  PID_Init(&p, 0.0f, 0.0f, 100.0f, 1.0e6f, 800.0f);
  p.d_alpha = 1.0f;

  for (i = 0; i < 10; i++)
  {
    float meas = 10.0f * (float)i * 0.005f;    /* 每秒涨 10 rpm */
    out = PID_Update(&p, 0.0f, meas, 0.005f);
  }

  CHECK(out < 0.0f, "转速上升 → D 项为负（提前刹车，实际 %.1f‰）", out);

  /* 低通真正的作用：①压制“突变尖尖”（噪声）②不改变稳态斜坡的幅值 */
  {
    PID_t q;
    float out_raw = 0.0f;
    float out_lp = 0.0f;

    PID_Init(&q, 0.0f, 0.0f, 100.0f, 1.0e6f, 800.0f);
    q.d_alpha = 1.0f;                            /* 不滤波 */
    (void)PID_Update(&q, 0.0f, 0.0f, 0.005f);
    out_raw = PID_Update(&q, 0.0f, 0.05f, 0.005f); /* 第一拍就出现 Δmeas=0.05 → raw_d=10 */

    PID_Init(&q, 0.0f, 0.0f, 100.0f, 1.0e6f, 800.0f);
    q.d_alpha = 0.2f;
    (void)PID_Update(&q, 0.0f, 0.0f, 0.005f);
    out_lp = PID_Update(&q, 0.0f, 0.05f, 0.005f);

    CHECK(fabsf(out_lp) < fabsf(out_raw) * 0.3f,
          "alpha=0.2 把突变尖尖压到 %.0f（不滤波是 %.0f）", fabsf(out_lp), fabsf(out_raw));
  }

  /* 持续斜坡时，滤波后最终会跟到同样的幅值（不引入稳态偏差）*/
  {
    PID_t q;
    int k;
    float out_lp = 0.0f;

    PID_Init(&q, 0.0f, 0.0f, 100.0f, 1.0e6f, 800.0f);
    q.d_alpha = 0.2f;
    for (k = 0; k < 200; k++)
    {
      out_lp = PID_Update(&q, 0.0f, 10.0f * (float)k * 0.005f, 0.005f);
    }
    CHECK(NEAR(fabsf(out_lp), 1000.0f, 30.0f),
          "持续斜坡最终跟到 ~1000（实际 %.0f），说明滤波没有偷掉增益", fabsf(out_lp));
  }
}

/* ------------------------------------------------------------- 6. 复位与保护 */
static void test_reset_and_guard(void)
{
  PID_t p;
  float out;

  printf("[6] PID_Reset 清状态；dt<=0 不把 D 项炸成 inf\n");
  PID_Init(&p, 5.0f, 10.0f, 0.0f, 900.0f, 800.0f);
  (void)PID_Update(&p, 100.0f, 0.0f, 0.005f);
  CHECK(p.integral > 0.0f, "积分已累加（%.3f）", p.integral);

  PID_Reset(&p);
  CHECK(NEAR(p.integral, 0.0f, 1e-6f) && NEAR(p.out, 0.0f, 1e-6f), "Reset 后积分/输出归零");

  out = PID_Update(&p, 100.0f, 0.0f, 0.0f);         /* dt = 0 */
  CHECK(isfinite(out), "dt=0 时返回有限值（%.1f），没有 NaN/inf", out);

  out = PID_Update(NULL, 100.0f, 0.0f, 0.005f);
  CHECK(NEAR(out, 0.0f, 1e-6f), "传 NULL 不崩、返回 0");
}

/* --------------------------------------- 7. 闭环：一阶惯性模型（预演指标） */
static void test_closed_loop(void)
{
  PID_t p;
  const float dt = 0.005f;
  const float T  = 0.080f;      /* 电机机械时间常数（估的，实测后替换）*/
  const float K  = 0.35f;       /* ‰ 占空比 → rpm 的静态增益 */
  const float target = 150.0f;
  float v = 0.0f;
  float out = 0.0f;
  float max_v = 0.0f;
  float rise_t = -1.0f;
  float sum_err = 0.0f;
  int   n_sum = 0;
  int   i;
  const int N = 2000;           /* 10 s */

  printf("[7] 闭环仿真（一阶惯性被控对象）：推一个“预期指标”\n");
  PID_Init(&p, 3.0f, 30.0f, 0.0f, 900.0f, 800.0f);

  for (i = 0; i < N; i++)
  {
    float t = (float)i * dt;

    out = PID_Update(&p, target, v, dt);
    v  += (K * out - v) * dt / T;            /* 一阶惯性：u 越大，稳态转速越高 */

    if (v > max_v) { max_v = v; }
    if ((rise_t < 0.0f) && (v >= 0.95f * target)) { rise_t = t; }
    if (t > 8.0f) { sum_err += fabsf(target - v); n_sum++; }
  }

  printf("      上升时间(到95%%) = %.0f ms | 超调 = %.1f%% | 稳态误差 = %.2f%%\n",
         rise_t * 1000.0f,
         (max_v - target) / target * 100.0f,
         (sum_err / (float)n_sum) / target * 100.0f);

  CHECK(rise_t >= 0.0f, "能在 10 s 内到达 95%%（%.0f ms）", rise_t * 1000.0f);
  CHECK((max_v - target) / target < 0.15f, "超调 < 15%%（实际 %.1f%%）",
        (max_v - target) / target * 100.0f);
  CHECK((sum_err / (float)n_sum) / target < 0.05f, "稳态误差 < 5%%（实际 %.2f%%）",
        (sum_err / (float)n_sum) / target * 100.0f);
  CHECK(fabsf(out) <= 900.0f + 1e-3f, "输出始终在 ±900‰ 内（实际 %.1f）", out);
}

int main(void)
{
  printf("=== pid.c 主机端单测 ===\n");
  test_proportional();
  test_integral();
  test_anti_windup();
  test_no_derivative_kick();
  test_derivative_direction();
  test_reset_and_guard();
  test_closed_loop();

  printf("\n=== 结果：%d 通过，%d 失败 ===\n", g_pass, g_fail);
  return (g_fail == 0) ? 0 : 1;
}
