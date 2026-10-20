/**
  ******************************************************************************
  * @file    speed-loop.h
  * @brief   速度环调度：5 ms 一拍，把「目标 → 占空比」这条链串起来
  ******************************************************************************
  * @attention
  *
  * ── 三个开关（决定这一版练什么）────────────────────────────────────────────
  *
  *   SPEED_LOOP_FEEDBACK   0 = 开环（只有前馈：duty = Kv × target，不读编码器）
  *                         1 = 闭环（编码器测速 + PID 修正）
  *   SPEED_LOOP_SIM_PLANT  1 = 用一阶惯性模型冒充电机（没有硬件也能出曲线）
  *
  *   现在（电机/编码器未到）：FEEDBACK=0、SIM=0 → 串口给多少转，就按比例给占空比
  *   编码器到了：             FEEDBACK=1
  *   想在没有硬件时调 PID：   SIM=1（FEEDBACK 无所谓）
  *
  * ── 目标值从哪来 ───────────────────────────────────────────────────────────
  *
  *   **唯一来源是 App/cmd**（串口命令 S200 / S-100 / X）。
  *   自动阶跃也通过 CMD_SetTargetRpm() 写回去，所以永远只有一个目标。
  *
  * ── 单位 ───────────────────────────────────────────────────────────────────
  *
  *   目标/实际 = rpm（float）；输出 = 千分比 ‰（-1000~1000，符号 = 方向）。
  *
  ******************************************************************************
  */
#ifndef __SPEED_LOOP_H
#define __SPEED_LOOP_H

#include <stdint.h>

/* ==========================================================================
 * 开关
 * ========================================================================== */

/** 0 = 开环（只有前馈）；1 = 编码器闭环 */
#ifndef SPEED_LOOP_FEEDBACK
#define SPEED_LOOP_FEEDBACK      0
#endif

/** 1 = 一阶惯性模型冒充电机（无硬件调试用） */
#ifndef SPEED_LOOP_SIM_PLANT
#define SPEED_LOOP_SIM_PLANT     0
#endif

/* ==========================================================================
 * 调参旋钮（只改这一段，别在 .c 里写魔数）
 * ========================================================================== */

/** 前馈系数 ‰/rpm：稳态时 1 rpm 需要多少占空比。
  * 初值按"满量程 900‰ 对应 333 rpm"估，**接真电机后必须开环实测再改**（见 AGENTS §10）。*/
#define SPEED_LOOP_KV            2.70f

#define SPEED_LOOP_KP            3.0f      /* 位置式 PI 起步：Kd 先 0 */
#define SPEED_LOOP_KI            30.0f
#define SPEED_LOOP_KD            0.0f

#define SPEED_LOOP_OUT_MAX       900.0f    /* 输出限幅 ±900‰（留 10% 余量）*/
#define SPEED_LOOP_INTEG_MAX     300.0f    /* 积分限幅（抗饱和）*/
#define SPEED_LOOP_DT_S          0.005f    /* 控制周期 5 ms */

/* 自动阶跃：上电 2 s → STEP_RPM 保持 3 s → 回 0 → 循环（共 10 s）
 * 目的是让每条曲线在时间轴上对齐，叠图就能看出"哪次更快"。*/
#define SPEED_LOOP_STEP_RPM      150
#define SPEED_LOOP_STEP_DELAY    400       /* 拍数，400×5 ms = 2 s */
#define SPEED_LOOP_STEP_HOLD     600       /* 600×5 ms = 3 s */
#define SPEED_LOOP_STEP_PERIOD   2000      /* 2000×5 ms = 10 s */

/* 保护参数（只有闭环时生效）*/
#define SPEED_LOOP_NOPULSE_MS    200       /* 无脉冲：持续多久算异常 */
#define SPEED_LOOP_STALL_MS      500       /* 堵转：持续多久算异常 */

/* ==========================================================================
 * 遥测：中断里写，主循环只读（拷贝一份走）
 * ========================================================================== */

typedef struct
{
  float    target_rpm;      /* 目标 */
  float    actual_rpm;      /* 实测（开环时恒 0）*/
  float    out_permille;    /* 本拍实际写出去的占空比（含前馈 + PID）*/
  float    kp, ki, kd;      /* 参数快照：OLED / 答辩能直接报 */
  uint32_t tick;            /* 5 ms 计数（每秒 +200，可当"心跳"判据）*/
  uint8_t  feedback;        /* 1 = 闭环，0 = 开环 */
} SpeedLoop_Telemetry_t;

/* ==========================================================================
 * 接口
 * ========================================================================== */

/** 初始化：MOTOR_Init / PID_Init /（可选）MOTOR_ENC_Init。在 MX_*_Init() 之后调（USER CODE 2）*/
void SpeedLoop_Init(void);

/** 启动：PWM 起振 +（可选）编码器启动 + TIM2 的 5 ms 中断。CubeMX 只配硬件，不管启动 */
void SpeedLoop_Start(void);

/** 5 ms 中断里唯一要调的函数（HAL_TIM_PeriodElapsedCallback 里）*/
void SpeedLoop_Tick(void);

/** 设目标（内部转调 CMD_SetTargetRpm，保证目标只有一个来源）*/
void SpeedLoop_SetTarget(float rpm);

/** 取遥测快照（短临界区拷贝）。主循环用它刷 OLED / 发 VOFA */
void SpeedLoop_GetTelemetry(SpeedLoop_Telemetry_t *out);

/** 故障原因，NULL = 正常。触发后输出已被强制拉 0 */
const char *SpeedLoop_Fault(void);

#endif /* __SPEED_LOOP_H */
