/**
  ******************************************************************************
  * @file    speed-loop.h
  * @brief   速度环调度：5 ms 一拍，把「目标 → 占空比」这条链串起来
  ******************************************************************************
  * @attention
  *
  * ── 两种运行时模式（串口命令切）────────────────────────────────────────────
  *
  *   正常   闭环：编码器测速 + PID 修正 + 前馈
  *   'C'    编码器标定：PWM 强制 0，只累计计数（手转输出轴 10 圈验 1320）
  *
  * 无硬件时的「纯开环」和「一阶惯性虚拟被控对象」两个编译期开关已删除
  * （无硬件调试阶段用过，真机调参完成后没意义了）。
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

/* ==========================================================================
 * 调参旋钮（只改这一段，别在 .c 里写魔数）
 * ========================================================================== */

/** 前馈系数 ‰/rpm：稳态时 1 rpm 需要多少占空比。
  * **2026-10-21 实测重算**（串口探针，out 与稳态转速的比值）：
  *     300‰ → 72.7 rpm   → K = 0.242 rpm/‰ → Kv = 4.13
  *     210‰ → 48.5 rpm   → K = 0.231 rpm/‰ → Kv = 4.33
  * 取中值 4.2（原估值 2.70 是按“900‰ ↔ 333rpm”算的，偏小 35%，导致前馈不足→超调）*/
#define SPEED_LOOP_KV            4.15f       /* 2026-10-22 自动调参给出：3.75 → 4.15（前馈是提速主手段）*/

#define SPEED_LOOP_KP           14.0f       /* 2026-10-22 自动调参：10 → 14（再大就超调/振荡）*/
#define SPEED_LOOP_KI            5.0f
#define SPEED_LOOP_KD            0.0f

/* 方向已用 S50/S80 实测确认（act 为正且跟随 target），可以放开到 900‰。
 * 保留记录：首轮曾用 300‰，结果 S80 时输出顶格仍只能到 72.7 rpm（静差 9%）.*/
#define SPEED_LOOP_OUT_MAX      1000.0f
#define SPEED_LOOP_INTEG_MAX     40.0f     /* I 项上限 = Ki×40 = 200‰（输出上限的 20%）*/
#define SPEED_LOOP_DT_S          0.005f    /* 控制周期 5 ms */

/* ---- 编码器量化极限环抑制（2026-10-22 加，实测数据见 notes/tuning-log.md）----
 *
 * 现象：Kp=14、5 ms 窗口下 1 个编码器计数 = 9.09 rpm，一个计数的抖动 × Kp
 *       = 127‰ 输出抖动。稳态实测 out 在 460~720‰ 之间逐拍摆、act 在 145/155 跳，
 *       听得到嗡嗡声、VOFA 上是 10 rpm 粗的毛带。
 * 做法：对 **P 项** 加死区（积分照常看真实误差，否则会留静差）。
 *       `SPEED_LOOP_DEADBAND_RPM` = 5.0 rpm ≈ 0.55 个计数，正好覆盖一个计数的抖动。
 * 为什么不会拖慢上升：阶跃后 `SPEED_LOOP_DEADBAND_DELAY_MS` 内死区为 0（全速 P），
 *       实测上升时间 43 ms，远小于 300 ms，所以对 50 ms 指标没有影响。
 * 想要完全关闭：把 DEADBAND_RPM 设 0。*/
#define SPEED_LOOP_DEADBAND_RPM        5.0f
#define SPEED_LOOP_DEADBAND_DELAY_MS   300U

/* 自动阶跃：上电 2 s → STEP_RPM 保持 3 s → 回 0 → 循环（共 10 s）
 * 目的是让每条曲线在时间轴上对齐，叠图就能看出"哪次更快"。*/
#define SPEED_LOOP_STEP_RPM      150
#define SPEED_LOOP_STEP_DELAY    400       /* 拍数，400×5 ms = 2 s */
#define SPEED_LOOP_STEP_HOLD     600       /* 600×5 ms = 3 s */
#define SPEED_LOOP_STEP_PERIOD   2000      /* 2000×5 ms = 10 s */

/* 保护参数（只有闭环时生效）*/
#define SPEED_LOOP_NOPULSE_MS    200       /* 无脉冲：持续多久算异常 */
#define SPEED_LOOP_STALL_MS      500       /* 堵转：持续多久算异常 */
#define SPEED_LOOP_DIRBAD_MS     100       /* 方向自检：必须持续这么久才判反（防单拍噪声误停）*/

/* ==========================================================================
 * 遥测：中断里写，主循环只读（拷贝一份走）
 * ========================================================================== */

typedef struct
{
  float    target_rpm;      /* 目标 */
  float    actual_rpm;      /* 实测（编码器测的）*/
  float    out_permille;    /* 本拍实际写出去的占空比（含前馈 + PID）*/
  float    kp, ki, kd;
  float    kv;              /* 前馈系数快照（‰/rpm）：自动调参会改它，必须可见 */      /* 参数快照：OLED / 答辩能直接报 */
  uint32_t tick;            /* 5 ms 计数（每秒 +200，可当"心跳"判据）*/

  /* ---- 编码器标定（'C' 模式）---- */
  int32_t  enc_total;       /* 累计原始计数（4 倍频，未折算）*/
  float    enc_revs;        /* = enc_total / counts_per_rev，手转圈数 */
  uint8_t  calib;           /* 1 = 标定模式（PWM 强制 0）*/
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

/** 故障码（数字，给上位机 / 自动调参用）：0=NONE 1=ENC_DIR 2=NO_PULSE 3=STALL */
uint8_t SpeedLoop_FaultCode(void);

/** 清故障 + 清标定计数（相当于"复位控制器"，不用复位单片机）*/
void SpeedLoop_ClearFault(void);

/* ==========================================================================
 * 调参旋钮（全局变量：F5 调试器里可以直接改，下一拍生效，不用重烧）
 * ==========================================================================
 * 在 VS Code 的「运行和调试 → 变量 → Globals」里改，改完 OLED 的 KP/KI 会立刻变。
 * 调好后把数值写回 speed-loop.h 的宏，再烧一次才算真正固化。
 */
extern float g_pid_kp;
extern float g_pid_ki;
extern float g_pid_kd;
extern float g_pid_kv;

/** 进出标定模式：on!=0 时 PWM 恒 0，只累计编码器计数，清空之前的计数 */
void    SpeedLoop_SetCalib(uint8_t on);
uint8_t SpeedLoop_GetCalib(void);

#endif /* __SPEED_LOOP_H */
