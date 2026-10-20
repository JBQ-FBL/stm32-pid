/**
  ******************************************************************************
  * @file    motor_encoder.h
  * @author  niko（需求） / pi coding agent（实现）
  * @version V1.2.0
  * @date    2026-10-19
  * @brief   增量式（正交）编码器接口：TIM Encoder Mode 计数读取 + 转速换算
  ******************************************************************************
  * @attention
  *
  * ── 和 CubeMX 的分工 ────────────────────────────────────────────────────────
  *
  *   CubeMX 负责：
  *     · 把定时器配成 Encoder Mode（Combined Channels = Encoder Mode）
  *     · A/B 引脚（CH1/CH2）、输入滤波（IC Filter）、上下拉
  *   本驱动负责：
  *     · 读 CNT、处理 16 位回绕、把计数差换算成 rpm
  *     · 校验编码器模式、算“一圈多少计数”、清零计数
  *
  *   应用负责【启动】（和 HAL 的 Init/Start 分层一致）：
  *     HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  *
  *   ⚠️ 驱动里没有 HAL_GPIO_Init()，也不改定时器模式 —— 配错了请改 .ioc。
  *
  * ── CubeMX 必须这样配 ──────────────────────────────────────────────────────
  *
  *   | 项目             | CubeMX 里在哪                        | 必须的值                     |
  *   |------------------|--------------------------------------|------------------------------|
  *   | 编码器模式       | Timers → TIMx → Combined Channels    | Encoder Mode                 |
  *   | 倍频             | Timers → TIMx → Encoder Mode         | Encoder Mode TI1 and TI2（4×）|
  *   | 输入滤波         | Timers → TIMx → IC1/IC2 Filter       | 0 起步，有毛刺再加 6~10      |
  *   | A/B 引脚上下拉   | System Core → GPIO → PA8/PA9         | 开漏编码器选 Pull-up         |
  *
  *   本工程：PA8 = TIM1_CH1，PA9 = TIM1_CH2。
  *
  *   ⚠️ PA9 同时是 USART1_TX 的默认引脚。以后接 USB-TTL 给 VOFA+ 看曲线时，
  *      编码器要换到 TIM3（PA6/PA7）或 TIM4（PB6/PB7）。
  *      换定时器只需改 CubeMX + MOTOR_ENC_Init(&htimx)，驱动代码不用动。
  *
  * ── 初始化顺序（必须）───────────────────────────────────────────────────────
  *
  *   MX_TIM1_Init() → 【USER CODE 2】
  *        MOTOR_ENC_Init(&htim1);                            // 校验 + 算 CPR + 清零
  *        HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);    // ← 由应用启动
  *
  *   CubeMX 只把模式配好，计数器不会自己走；本驱动也不会调用 Start。
  *
  * ── 设计上的两个关键点 ─────────────────────────────────────────────────────
  *
  *   1) 计数差用 int16_t 相减：16 位计数器从 65535 回绕到 0（或反向）时，
  *      有符号减法会自动得到正确的小差值。写成 uint16_t 会在过零点读出一个
  *      ±65535 的假速度 —— PID 一看到这个数就会飞车。
  *      前提：计数器是 16 位（STM32F1 的 TIM1~TIM4 都是）。移植到 32 位定时器
  *      的系列（如 F4 的 TIM2/TIM5）时，要把 int16_t 换成 int32_t。
  *
  *   2) 倍频系数（2× / 4×）在初始化时从 TIMx->SMCR 现场判断，不靠宏：
  *        Encoder Mode TI1        → 2×
  *        Encoder Mode TI2        → 2×
  *        Encoder Mode TI1 and TI2 → 4×
  *      所以你在 CubeMX 里改成四倍频后，速度不会悄悄差 2 倍。
  *
  * ── 用法（放在 10 ms 定时器中断里，和 PID 同频）────────────────────────────
  *
  *   if (MOTOR_ENC_Init(&htim1) != HAL_OK) { Error_Handler(); }
  *
  *   // 每 10 ms 调用一次：
  *   float rpm = MOTOR_ENC_UpdateSpeed(10);
  *
  * ── 单位换算 ───────────────────────────────────────────────────────────────
  *
  *   一圈的计数（输出轴）= PPR × 倍频 × 减速比
  *   转速 rpm = 计数差 / 一圈计数 × (60000 / 采样周期ms)
  *
  *   PPR 是【电机轴】（减速前）的每转脉冲数，看电机型号；
  *   直驱电机把 MOTOR_ENC_GEAR_RATIO 设成 1.0f。
  *
  ******************************************************************************
  */
#ifndef __MOTOR_ENCODER_H
#define __MOTOR_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==========================================================================
 * 配置区：只放“机械参数”，硬件配置全部在 CubeMX 里
 * ========================================================================== */

/** 编码器每转脉冲数（PPR），是【电机轴】上的值（减速之前），看电机型号 */
#ifndef MOTOR_ENC_PPR
#define MOTOR_ENC_PPR               11U   /* 工程侧可用 -DMOTOR_ENC_PPR=13 覆盖 */
#endif

/** 减速比：输出轴转 1 圈 = 电机轴转 N 圈。直驱电机填 1.0f */
#ifndef MOTOR_ENC_GEAR_RATIO
#define MOTOR_ENC_GEAR_RATIO        1.0f  /* 工程侧可用 -DMOTOR_ENC_GEAR_RATIO=30.0f 覆盖 */
#endif

/** 计数器方向取反
  *   0 = 原样（电机正转时计数增加）
  *   1 = 取反（电机正转时计数减少）
  * 判据：给 MOTOR_SetDuty(正数)，看 MOTOR_ENC_GetSpeedRpm() 是不是正数。
  * 反了的话 PID 会变成正反馈 → 飞车，必须改这里（或交换 A/B 两根线）。
  */
#ifndef MOTOR_ENC_DIR_INVERT
#define MOTOR_ENC_DIR_INVERT        0
#endif

/** 测速一阶低通滤波系数（0.05 ~ 1.0）
  *   1.0 = 不滤波（默认，行为最直观）
  *   0.2 ~ 0.5 = 平滑，但会引入一点滞后；D 项噪声大时再考虑用它
  */
#ifndef MOTOR_ENC_SPEED_LPF_ALPHA
#define MOTOR_ENC_SPEED_LPF_ALPHA   1.0f
#endif

/* 换工程不要改上面 4 个宏的默认值（否则工程副本与库不一致，drv doctor 会报）。
   在工程的 CMakeLists.txt 里覆盖，驱动文件保持与库逐字节一致：
     target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
         MOTOR_ENC_GEAR_RATIO=30.0f MOTOR_ENC_SPEED_LPF_ALPHA=0.5f) */

/* ==========================================================================
 * 编译期参数检查
 * ========================================================================== */
#if (MOTOR_ENC_PPR == 0U)
#error "MOTOR_ENC_PPR 必须大于 0"
#endif

#if (MOTOR_ENC_DIR_INVERT != 0) && (MOTOR_ENC_DIR_INVERT != 1)
#error "MOTOR_ENC_DIR_INVERT 只能是 0 或 1"
#endif

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/**
  * @brief  绑定编码器定时器：校验模式、算 CPR、清零计数
  * @param  htim_encoder 已经由 CubeMX 配成 Encoder Mode 的定时器句柄，例如 &htim1
  * @retval HAL_OK    成功
  * @retval HAL_ERROR 参数为空 / 该定时器不是 Encoder Mode
  * @note   **本函数不启动编码器。** 调用完还要自己执行：
  *             HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  *         这样“什么时候开始计数”由应用控制。
  * @note   返回 HAL_ERROR 时先去看 CubeMX：Combined Channels 要选 Encoder Mode。
  * @note   必须在 MX_TIM1_Init() 之后调用（USER CODE 2）。
  */
HAL_StatusTypeDef MOTOR_ENC_Init(TIM_HandleTypeDef *htim_encoder);

/**
  * @brief  清零：计数器、累计计数、上次速度全部归零
  */
void MOTOR_ENC_Reset(void);

/**
  * @brief  读出“上次调用至今”的计数差（有符号，16 位回绕安全）
  * @retval 正 = 正转方向，负 = 反转方向
  * @note   每次调用都会推进内部“上次计数”，所以同一个计数只会被返回一次。
  *         ⚠️ 用它调试时，不要和 MOTOR_ENC_UpdateSpeed() 混着调 ——
  *         后者内部也会读一次差，你会把它的计数“偷走”，导致测速周期变短。
  */
int16_t MOTOR_ENC_ReadDelta(void);

/**
  * @brief  读累计计数（int32，已处理 16 位回绕，可用来算总行程）
  * @retval 自上次 MOTOR_ENC_Reset() 以来的累计计数（带方向）
  */
int32_t MOTOR_ENC_GetTotalCount(void);

/**
  * @brief  按固定采样周期更新转速，并返回结果
  * @param  dt_ms 距离上次调用经过的毫秒数（速度环里就是控制周期，例如 10）
  * @retval 转速，单位 rpm（转/分），带符号
  * @note   **必须用固定周期调用**（定时器中断里最省事），否则 rpm 换算不准。
  * @note   调用前请确保编码器已经 Start（否则计数不动，rpm 一直是 0）。
  * @note   dt_ms 越大，量化噪声越小但响应越慢；10 ms 是速度环的常用值。
  */
float MOTOR_ENC_UpdateSpeed(uint32_t dt_ms);

/**
  * @brief  读上次 MOTOR_ENC_UpdateSpeed() 算出的转速（不重新采样）
  * @retval rpm，带符号
  */
float MOTOR_ENC_GetSpeedRpm(void);

/**
  * @brief  读“输出轴转一圈对应多少个计数”（PPR × 倍频 × 减速比）
  * @retval 计数/圈；未初始化时返回 0
  * @note   用来把 rpm 换算成 m/s：v = rpm/60 × 轮周长(m)。
  */
float MOTOR_ENC_GetCountsPerRev(void);

/**
  * @brief  读初始化时自动识别出的倍频系数
  * @retval 2 或 4（TI1 / TI2 = 2，TI1 and TI2 = 4）
  */
uint8_t MOTOR_ENC_GetEdgesPerCycle(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_ENCODER_H */
