/**
  ******************************************************************************
  * @file    motor_encoder.c
  * @author  niko（需求） / pi coding agent（实现）
  * @version V1.2.0
  * @date    2026-10-19
  * @brief   增量式（正交）编码器接口：TIM Encoder Mode 计数读取 + 转速换算
  ******************************************************************************
  * @attention
  *
  * 这个文件只做 CubeMX 不管的事：
  *   1) 校验编码器模式、算“一圈多少计数”、清零计数
  *   2) 读 CNT、处理 16 位回绕
  *   3) 把计数差换算成 rpm
  *
  * 编码器的启动由应用负责（和 HAL 的 Init/Start 分层一致）：
  *   MOTOR_ENC_Init(&htim1);
  *   HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  *
  * 引脚模式、编码器模式（TI1/TI2/TI12）、输入滤波、上下拉全部由 CubeMX 配置，
  * 本文件不调用 HAL_GPIO_Init()，也不改 TIMx->SMCR / CCMR。
  *
  * ── 16 位计数器回绕是怎么处理的 ────────────────────────────────────────────
  *
  * CNT 是 16 位的：65535 再加 1 就变成 0。如果直接
  *
  *     delta = (uint16_t)now - (uint16_t)last;
  *
  * 反向走一圈会得到一个巨大的数。正确写法是先把两边都当【有符号 16 位】看：
  *
  *     int16_t delta = (int16_t)((int16_t)now - (int16_t)last);
  *
  * 因为 int16 的减法在模 2^16 意义下与无符号一致，而结果又被解释成有符号数，
  * 所以正向回绕得到小的正数、反向回绕得到小的负数 —— 这正是我们要的。
  * 前提：一个采样周期内的计数变化不超过 ±32767（10 ms 内不可能超）。
  *
  ******************************************************************************
  */

#include "motor_encoder.h"

/* ==========================================================================
 * 0. 私有变量 / 私有函数声明
 * ========================================================================== */

/** 编码器定时器句柄（MOTOR_ENC_Init 时保存） */
static TIM_HandleTypeDef *s_htim_enc = NULL;

/** 上次读到的 16 位计数值 */
static int16_t s_last_counter = 0;

/** 累计计数（int32，已消除 16 位回绕） */
static int32_t s_total_count = 0;

/** 上次算出的转速（rpm，带符号） */
static float s_speed_rpm = 0.0f;

/** 输出轴转一圈对应的计数（PPR × 倍频 × 减速比） */
static float s_counts_per_rev = 0.0f;

/** 自动识别出的倍频系数：2 或 4 */
static uint8_t s_edges_per_cycle = 0U;

/** 是否已经初始化成功 */
static uint8_t s_initialized = 0U;

#if (MOTOR_ENC_MA_N > 1U)
/** 滑动平均：最近 N 个「计数差」（存原始计数，不存 rpm） */
static int16_t s_ma_buf[MOTOR_ENC_MA_N];
/** 环形写指针 */
static uint8_t s_ma_idx = 0U;
/** 窗口内已填的样本数（上电前 N-1 拍不能除以 N，否则读到偏低的转速） */
static uint8_t s_ma_fill = 0U;
/** 窗口内计数差之和（int32 防溢出：N=32、单拍 32767 也不超 2^31） */
static int32_t s_ma_sum = 0;
#endif

static uint8_t MOTOR_ENC_DetectEdgesPerCycle(TIM_HandleTypeDef *htim_encoder);

/* ==========================================================================
 * 1. 初始化
 * ========================================================================== */

/**
  * @brief  从定时器寄存器判断编码器模式是 2 倍频还是 4 倍频
  * @param  htim_encoder 编码器定时器句柄
  * @retval 2 或 4；不是编码器模式时返回 0
  * @note   SMCR.SMS 编码（见 RM0008「从模式控制寄存器」）：
  *           001 = Encoder mode 1（只数 TI1 的边沿，2 倍频）
  *           010 = Encoder mode 2（只数 TI2 的边沿，2 倍频）
  *           011 = Encoder mode 3（TI1、TI2 边沿都数，4 倍频）
  *         这里直接读寄存器而不是用宏，是因为 HAL 没有“读当前编码器模式”的接口。
  *         本函数只读，不改配置 —— 配置仍然是 CubeMX 的事。
  */
static uint8_t MOTOR_ENC_DetectEdgesPerCycle(TIM_HandleTypeDef *htim_encoder)
{
  uint32_t slave_mode = htim_encoder->Instance->SMCR & TIM_SMCR_SMS;

  switch (slave_mode)
  {
    case TIM_ENCODERMODE_TI1:
    case TIM_ENCODERMODE_TI2:
      return 2U;

    case TIM_ENCODERMODE_TI12:
      return 4U;

    default:
      return 0U;      /* 不是编码器模式 */
  }
}

/**
  * @brief  绑定编码器定时器：校验模式、算 CPR、清零计数
  * @param  htim_encoder CubeMX 已配成 Encoder Mode 的定时器句柄
  * @retval HAL_OK / HAL_ERROR
  * @note   不启动编码器。调用完请自己执行：
  *             HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  */
HAL_StatusTypeDef MOTOR_ENC_Init(TIM_HandleTypeDef *htim_encoder)
{
  uint8_t edges;

  /* ---- 参数检查 --------------------------------------------------------- */
  if ((htim_encoder == NULL) || (htim_encoder->Instance == NULL))
  {
    return HAL_ERROR;
  }

  /* ---- 1) 确认定时器真的在编码器模式（CubeMX 忘配时能立刻发现）---------- */
  edges = MOTOR_ENC_DetectEdgesPerCycle(htim_encoder);
  if (edges == 0U)
  {
    return HAL_ERROR;
  }

  /* ---- 2) 清零计数，清掉可能残留的更新标志 ------------------------------ */
  __HAL_TIM_SET_COUNTER(htim_encoder, 0U);
  __HAL_TIM_CLEAR_FLAG(htim_encoder, TIM_FLAG_UPDATE);

  /* ---- 3) 算“输出轴一圈多少计数” ---------------------------------------- */
  s_counts_per_rev = (float)MOTOR_ENC_PPR
                   * (float)edges
                   * MOTOR_ENC_GEAR_RATIO;

  if (s_counts_per_rev <= 0.0f)
  {
    return HAL_ERROR;
  }

  /* ---- 注意：这里**不**调用 HAL_TIM_Encoder_Start() ---------------------
   * 启动交给应用：
   *     HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
   * ---------------------------------------------------------------------- */

  /* ---- 4) 记录状态 ------------------------------------------------------- */

  s_htim_enc        = htim_encoder;
  s_last_counter    = 0;
  s_total_count     = 0;
  s_speed_rpm       = 0.0f;
  s_edges_per_cycle = edges;
  s_initialized     = 1U;

  return HAL_OK;
}

/**
  * @brief  清零计数器、累计计数和速度
  */
void MOTOR_ENC_Reset(void)
{
  if (s_initialized == 0U)
  {
    return;
  }

  __HAL_TIM_SET_COUNTER(s_htim_enc, 0U);

  s_last_counter = 0;
  s_total_count  = 0;
  s_speed_rpm    = 0.0f;

#if (MOTOR_ENC_MA_N > 1U)
  {
    uint8_t i;
    for (i = 0U; i < (uint8_t)MOTOR_ENC_MA_N; i++) { s_ma_buf[i] = 0; }
    s_ma_idx  = 0U;
    s_ma_fill = 0U;
    s_ma_sum  = 0;
  }
#endif
}

/* ==========================================================================
 * 2. 计数
 * ========================================================================== */

/**
  * @brief  读“上次调用至今”的计数差（有符号、回绕安全）
  * @retval 计数差，正负代表方向
  */
int16_t MOTOR_ENC_ReadDelta(void)
{
  int16_t now;
  int16_t delta;

  if (s_initialized == 0U)
  {
    return 0;
  }

  /* 直接把 16 位 CNT 当有符号数读；回绕处理见文件头说明 */
  now   = (int16_t)__HAL_TIM_GET_COUNTER(s_htim_enc);
  delta = (int16_t)(now - s_last_counter);

  s_last_counter = now;

#if (MOTOR_ENC_DIR_INVERT != 0)
  delta = (int16_t)(-delta);
#endif

  s_total_count += delta;

  return delta;
}

/**
  * @brief  读累计计数
  * @retval 自上次 Reset 以来的累计计数（带方向）
  */
int32_t MOTOR_ENC_GetTotalCount(void)
{
  return s_total_count;
}

/* ==========================================================================
 * 3. 转速
 * ========================================================================== */

/**
  * @brief  按固定采样周期更新转速
  * @param  dt_ms 采样周期（毫秒），例如 10
  * @retval 转速 rpm（带符号）
  * @note   公式：rpm = 计数差 / 一圈计数 × (60000 / dt_ms)
  *         —— 除以“一圈计数”得到圈数，乘 60000/dt_ms 把“每采样周期圈数”
  *            换算成“每分钟圈数”。
  */
float MOTOR_ENC_UpdateSpeed(uint32_t dt_ms)
{
  int16_t delta;
  float   rpm;

  if ((s_initialized == 0U) || (dt_ms == 0U) || (s_counts_per_rev <= 0.0f))
  {
    return 0.0f;
  }

  delta = MOTOR_ENC_ReadDelta();

#if (MOTOR_ENC_MA_N > 1U)
  /* ---- 滑动求和：把测速窗口从 dt 拉长到 N·dt，台阶降到 q/N ----
   * 除数是 n（窗口里实际已有的样本数）而不是 N：上电前 N-1 拍不会读到偏低值。*/
  s_ma_sum += (int32_t)delta - (int32_t)s_ma_buf[s_ma_idx];
  s_ma_buf[s_ma_idx] = delta;
  s_ma_idx = (uint8_t)((s_ma_idx + 1U) % MOTOR_ENC_MA_N);
  if (s_ma_fill < (uint8_t)MOTOR_ENC_MA_N) { s_ma_fill++; }

  rpm = (((float)s_ma_sum / (float)s_ma_fill) / s_counts_per_rev)
        * (60000.0f / (float)dt_ms);
#else
  rpm = ((float)delta / s_counts_per_rev) * (60000.0f / (float)dt_ms);
#endif

  /* 一阶低通：alpha=1 时就是 rpm 本身（不滤波） */
  s_speed_rpm += MOTOR_ENC_SPEED_LPF_ALPHA * (rpm - s_speed_rpm);

  return s_speed_rpm;
}

/**
  * @brief  读上次算出的转速（不重新采样）
  * @retval rpm，带符号
  */
float MOTOR_ENC_GetSpeedRpm(void)
{
  return s_speed_rpm;
}

/**
  * @brief  读“输出轴一圈对应多少计数”
  * @retval 计数/圈；未初始化时 0
  */
float MOTOR_ENC_GetCountsPerRev(void)
{
  return s_counts_per_rev;
}

/**
  * @brief  读自动识别出的倍频系数
  * @retval 2 或 4；未初始化时 0
  */
uint8_t MOTOR_ENC_GetEdgesPerCycle(void)
{
  return s_edges_per_cycle;
}
