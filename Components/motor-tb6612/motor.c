/**
  ******************************************************************************
  * @file    motor.c
  * @author  niko（需求） / pi coding agent（实现）
  * @version V1.2.0
  * @date    2026-10-19
  * @brief   TB6612 类「双 IN + PWM」H 桥直流有刷电机驱动（HAL + CubeMX）
  ******************************************************************************
  * @attention
  *
  * ── 这个文件只做 CubeMX 不管的事 ───────────────────────────────────────────
  *
  *   CubeMX 已经把引脚配成推挽输出、把 TIM2_CH3 配成 PWM 并算好 PSC/ARR。
  *   本文件不再调用 HAL_GPIO_Init()，也不改任何定时器参数，更不启动 PWM，只做：
  *
  *     1) 按方向写 AIN1 / AIN2 的电平
  *     2) 把千分比占空比换算成 CCR
  *
  *   启动 PWM 由应用负责（和 HAL 的 Init/Start 分层一致）：
  *
  *     MOTOR_Init(&htim2);
  *     HAL_TIM_PWM_Start(&htim2, MOTOR_PWM_TIM_CHANNEL);
  *
  * ── 占空比是怎么变成 CCR 的 ────────────────────────────────────────────────
  *
  * 定时器在 PWM1 模式、向上计数时：
  *
  *     CCR = 0        → 整个周期都是低电平 → 占空比 0%
  *     CCR = ARR/2    → 一半高一半低     → 占空比 50%
  *     CCR = ARR+1    → 整个周期都是高   → 占空比 100%（CCR > ARR 时输出常有效）
  *
  * 所以：  compare = |duty_permille| * (ARR + 1) / 1000
  * 其中 (ARR + 1) 在 MOTOR_Init() 里从定时器现场读出 —— 也就是说
  * **PSC/ARR 始终以 CubeMX 为准**，驱动不复制一份、也不会算错。
  *
  * ── 为什么换向要先清零 ────────────────────────────────────────────────────
  *
  * 电机是电感。正转时绕组里有电流，此时直接把 AIN1/AIN2 反过来，
  * H 桥等于把电源反接到一个仍然带着电流的电感上 → 瞬时电流可能达到额定值的
  * 好几倍。本驱动在符号变化时先写 CCR=0（输出低电平，电流经续流二极管衰减），
  * 再切方向位，最后才给新的占空比。真正的“死区时间”由 H 桥内部逻辑保证。
  *
  * ⚠️ 一个诚实的细节：HAL_TIM_PWM_ConfigChannel() 打开了 CCR 的影子寄存器
  *   （OCxPE），所以 `__HAL_TIM_SET_COMPARE()` 写进去的值要到**下一个更新事件**
  *   （计数器溢出）才生效，10 kHz 时最多是一个周期 100 us。也就是说“清零→换向”
  *   之间最多还有 100 us 的旧占空比残留。
  *   为什么这仍然够用：电机绕组的电气时间常数 L/R 通常是 1 ~ 5 ms，
  *   远大于 100 us，电流来不及按“反向全压”冲上去。
  *   如果以后换成大功率电机、或者供电很硬，别指望这 100 us 的软件处理 ——
  *   在应用层给 PID 输出加斜坡（每 10 ms 最多变多少占空比）才是正解。
  *
  ******************************************************************************
  */

#include "motor.h"

/* ==========================================================================
 * 0. 私有宏 / 私有变量 / 私有函数声明
 * ========================================================================== */

/** 让某个半桥“导通”的引脚电平（受 MOTOR_AIN_ACTIVE_HIGH 控制） */
#define MOTOR_AIN_ON_STATE      (MOTOR_AIN_ACTIVE_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET)

/** 让某个半桥“关断”的引脚电平 */
#define MOTOR_AIN_OFF_STATE     (MOTOR_AIN_ACTIVE_HIGH ? GPIO_PIN_RESET : GPIO_PIN_SET)

#if (MOTOR_USE_STBY != 0)
#define MOTOR_STBY_ON_STATE     (MOTOR_STBY_ACTIVE_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET)
#endif

/** PWM 定时器句柄（MOTOR_Init 时保存，后面所有操作都用它） */
static TIM_HandleTypeDef *s_htim_pwm = NULL;

/** PWM 一个周期的计数值 = ARR + 1（初始化时从定时器读出） */
static uint32_t s_pwm_period = 0U;

/** 当前设定的占空比，千分比，带符号 */
static int16_t s_duty_permille = 0;

/** 当前方向：+1 正转，-1 反转，0 停止 */
static int8_t s_direction = 0;

/** 是否已经初始化成功（未初始化时所有调速接口返回 HAL_ERROR） */
static uint8_t s_initialized = 0U;

static void MOTOR_WriteOutputs(GPIO_PinState ain1_state, GPIO_PinState ain2_state);
static void MOTOR_WriteDirection(int8_t direction);

/* ==========================================================================
 * 1. 底层：写方向引脚电平（只写电平，不配置引脚模式）
 * ========================================================================== */

/**
  * @brief  把 AIN1 / AIN2 写成指定状态
  * @param  ain1_state AIN1 的目标电平
  * @param  ain2_state AIN2 的目标电平
  * @note   引脚模式由 CubeMX 的 MX_GPIO_Init() 配置（推挽输出），
  *         本函数只调用 HAL_GPIO_WritePin() 改电平。
  * @note   先两个都写“关断”，再写目标状态：
  *         这样从 (1,0) 切到 (0,1) 时，中间不会出现 (1,1) 这个刹车组合。
  */
static void MOTOR_WriteOutputs(GPIO_PinState ain1_state, GPIO_PinState ain2_state)
{
  HAL_GPIO_WritePin(MOTOR_AIN1_GPIO_PORT, MOTOR_AIN1_GPIO_PIN, MOTOR_AIN_OFF_STATE);
  HAL_GPIO_WritePin(MOTOR_AIN2_GPIO_PORT, MOTOR_AIN2_GPIO_PIN, MOTOR_AIN_OFF_STATE);

  if (ain1_state == MOTOR_AIN_ON_STATE)
  {
    HAL_GPIO_WritePin(MOTOR_AIN1_GPIO_PORT, MOTOR_AIN1_GPIO_PIN, MOTOR_AIN_ON_STATE);
  }
  if (ain2_state == MOTOR_AIN_ON_STATE)
  {
    HAL_GPIO_WritePin(MOTOR_AIN2_GPIO_PORT, MOTOR_AIN2_GPIO_PIN, MOTOR_AIN_ON_STATE);
  }
}

/**
  * @brief  按方向写方向引脚
  * @param  direction +1 正转（AIN1 有效 / AIN2 无效）
  *                   -1 反转（AIN2 有效 / AIN1 无效）
  */
static void MOTOR_WriteDirection(int8_t direction)
{
  if (direction > 0)
  {
    MOTOR_WriteOutputs(MOTOR_AIN_ON_STATE, MOTOR_AIN_OFF_STATE);
  }
  else
  {
    MOTOR_WriteOutputs(MOTOR_AIN_OFF_STATE, MOTOR_AIN_ON_STATE);
  }
}

/* ==========================================================================
 * 2. 初始化
 * ========================================================================== */

/**
  * @brief  绑定 PWM 定时器：读 ARR、方向脚置停止、CCR 清零
  * @param  htim_pwm CubeMX 已初始化的 PWM 定时器句柄（例如 &htim2）
  * @retval HAL_OK    成功
  * @retval HAL_ERROR 参数为空 / ARR=0
  * @note   不启动 PWM。调用完请自己执行：
  *             HAL_TIM_PWM_Start(&htim2, MOTOR_PWM_TIM_CHANNEL);
  */
HAL_StatusTypeDef MOTOR_Init(TIM_HandleTypeDef *htim_pwm)
{
  uint32_t autoreload;

  /* ---- 参数检查：CubeMX 的 MX_TIMx_Init() 没跑时 Instance 为 NULL -------- */
  if ((htim_pwm == NULL) || (htim_pwm->Instance == NULL))
  {
    return HAL_ERROR;
  }

  /* ---- 1) 先把方向写成“停止”（引脚模式已由 CubeMX 配好）---------------- */
  MOTOR_WriteOutputs(MOTOR_AIN_OFF_STATE, MOTOR_AIN_OFF_STATE);

#if (MOTOR_USE_STBY != 0)
  /* STBY 使能（引脚模式同样由 CubeMX 配置） */
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_PORT, MOTOR_STBY_GPIO_PIN, MOTOR_STBY_ON_STATE);
#endif

  /* ---- 2) 读 ARR：占空比换算的依据（以 CubeMX 的参数为准）---------------- */
  autoreload = __HAL_TIM_GET_AUTORELOAD(htim_pwm);
  if (autoreload == 0U)
  {
    /* ARR = 0 时一个周期只有 1 个计数，没法定义占空比 */
    return HAL_ERROR;
  }
  s_pwm_period = autoreload + 1U;

  /* ---- 3) 先把比较值清零（这样应用启动 PWM 时不会突然满速）-------------- */
  __HAL_TIM_SET_COMPARE(htim_pwm, MOTOR_PWM_TIM_CHANNEL, 0U);

  /* ---- 注意：这里**不**调用 HAL_TIM_PWM_Start() --------------------------
   * PWM 的启动交给应用：
   *     HAL_TIM_PWM_Start(&htim2, MOTOR_PWM_TIM_CHANNEL);
   * 这样“何时开始输出”由应用控制，也符合 HAL 的 Init/Start 分层。
   * ---------------------------------------------------------------------- */

  /* ---- 4) 记录状态 ------------------------------------------------------- */
  s_htim_pwm      = htim_pwm;
  s_duty_permille = 0;
  s_direction     = 0;
  s_initialized   = 1U;

  return HAL_OK;
}

/* ==========================================================================
 * 3. 调速
 * ========================================================================== */

/**
  * @brief  设置带符号占空比（千分比）
  * @param  duty_permille -1000 ~ +1000
  * @retval HAL_OK / HAL_ERROR
  */
HAL_StatusTypeDef MOTOR_SetDuty(int16_t duty_permille)
{
  int8_t   direction;
  uint32_t magnitude;
  uint32_t compare;

  /* ---- 未初始化时不允许操作，避免往空句柄里写寄存器 --------------------- */
  if (s_initialized == 0U)
  {
    return HAL_ERROR;
  }

  /* ---- 限幅 ------------------------------------------------------------- */
  if (duty_permille > MOTOR_DUTY_PERMILLE_MAX)
  {
    duty_permille = MOTOR_DUTY_PERMILLE_MAX;
  }
  else if (duty_permille < -MOTOR_DUTY_PERMILLE_MAX)
  {
    duty_permille = -MOTOR_DUTY_PERMILLE_MAX;
  }

  /* ---- 0：按配置滑行或刹车 --------------------------------------------- */
  if (duty_permille == 0)
  {
#if (MOTOR_STOP_MODE_BRAKE != 0)
    return MOTOR_Brake();
#else
    return MOTOR_Coast();
#endif
  }

  direction = (duty_permille > 0) ? 1 : -1;
  magnitude = (uint32_t)((duty_permille > 0) ? duty_permille : -duty_permille);

  /* ---- 换向：先撤掉 PWM，再切方向位 ------------------------------------ */
  if (direction != s_direction)
  {
    __HAL_TIM_SET_COMPARE(s_htim_pwm, MOTOR_PWM_TIM_CHANNEL, 0U);
    MOTOR_WriteDirection(direction);
    s_direction = direction;
  }

  /* ---- 千分比 → CCR（四舍五入，避免 30% 变成 29%）---------------------- */
  compare = (magnitude * s_pwm_period + (MOTOR_DUTY_PERMILLE_MAX / 2U))
            / MOTOR_DUTY_PERMILLE_MAX;

  /* CCR > ARR 在 PWM1 模式下等于“整周期有效”，正好是 100% */
  if (compare > s_pwm_period)
  {
    compare = s_pwm_period;
  }

  __HAL_TIM_SET_COMPARE(s_htim_pwm, MOTOR_PWM_TIM_CHANNEL, compare);

  s_duty_permille = duty_permille;

  return HAL_OK;
}

/* ==========================================================================
 * 4. 停止
 * ========================================================================== */

/**
  * @brief  滑行停止（两个输入都无效）
  * @retval HAL_OK / HAL_ERROR
  */
HAL_StatusTypeDef MOTOR_Coast(void)
{
  if (s_initialized == 0U)
  {
    return HAL_ERROR;
  }

  __HAL_TIM_SET_COMPARE(s_htim_pwm, MOTOR_PWM_TIM_CHANNEL, 0U);
  MOTOR_WriteOutputs(MOTOR_AIN_OFF_STATE, MOTOR_AIN_OFF_STATE);

  s_duty_permille = 0;
  s_direction     = 0;

  return HAL_OK;
}

/**
  * @brief  刹车停止（两个输入都有效，绕组短接）
  * @retval HAL_OK / HAL_ERROR
  */
HAL_StatusTypeDef MOTOR_Brake(void)
{
  if (s_initialized == 0U)
  {
    return HAL_ERROR;
  }

  __HAL_TIM_SET_COMPARE(s_htim_pwm, MOTOR_PWM_TIM_CHANNEL, 0U);
  MOTOR_WriteOutputs(MOTOR_AIN_ON_STATE, MOTOR_AIN_ON_STATE);

  s_duty_permille = 0;
  s_direction     = 0;

  return HAL_OK;
}

/**
  * @brief  紧急停止：PWM=0，方向引脚全部无效
  * @note   只做寄存器 / GPIO 写操作，不调用 HAL 函数、不做参数检查，
  *         所以可以在中断、故障处理里调用。
  */
void MOTOR_EmergencyStop(void)
{
  if (s_initialized == 0U)
  {
    return;
  }

  __HAL_TIM_SET_COMPARE(s_htim_pwm, MOTOR_PWM_TIM_CHANNEL, 0U);
  HAL_GPIO_WritePin(MOTOR_AIN1_GPIO_PORT, MOTOR_AIN1_GPIO_PIN, MOTOR_AIN_OFF_STATE);
  HAL_GPIO_WritePin(MOTOR_AIN2_GPIO_PORT, MOTOR_AIN2_GPIO_PIN, MOTOR_AIN_OFF_STATE);

  s_duty_permille = 0;
  s_direction     = 0;
}

/**
  * @brief  读回当前占空比（千分比，带符号）
  * @retval -1000 ~ +1000
  */
int16_t MOTOR_GetDuty(void)
{
  return s_duty_permille;
}
