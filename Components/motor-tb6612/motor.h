/**
  ******************************************************************************
  * @file    motor.h
  * @author  niko（需求） / pi coding agent（实现）
  * @version V1.2.0
  * @date    2026-10-19
  * @brief   TB6612 类「双 IN + PWM」H 桥直流有刷电机驱动（HAL + CubeMX）
  ******************************************************************************
  * @attention
  *
  * ── 和 CubeMX 的分工（这个驱动为什么长这样）─────────────────────────────────
  *
  *   CubeMX（.ioc / MX_xxx_Init）负责【硬件配置】：
  *     · 引脚模式、上下拉、初始电平、User Label
  *     · 定时器的时钟源、PSC、ARR、PWM 通道模式
  *     · 外设时钟、NVIC
  *
  *   本驱动负责【运行逻辑】：
  *     · 把带符号的占空比换算成 CCR
  *     · 按方向写 AIN1 / AIN2 的电平
  *     · 读方向引脚、写寄存器，不碰任何配置
  *
  *   应用负责【启动 / 停止】（和 HAL 的习惯一致：Init 只配置，Start 由应用调）：
  *     HAL_TIM_PWM_Start(&htim2, MOTOR_PWM_TIM_CHANNEL);
  *
  *   ⚠️ 驱动里**没有 HAL_GPIO_Init()**，也不会改 PSC/ARR/引脚模式，
  *      更不会替你调用 HAL_TIM_PWM_Start()。
  *      配错了（比如把 PA4/PA5 配成开漏）驱动不会替你修 —— 请去改 .ioc。
  *      这样 `.ioc` 和运行时永远说的是同一套配置，不会出现“两套真相”。
  *
  * ── CubeMX 必须这样配（先对照检查，再写代码）────────────────────────────────
  *
  *   | 项目                 | CubeMX 里在哪                          | 必须的值                       |
  *   |----------------------|----------------------------------------|--------------------------------|
  *   | PA4 / PA5 引脚模式    | System Core → GPIO → 选中引脚          | Output Push Pull（不是 Open Drain）|
  *   | PA4 / PA5 初始电平    | 同上 → GPIO output level               | Low                            |
  *   | PA4 / PA5 上下拉      | 同上 → GPIO Pull-up/Pull-down          | No pull                        |
  *   | PA4 / PA5 User Label  | 同上 → User Label                      | AIN1 / AIN2（可选，推荐）      |
  *   | PWM 通道             | Timers → TIMx → Channelx               | PWM Generation CHx             |
  *   | PWM 频率             | Timers → TIMx → Prescaler / Period     | 10 kHz 左右（本工程 0 / 7199） |
  *
  * ── 初始化顺序（必须）───────────────────────────────────────────────────────
  *
  *   HAL_Init() → SystemClock_Config() → MX_GPIO_Init() → MX_TIM2_Init()
  *   → 【USER CODE 2】
  *        MOTOR_Init(&htim2);                                 // 绑定句柄 + 读 ARR
  *        HAL_TIM_PWM_Start(&htim2, MOTOR_PWM_TIM_CHANNEL);   // ← 由应用启动
  *
  *   为什么要分两步：HAL 的惯例是 Init 只做配置，Start 由应用决定时机；
  *   本驱动不调用 HAL_TIM_PWM_Start()，所以“什么时候真正输出 PWM”由你控制。
  *
  *   为什么不能放到 USER CODE 2 之前：
  *     · 那时 htim2.Instance 还是 NULL，MOTOR_Init() 会返回 HAL_ERROR；
  *     · MX_GPIO_Init() 之后还会按 .ioc 重新配置方向引脚，把启动前写的电平冲掉。
  *
  * ── 引脚号从哪来 ───────────────────────────────────────────────────────────
  *
  *   优先用 CubeMX 生成的 User Label：
  *
  *       CubeMX 里 PA4 的 User Label = AIN1
  *       → main.h 自动生成：
  *             #define AIN1_Pin        GPIO_PIN_4
  *             #define AIN1_GPIO_Port  GPIOA
  *       → 驱动直接使用这两个宏，你换引脚只改 CubeMX
  *
  *   没设 User Label 时，退回下面配置区的默认值（PA4 / PA5）。
  *
  * ── 用法 ────────────────────────────────────────────────────────────────────
  *
  *   if (MOTOR_Init(&htim2) != HAL_OK) { Error_Handler(); }
  *   HAL_TIM_PWM_Start(&htim2, MOTOR_PWM_TIM_CHANNEL);
  *
  *   MOTOR_SetDuty( 300);    // 正转，占空比 30.0%
  *   MOTOR_SetDuty(-1000);   // 反转，满速
  *   MOTOR_SetDuty(   0);    // 停止（默认滑行，见 MOTOR_STOP_MODE_BRAKE）
  *   MOTOR_Coast();          // 滑行
  *   MOTOR_Brake();          // 刹车
  *
  *   占空比单位是【千分比 permille】：-1000 ~ +1000，符号就是方向。
  *
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==========================================================================
 * 配置区：这里只声明“驱动要知道的接线信息”，不改任何硬件配置
 * ========================================================================== */

/* ---------- 方向引脚 ------------------------------------------------------
 * 优先使用 CubeMX 的 User Label（AIN1_Pin / AIN1_GPIO_Port）。
 * 没设 label 时用默认值 PA4 / PA5 —— 此时必须和 .ioc 里保持一致。
 * ------------------------------------------------------------------------ */
#if defined(AIN1_GPIO_Port) && defined(AIN1_Pin)
#define MOTOR_AIN1_GPIO_PORT        AIN1_GPIO_Port
#define MOTOR_AIN1_GPIO_PIN         AIN1_Pin
#else
#define MOTOR_AIN1_GPIO_PORT        GPIOA
#define MOTOR_AIN1_GPIO_PIN         GPIO_PIN_4
#endif

#if defined(AIN2_GPIO_Port) && defined(AIN2_Pin)
#define MOTOR_AIN2_GPIO_PORT        AIN2_GPIO_Port
#define MOTOR_AIN2_GPIO_PIN         AIN2_Pin
#else
#define MOTOR_AIN2_GPIO_PORT        GPIOA
#define MOTOR_AIN2_GPIO_PIN         GPIO_PIN_5
#endif

/* ---------- PWM 通道 ------------------------------------------------------
 * 只写通道号；定时器句柄由 MOTOR_Init(&htimPwm) 传进来。
 * 库默认值 = TIM-Motor 的接法（PA2 = TIM2_CH3）。
 * 换工程**不要改这里**（改了工程副本就和库不一致，drv doctor 会报），
 * 在工程的 CMakeLists.txt 里覆盖，驱动文件本身保持与库逐字节一致：
 *     target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
 *         MOTOR_PWM_TIM_CHANNEL=TIM_CHANNEL_1)   # 例如 PA8 = TIM1_CH1
 * ------------------------------------------------------------------------ */
#ifndef MOTOR_PWM_TIM_CHANNEL
#define MOTOR_PWM_TIM_CHANNEL       TIM_CHANNEL_3
#endif

/* ---------- 电平极性 / 停止方式 ------------------------------------------ */

/** 方向输入的“有效电平”
  *   1 = 高电平让对应半桥导通（TB6612 / L298N / DRV8871 双 IN 都是这样）
  *   0 = 低电平有效（反相逻辑板）
  * 工程侧可用 -DMOTOR_AIN_ACTIVE_HIGH=0 覆盖。
  */
#ifndef MOTOR_AIN_ACTIVE_HIGH
#define MOTOR_AIN_ACTIVE_HIGH       1
#endif

/** MOTOR_SetDuty(0) 的行为
  *   0 = 滑行 Coast（两个输入都无效，电机自由转）
  *   1 = 刹车 Brake（两个输入都有效，绕组短接）
  * 工程侧可用 -DMOTOR_STOP_MODE_BRAKE=1 覆盖。
  */
#ifndef MOTOR_STOP_MODE_BRAKE
#define MOTOR_STOP_MODE_BRAKE       0
#endif

/* ---------- STBY / EN（可选）---------------------------------------------
 * TB6612 模块的 STBY 必须为高，输出才使能（黡空 = 待机，实测踩过）。
  *   - STBY 直接焊到 VCC / 3V3：保持 0，不用管
  *   - STBY 接到某个 GPIO：设成 1，并在 CubeMX 里把那个脚配成 Output Push Pull，
  *     User Label 设成 STBY（驱动会自动用 STBY_Pin / STBY_GPIO_Port）
  * ⚠️ 设成 1 又没在 CubeMX 里设 Label 时，回退默认值是 **PA6** ——
  *    新工程 PA6 是 TIM3_CH1（编码器 A 相），驱动会把编码器脚当 GPIO 写电平，测速直接废。
  * ------------------------------------------------------------------------ */
#ifndef MOTOR_USE_STBY
#define MOTOR_USE_STBY              0
#endif

#if (MOTOR_USE_STBY != 0)
#if defined(STBY_GPIO_Port) && defined(STBY_Pin)
#define MOTOR_STBY_GPIO_PORT        STBY_GPIO_Port
#define MOTOR_STBY_GPIO_PIN         STBY_Pin
#else
#define MOTOR_STBY_GPIO_PORT        GPIOA
#define MOTOR_STBY_GPIO_PIN         GPIO_PIN_6
#endif
#define MOTOR_STBY_ACTIVE_HIGH      1
#endif /* MOTOR_USE_STBY */

/** 占空比千分比的上限：-1000 ~ +1000 对应 -100% ~ +100% */
#define MOTOR_DUTY_PERMILLE_MAX     1000

/* ==========================================================================
 * 编译期参数检查：配错了直接编译失败，比运行时“电机不转”好查
 * ========================================================================== */
#if (MOTOR_AIN_ACTIVE_HIGH != 0) && (MOTOR_AIN_ACTIVE_HIGH != 1)
#error "MOTOR_AIN_ACTIVE_HIGH 只能是 0 或 1"
#endif

#if (MOTOR_STOP_MODE_BRAKE != 0) && (MOTOR_STOP_MODE_BRAKE != 1)
#error "MOTOR_STOP_MODE_BRAKE 只能是 0 或 1"
#endif

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/**
  * @brief  绑定 PWM 定时器：读 ARR、把方向脚置为停止、把 CCR 清零
  * @param  htim_pwm CubeMX 已初始化好的 PWM 定时器句柄，例如 &htim2
  * @retval HAL_OK    成功
  * @retval HAL_ERROR 参数为空 / ARR=0
  * @note   必须在 MX_GPIO_Init() 和 MX_TIM2_Init() 之后调用（USER CODE 2）。
  * @note   **本函数不启动 PWM。** 调用完还要自己执行：
  *             HAL_TIM_PWM_Start(&htim2, MOTOR_PWM_TIM_CHANNEL);
  *         这样“什么时候开始输出”由应用掌握，和 HAL 的 Init/Start 分层一致。
  * @note   本函数不会初始化引脚，也不会改定时器参数 —— 那是 CubeMX 的职责。
  */
HAL_StatusTypeDef MOTOR_Init(TIM_HandleTypeDef *htim_pwm);

/**
  * @brief  设置占空比（带符号）
  * @param  duty_permille 千分比，范围 -1000 ~ +1000（超出自动限幅）
  *                       正 = 正转，负 = 反转，0 = 停止（滑行或刹车，见宏）
  * @retval HAL_OK    设置成功
  * @retval HAL_ERROR 还没调 MOTOR_Init()
  * @note   符号变化时会先清零占空比再切方向，见 motor.c 文件头说明。
  * @note   调用前请确保 PWM 已经 Start（否则只写 CCR，引脚没有波形）。
  * @note   这是“立刻生效”的接口。PID 输出直接喂进来即可；
  *         如果换向/起步电流太大，在应用层给输出加斜坡。
  */
HAL_StatusTypeDef MOTOR_SetDuty(int16_t duty_permille);

/**
  * @brief  滑行停止：PWM=0，AIN1=AIN2=无效 → 电机断电，靠惯性转完
  * @retval HAL_OK / HAL_ERROR（同 MOTOR_SetDuty）
  */
HAL_StatusTypeDef MOTOR_Coast(void);

/**
  * @brief  刹车停止：PWM=0，AIN1=AIN2=有效 → 绕组短接，动能变成电流消耗掉
  * @retval HAL_OK / HAL_ERROR（同 MOTOR_SetDuty）
  * @note   高速时刹车电流很大，电源和 H 桥要扛得住；平时停车用 Coast 更温和。
  */
HAL_StatusTypeDef MOTOR_Brake(void);

/**
  * @brief  紧急停止：只写寄存器，不调用任何阻塞函数，可在中断里调用
  * @note   与 MOTOR_Coast() 效果相同（滑行），但不检查初始化状态、
  *         不做 HAL 调用，因此在故障处理 / 中断里也能安全使用。
  */
void MOTOR_EmergencyStop(void);

/**
  * @brief  读回当前设定的占空比
  * @retval -1000 ~ +1000（调用 MOTOR_SetDuty 时被限幅后的值）
  */
int16_t MOTOR_GetDuty(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
