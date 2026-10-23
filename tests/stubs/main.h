#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { void *Instance; } TIM_HandleTypeDef;
extern uint32_t test_primask;
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1U; }
static inline void __set_PRIMASK(uint32_t value) { test_primask = value; }
#define TIM2 ((void *)2)
#define TIM_CHANNEL_ALL 0xFFFFU
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *);
void Error_Handler(void);
#endif
