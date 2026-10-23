#include "main.h"
#define MOTOR_PWM_TIM_CHANNEL 1U
HAL_StatusTypeDef MOTOR_Init(TIM_HandleTypeDef *);
HAL_StatusTypeDef MOTOR_SetDuty(int16_t);
void MOTOR_EmergencyStop(void);
