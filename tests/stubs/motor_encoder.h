#include "main.h"
HAL_StatusTypeDef MOTOR_ENC_Init(TIM_HandleTypeDef *);
void MOTOR_ENC_Reset(void);
int16_t MOTOR_ENC_ReadDelta(void);
float MOTOR_ENC_UpdateSpeed(uint32_t);
float MOTOR_ENC_GetCountsPerRev(void);
