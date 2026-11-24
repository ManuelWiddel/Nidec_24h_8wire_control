#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

#define ARR_MAX 2500

void Motor_Init(TIM_HandleTypeDef *motor_pwm_timer);

void Motor_Enable(uint16_t EN_Pin,
		GPIO_TypeDef * EN_Port);

void Motor_Disable(uint16_t EN_Pin,
		GPIO_TypeDef * EN_Port);

void Motor_Set_Pwm(uint16_t DIR_Pin,
					GPIO_TypeDef * DIR_Port,
					int16_t Duty_cycle_ARR,
					uint8_t pwm_neg,
	                uint32_t Channel);

#endif /* MOTOR_H */
