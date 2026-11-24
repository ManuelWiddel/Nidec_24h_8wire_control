#include "motor.h"

/* ----------------------------------------------------------
 * Configuration
 * ---------------------------------------------------------- */




/* ----------------------------------------------------------
 * Private variables
 * ---------------------------------------------------------- */

static TIM_HandleTypeDef *motor_pwm_timer = NULL;


/* ----------------------------------------------------------
 * Private functions
 * ---------------------------------------------------------- */





/* ----------------------------------------------------------
 * Public functions
 * ---------------------------------------------------------- */


void Motor_Init(TIM_HandleTypeDef *motor_pwm_timer_) {

	motor_pwm_timer = motor_pwm_timer_;

}

void Motor_Set_Pwm(uint16_t DIR_Pin,
					GPIO_TypeDef * DIR_Port,
					int16_t Duty_cycle_ARR,
					uint8_t pwm_neg,
	                uint32_t Channel)
{
	//check if duty cycle is in range
	if((Duty_cycle_ARR >= -ARR_MAX) && (Duty_cycle_ARR <= ARR_MAX)) {

		//write DIR_Pin according to CW or CCW rotation
		if (Duty_cycle_ARR < 0) {
			HAL_GPIO_WritePin(DIR_Port, DIR_Pin, SET);
		}
		else {
			HAL_GPIO_WritePin(DIR_Port, DIR_Pin, RESET);
		}

		//take duty cycle per mil and make positive
		Duty_cycle_ARR = (Duty_cycle_ARR >= 0) ? Duty_cycle_ARR : -Duty_cycle_ARR;

		//if motor pwm logic is reversed (check with "pwm_neg", reverse map duty cycle value
		Duty_cycle_ARR = (pwm_neg == 1) ? (ARR_MAX - Duty_cycle_ARR) : Duty_cycle_ARR;

		// Update only a valid, enabled timer channel
		    switch (Channel)
		    {
		        case TIM_CHANNEL_1:
		            if (motor_pwm_timer->Instance->CCER & TIM_CCER_CC1E)
		            {
		                __HAL_TIM_SET_COMPARE(motor_pwm_timer,
		                                      TIM_CHANNEL_1,
		                                      Duty_cycle_ARR);
		            }
		            break;

		        case TIM_CHANNEL_2:
		            if (motor_pwm_timer->Instance->CCER & TIM_CCER_CC2E)
		            {
		                __HAL_TIM_SET_COMPARE(motor_pwm_timer,
		                                      TIM_CHANNEL_2,
		                                      Duty_cycle_ARR);
		            }
		            break;

		        case TIM_CHANNEL_3:
		            if (motor_pwm_timer->Instance->CCER & TIM_CCER_CC3E)
		            {
		                __HAL_TIM_SET_COMPARE(motor_pwm_timer,
		                                      TIM_CHANNEL_3,
		                                      Duty_cycle_ARR);
		            }
		            break;

		        case TIM_CHANNEL_4:
		            if (motor_pwm_timer->Instance->CCER & TIM_CCER_CC4E)
		            {
		                __HAL_TIM_SET_COMPARE(motor_pwm_timer,
		                                      TIM_CHANNEL_4,
		                                      Duty_cycle_ARR);
		            }
		            break;

		        default:
		            // Invalid channel: do nothing
		            break;
		    }
	}
}
