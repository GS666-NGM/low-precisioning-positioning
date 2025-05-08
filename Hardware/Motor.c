#include "Motor.h"

void PWM_SetCpmpare1(uint16_t pwm)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
}

void PWM_SetCpmpare4(uint16_t pwm)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
}

void MotorL_Direction(uint8_t direction)
{
	if (direction == 1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (GPIO_PinState)0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (GPIO_PinState)1);
	}
	else if (direction == 2)
	{
		
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (GPIO_PinState)0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (GPIO_PinState)1);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (GPIO_PinState)0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (GPIO_PinState)0);
	}
}

void MotorR_Direction(uint8_t direction)
{
	if (direction == 1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (GPIO_PinState)0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (GPIO_PinState)1);
	}
	else if (direction == 2)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (GPIO_PinState)0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (GPIO_PinState)1);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (GPIO_PinState)0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (GPIO_PinState)0);
	}
}



void Motor_SetSpeed(int16_t pwml, int16_t pwmr)
{
    if(pwml>0)
        pwml *= 0.95;
    else if(pwml<0)
        pwml *= 1.04;
	if(pwml>=0)
	{
		MotorL_Direction(1);
		PWM_SetCpmpare4(pwml);
	}
	else
	{
		MotorL_Direction(2);
		PWM_SetCpmpare4(-pwml);
	}
	if(pwmr>=0)
	{
		MotorR_Direction(1);
		PWM_SetCpmpare1(pwmr);
	}
	else
	{
		MotorR_Direction(2);
		PWM_SetCpmpare1(-pwmr);
	}
}

