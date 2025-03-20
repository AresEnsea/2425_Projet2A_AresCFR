/*
 * motor.c
 *
 *  Created on: Mar 20, 2025
 *      Author: nicolas
 */


#include "motor.h"


Motor_HandleTypeDef hmotor;

HAL_StatusTypeDef Motor_Init(Motor_HandleTypeDef *hmotor, TIM_HandleTypeDef *htim_high, uint32_t channel_high, TIM_HandleTypeDef *htim_low, uint32_t channel_low, GPIO_TypeDef *motor_enable_port, uint16_t motor_enable_pin){
	hmotor->htim_high = htim_high;
	hmotor->htim_low = htim_low;
	hmotor->channel_high = channel_high;
	hmotor->channel_low = channel_low;
	hmotor->motor_enable_port = motor_enable_port;
	hmotor->motor_enable_pin = motor_enable_pin;
	;
}
HAL_StatusTypeDef Motor_Start(Motor_HandleTypeDef *hmotor){
	HAL_GPIO_WritePin(hmotor->motor_enable_port, hmotor->motor_enable_pin, SET);
}

HAL_StatusTypeDef Motor_Stop(Motor_HandleTypeDef *hmotor){
	HAL_GPIO_WritePin(hmotor->motor_enable_port, hmotor->motor_enable_pin, RESET);
}

HAL_StatusTypeDef Motor_Set_Speed(Motor_HandleTypeDef *hmotor, float motor_speed){
	if(motor_speed > 24) motor_speed = 24;
	if(motor_speed < -24) motor_speed = -24;
	hmotor->motor_speed = motor_speed;
	hmotor->motor_duty_cycle = 1000/2*(motor_speed/24+1);

	__HAL_TIM_SET_COMPARE(hmotor->htim_high, hmotor->channel_high, hmotor->motor_duty_cycle);
	__HAL_TIM_SET_COMPARE(hmotor->htim_low, hmotor->channel_low, 1000-hmotor->motor_duty_cycle);

}

