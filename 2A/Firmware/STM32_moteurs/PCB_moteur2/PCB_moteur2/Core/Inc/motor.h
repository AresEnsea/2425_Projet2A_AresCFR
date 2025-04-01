/*
 * motor.h
 *
 *  Created on: Mar 20, 2025
 *      Author: nicolas
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

typedef struct {
	TIM_HandleTypeDef *htim_high;
	uint32_t channel_high;
	TIM_HandleTypeDef *htim_low;
	uint32_t channel_low;
	GPIO_TypeDef *motor_enable_port;
	uint16_t motor_enable_pin;
	float motor_speed;
	uint16_t motor_duty_cycle;
} Motor_HandleTypeDef;

extern Motor_HandleTypeDef hmotor;

HAL_StatusTypeDef Motor_Init(Motor_HandleTypeDef *hmotor, TIM_HandleTypeDef *htim_high, uint32_t channel_high, TIM_HandleTypeDef *htim_low, uint32_t channel_low, GPIO_TypeDef *motor_enable_port, uint16_t motor_enable_pin);
HAL_StatusTypeDef Motor_Start(Motor_HandleTypeDef *hmotor);
HAL_StatusTypeDef Motor_Stop(Motor_HandleTypeDef *hmotor);
HAL_StatusTypeDef Motor_Set_Speed(Motor_HandleTypeDef *hmotor, float motor_speed);

#endif /* INC_MOTOR_H_ */
