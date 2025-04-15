/*
 * MotorFunctions.c
 *
 *  Created on: Apr 9, 2025
 *      Author: capod
 */
#include "main.h"
#include "MotorFunctions.h"
#include <stdbool.h>


bool OL(){    // Le capteur le plus a gauche "Outer Left"
	return ADC_VAL[0]>threshold;   // Renvoie Vrai si c'est sur du blanc
}
bool IL(){					// Le deuxième capteur à partir du gauche
	return ADC_VAL[1]>threshold;
}
bool IR(){
	return ADC_VAL[2]>threshold;
}
bool OR(){
	return ADC_VAL[3]>threshold;
}

void Run(uint16_t speedL, uint16_t speedR){
	// Set direction
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 0);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 1);
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 1);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 0);
	// Set the speed
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, speedL);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4, speedR);
}

void TurnLeft(uint16_t speed){
	// Set direction
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 0);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 0);
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 1);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 0);
	// Set the speed
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, 30);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4, speed);
}

void TurnRight(uint16_t speed){
	// Set direction
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 0);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 1);
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 0);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 0);
	// Set the speed
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4, 30);
}

void Stop(){
	// Set direction
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 0);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, 0);
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, 0);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, 0);
	// Set the speed
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4, 0);
}



