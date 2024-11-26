/*
 * maxon.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Antle
 */


#include "maxon.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;

// Code fonctionnel final pour contrôler le moteur
void mot_maxonD(int result){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // Moteur droit
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, result);
}

void mot_maxonG(int result){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // enable droit
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1); // enablegauche
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, result);
}

