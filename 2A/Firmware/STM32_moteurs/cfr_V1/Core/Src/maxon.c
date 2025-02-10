/*
 * maxon.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Antle
 */

#include "maxon.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;

// Function to control the right motor (Motor_R)
void mot_maxonD(int result, int direction) {
    if (direction == 1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);   // Set B4 high
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // Set B8 low
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // Set B4 low
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // Set B8 high
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, result);
}

// Function to control the left motor (Motor_L)
void mot_maxonG(int result, int direction) {
    if (direction) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   // Set B9 high
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Set B5 low
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // Set B9 low
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // Set B5 high
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, result);
}
