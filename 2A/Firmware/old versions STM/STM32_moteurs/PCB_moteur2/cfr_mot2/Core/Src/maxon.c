/*
 * maxon.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Antle
 */


#include "maxon.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;

void mot_maxonD(int result, int inv) {
    if (inv == 1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);    // Direction avant
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, result);  // Vitesse
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);  // Direction arrière
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, result);  // Vitesse
    }
}

void mot_maxonG(int result, int inv) {
    if (inv == 1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);    // Direction avant
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, result);  // Vitesse
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);  // Direction arrière
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, result);  // Vitesse
    }
}
