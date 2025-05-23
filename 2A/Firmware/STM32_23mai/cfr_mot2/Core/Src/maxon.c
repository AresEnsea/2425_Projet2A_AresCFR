/*
* maxon.c
*
* Created on: Nov 19, 2024
* Author: Antle
*/

#include "maxon.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;


void mot_maxon_both(int result_D, int inv_D, int result_G, int inv_G) {
    // Appliquer la direction immédiatement
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, inv_D);      // Droit A
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, !inv_D);     // Droit B
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, inv_G);      // Gauche A
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, !inv_G);     // Gauche B

    // Appliquer immédiatement la PWM (tout ou rien)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, result_D);  // Moteur Droit
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, result_G);  // Moteur Gauche

}
