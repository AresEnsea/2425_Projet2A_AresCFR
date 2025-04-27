/*
 * servo.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Antle
 */

#include "servo.h"
extern TIM_HandleTypeDef htim1;

void reach_goal_servo(char goal[3]) {
    // Control first servo (CCR1) and third servo (CCR3)
    if (goal[0] == '1') {
        TIM1->CCR1 = 2000;
        TIM1->CCR3 = 2000;
    } else {
        TIM1->CCR1 = 1000;
        TIM1->CCR3 = 1000;
    }

    // Control second servo (CCR2)
    if (goal[1] == '1') {
        TIM1->CCR2 = 2000;
    } else {
        TIM1->CCR2 = 1000;
    }

    // Stepper motor control (corrected syntax and logic)
    if (goal[2] == '0') {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    } else if (goal[2] == '1') {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    } else if (goal[2] == '2') {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    }
}
