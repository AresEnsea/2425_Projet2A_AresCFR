/*
 * servo.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Antle
 */

#include "servo.h"
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart3;
//void reach_goal_servo(char goal[12]) {
//    // Control first servo (CCR1) and third servo (CCR3)
//    if (goal[0] == '1') {
//        TIM1->CCR1 = 2000;
//        TIM1->CCR3 = 2000;
//    } else {
//        TIM1->CCR1 = 1000;
//        TIM1->CCR3 = 1000;
//    }
//
//    // Control second servo (CCR2)
//    if (goal[1] == '1') {
//        TIM1->CCR2 = 2000;
//    } else {
//        TIM1->CCR2 = 1000;
//    }
//
//    // Stepper motor control (using ASCII conversion for UART transmission)
//    if (goal[2] >= '0' && goal[2] <= '5') {
//        uint8_t data = goal[2] - '0'; // Convert char to integer (0-5)
//        HAL_UART_Transmit(&huart3, &data, 1, HAL_MAX_DELAY);
//    }
//}
