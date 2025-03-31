/*
 * servo.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Antle
 */
#include "servo.h"
extern TIM_HandleTypeDef htim1;

void reach_goal_servo(char goal[3]){
    // Control first servo (CCR1) and 3
    if(goal[0] == '1') {
        TIM1->CCR1 = 2000; // Set to 2000 if character is '1'
        TIM1->CCR3 = 2000;
    } else {
        TIM1->CCR1 = 1000; // Set to 1000 if character is '0'
        TIM1->CCR3 = 1000;
    }

    // Control second servo (CCR2)
    if(goal[1] == '1') {
        TIM1->CCR2 = 2000;
    } else {
        TIM1->CCR2 = 1000;
    }

    if(goal[2] == '1') {

     } else {

     }



}
