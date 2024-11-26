/*
 * servo.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Antle
 */
#include "servo.h"
extern TIM_HandleTypeDef htim1;

void reach_goal_servo(int goal){
	TIM1->CCR1 = goal;

}


