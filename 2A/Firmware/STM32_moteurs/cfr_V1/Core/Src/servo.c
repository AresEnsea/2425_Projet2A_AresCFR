/*
 * servo.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Antle
 */
#include "servo.h"
extern TIM_HandleTypeDef htim1;

void reach_goal_servo(int goal){
	if (goal==1){
		TIM1->CCR1 = 5;
	}
	else{
		TIM1->CCR1 = 20;
	}

}


