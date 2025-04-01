/*
 * servo.c
 *
 *  Created on: Nov 15, 2024
 *      Author: Antle
 */
#include "servo.h"
extern TIM_HandleTypeDef htim1;

void reach_goal_servo(char goal[]){
	int claws_order = atoi(goal[0]-'0'); //    intValue = (int)(charValue - '0');  // Convert the char digit to an integer
	int plank_order = atoi(goal[1]-'0');
	if (claws_order == 1){
		TIM1 -> CCR1 = 1000;
	}
	if (claws_order == 0){
		TIM1 -> CCR1 = 2000;
	}
	if (plank_order == 1){
		TIM1 -> CCR2 = 1000;
	}
	if (plank_order == 0){
		TIM1 -> CCR2 = 2000;
	}




}


