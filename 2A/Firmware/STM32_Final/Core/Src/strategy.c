/*
 * strategy.c
 *
 *  Created on: May 7, 2024
 *      Author: nicolas
 */


#include "strategy.h"
#include "stepper.h"
#include "main.h"
#include "usart.h"

__STRATEGY_HandleTypeDef strategies[7];
__STRATEGY_HandleTypeDef actual_strategy;

uint8_t id_strategy = 0;
extern uint8_t is_paused;

void STRATEGY_Init(void){
	// STRATEGY 0
	strategies[0].id = NO_Strat;
	strategies[0].name[0] = 'N';
	strategies[0].name[1] = '0';
	strategies[0].action_nb = 0;
	strategies[0].action[0].start_Time = 0;
	strategies[0].action[0].speed_Motor_Left = 0;
	strategies[0].action[0].speed_Motor_Right = 0;
	strategies[0].action[0].enable_stop_lidar = 1;

	// STRATEGY BLUE 1
	strategies[1].id = B1_Strat;
	strategies[1].name[0] = 'B';
	strategies[1].name[1] = '1';
	strategies[1].action_id = 0;
	strategies[1].action_nb = 4;
	strategies[1].action[0].start_Time = 0;
	strategies[1].action[0].speed_Motor_Left = 0;
	strategies[1].action[0].speed_Motor_Right = 0;
	strategies[1].action[0].enable_stop_lidar = 1;

	strategies[1].action[1].start_Time = 50;
	strategies[1].action[1].speed_Motor_Left = 10;
	strategies[1].action[1].speed_Motor_Right = 10;
	strategies[1].action[1].enable_stop_lidar = 1;

	strategies[1].action[2].start_Time = 100;
	strategies[1].action[2].speed_Motor_Left = 0;
	strategies[1].action[2].speed_Motor_Right = 10;
	strategies[1].action[2].enable_stop_lidar = 1;

	strategies[1].action[3].start_Time = 150;
	strategies[1].action[3].speed_Motor_Left = 0;
	strategies[1].action[3].speed_Motor_Right = 0;
	strategies[1].action[3].enable_stop_lidar = 1;

	// STRATEGY BLUE 2
	strategies[2].id = B2_Strat;
	strategies[2].name[0] = 'B';
	strategies[2].name[1] = '2';
	strategies[2].action_id = 0;
	strategies[2].action_nb = 1;
	strategies[2].action[0].start_Time = 0;
	strategies[2].action[0].speed_Motor_Left = 0;
	strategies[2].action[0].speed_Motor_Right = 0;
	strategies[2].action[0].enable_stop_lidar = 1;

	strategies[3].id = B3_Strat;
	strategies[3].name[0] = 'B';
	strategies[3].name[1] = '3';
	strategies[3].action_id = 0;
	strategies[3].action_nb = 1;
	strategies[3].action[0].start_Time = 0;
	strategies[3].action[0].speed_Motor_Left = 0;
	strategies[3].action[0].speed_Motor_Right = 0;
	strategies[3].action[0].enable_stop_lidar = 1;

	strategies[4].id = J1_Strat;
	strategies[4].name[0] = 'J';
	strategies[4].name[1] = '1';
	strategies[4].action_id = 0;
	strategies[4].action_nb = 1;
	strategies[4].action[0].start_Time = 0;
	strategies[4].action[0].speed_Motor_Left = 0;
	strategies[4].action[0].speed_Motor_Right = 0;
	strategies[4].action[0].enable_stop_lidar = 1;

	strategies[5].id = J2_Strat;
	strategies[5].name[0] = 'J';
	strategies[5].name[1] = '2';
	strategies[5].action_id = 0;
	strategies[5].action_nb = 1;
	strategies[5].action[0].start_Time = 0;
	strategies[5].action[0].speed_Motor_Left = 0;
	strategies[5].action[0].speed_Motor_Right = 0;
	strategies[5].action[0].enable_stop_lidar = 1;

	strategies[6].id = J3_Strat;
	strategies[6].name[0] = 'J';
	strategies[6].name[1] = '3';
	strategies[6].action_id = 0;
	strategies[6].action_nb = 2;
	strategies[6].action[0].start_Time = 0;
	strategies[6].action[0].speed_Motor_Left = 0;
	strategies[6].action[0].speed_Motor_Right = 0;
	strategies[6].action[0].enable_stop_lidar = 1;

	actual_strategy = strategies[0];
}

void STRATEGY_Update(){
	id_strategy = (id_strategy + 1)%7;
	actual_strategy = strategies[id_strategy];
}


// FAIRE MACHINE A ETAT
// FAIRE UN ENUM + SWITCH CASE POUR LA STATE MACHINE
void STRATEGY_Refresh(uint32_t time, uint8_t object_detected, uint8_t is_started){
	if(is_started){
		uint8_t next_id_action = actual_strategy.action_id + 1;
		if((actual_strategy.action[next_id_action].start_Time > time) && (next_id_action < actual_strategy.action_nb)){
			actual_strategy.action_id++;
			if((object_detected == 1) && (actual_strategy.action[actual_strategy.action_id].enable_stop_lidar == 1)){
				HAL_UART_Transmit(&huart2, "stop\r\n", 6, 10);
				Stepper_Stop(&hstepperLeft);
				Stepper_Stop(&hstepperRight);
				is_paused = 1;
			}
			else{
				HAL_UART_Transmit(&huart2, "speed\r\n", 7, 10);
				Stepper_Set_Speed(&hstepperLeft, actual_strategy.action[actual_strategy.action_id].speed_Motor_Left, SET);
				Stepper_Set_Speed(&hstepperRight, actual_strategy.action[actual_strategy.action_id].speed_Motor_Right, SET);
				is_paused = 0;
			}
		}
	}
}
