/*
 * MotorFunctions.h
 *
 *  Created on: Apr 9, 2025
 *      Author: capod
 */

#ifndef INC_MOTORFUNCTIONS_H_
#define INC_MOTORFUNCTIONS_H_
#include <stdbool.h>

extern TIM_HandleTypeDef htim4;
void Run(uint16_t speedL, uint16_t speedR);
void TurnLeft(uint16_t speed);
void TurnRight(uint16_t speed);
void Stop(void);

bool OL(void);
bool IL(void);
bool IR(void);
bool OR(void);

extern uint16_t ADC_VAL[4];
extern int threshold;
#endif /* INC_MOTORFUNCTIONS_H_ */
