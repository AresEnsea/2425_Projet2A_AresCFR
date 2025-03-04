/*
 * maxon.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Antle
 */

#ifndef INC_MAXON_H_
#define INC_MAXON_H_

#include "stm32l4xx_hal.h"
#include <stdlib.h>

void mot_maxonD(int result, int dir);
void mot_maxonG(int result, int dir);

#endif /* INC_MAXON_H_ */
