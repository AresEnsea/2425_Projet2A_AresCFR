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
#include <string.h>
#include <stdio.h>


void mot_maxonD(int result, int inv);
void mot_maxonG(int result, int inv);

#endif /* INC_MAXON_H_ */
