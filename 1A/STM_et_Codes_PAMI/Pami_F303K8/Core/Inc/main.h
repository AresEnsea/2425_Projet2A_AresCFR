/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define S_L_Pin GPIO_PIN_0
#define S_L_GPIO_Port GPIOA
#define S_M_Pin GPIO_PIN_1
#define S_M_GPIO_Port GPIOA
#define S_R_Pin GPIO_PIN_2
#define S_R_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_3
#define IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_4
#define IN2_GPIO_Port GPIOA
#define IN3_Pin GPIO_PIN_5
#define IN3_GPIO_Port GPIOA
#define IN4_Pin GPIO_PIN_6
#define IN4_GPIO_Port GPIOA
#define Echo_Pin GPIO_PIN_0
#define Echo_GPIO_Port GPIOB
#define Echo_EXTI_IRQn EXTI0_IRQn
#define Trig_Pin GPIO_PIN_1
#define Trig_GPIO_Port GPIOB
#define Servo_Pin GPIO_PIN_8
#define Servo_GPIO_Port GPIOA
#define EN1_Pin GPIO_PIN_4
#define EN1_GPIO_Port GPIOB
#define EN2_Pin GPIO_PIN_5
#define EN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
