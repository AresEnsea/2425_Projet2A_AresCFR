/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l5xx_hal.h"

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
#define X1_Pin GPIO_PIN_4
#define X1_GPIO_Port GPIOA
#define X2_Pin GPIO_PIN_5
#define X2_GPIO_Port GPIOA
#define X3_Pin GPIO_PIN_6
#define X3_GPIO_Port GPIOA
#define X4_Pin GPIO_PIN_7
#define X4_GPIO_Port GPIOA
#define Button_Pin GPIO_PIN_0
#define Button_GPIO_Port GPIOB
#define Button_EXTI_IRQn EXTI0_IRQn
#define TRIG_Pin GPIO_PIN_12
#define TRIG_GPIO_Port GPIOB
#define ECHO_Pin GPIO_PIN_13
#define ECHO_GPIO_Port GPIOB
#define ECHO_EXTI_IRQn EXTI13_IRQn
#define Servo2_Pin GPIO_PIN_8
#define Servo2_GPIO_Port GPIOA
#define Servo1_Pin GPIO_PIN_9
#define Servo1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_4
#define AIN2_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_5
#define AIN1_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOB
#define STBY_Pin GPIO_PIN_7
#define STBY_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_3
#define BIN1_GPIO_Port GPIOH
#define BIN2_Pin GPIO_PIN_8
#define BIN2_GPIO_Port GPIOB
#define PWMB_Pin GPIO_PIN_9
#define PWMB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
