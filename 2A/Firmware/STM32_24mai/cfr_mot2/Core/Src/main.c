/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Programme principal pour l'odométrie et le contrôle moteur
  *                  du robot utilisant un microcontrôleur STM32 avec encodeur AMT102-V.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "servo.h"
#include "maxon.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_COUNTS_PER_REV 4096
#define WHEEL_DIAMETER 0.0762f
#define WHEEL_BASE 0.240f
#define WHEEL_CIRCUMFERENCE 0.2394f
#define ENCODER_TOLERANCE 5  // Tolérance en comptes pour la position cible
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
char rx_data[27];
float x = 0.0f;
float y = 0.0f;
float theta = 0.0f;
int16_t prev_left = 0;
int16_t prev_right = 0;
uint32_t last_update_time = 0;

float K_p = 500.0f;  // Réduit pour éviter les oscillations
float K_i = 200.0f;  // Augmenté pour corriger l'erreur cumulée
float e_sum = 0.0f;
int v_L_base = 0;
int v_R_base = 0;
uint8_t inv_L = 0;
uint8_t inv_R = 0;
bool is_straight = false;
int s = 1;
const float right_motor_compensation = 0.95f;  // Réduit la vitesse du moteur droit de 5%

bool position_control_active = false;
bool control_locked = false;  // Verrouillage après atteinte des cibles
int32_t target_left = 0;
int32_t target_right = 0;
int32_t origin_left = 0;
int32_t origin_right = 0;
const int low_speed = 200;  // Réduit pour minimiser les dérives

float total_dist_left = 0.0f;  // Distance totale parcourue par la roue gauche (m)
float total_dist_right = 0.0f; // Distance totale parcourue par la roue droite (m)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_LPUART1_UART_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_UART5_Init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)rx_data, sizeof(rx_data));
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  prev_left = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  prev_right = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
  origin_left = prev_left;
  origin_right = prev_right;

  while (1)
  {
    int offset = -1;
    for (int j = 0; j < sizeof(rx_data); j++) {
      if (rx_data[j] == '>') {
        offset = j;
        break;
      }
    }

    if (offset >= 0) {
      char console_data[29];
      for (int i = 0; i < 27; i++) {
        console_data[i] = rx_data[(offset + i) % 27];
      }
      console_data[27] = '\r';
      console_data[28] = '\n';
      HAL_UART_Transmit(&huart2, (uint8_t *)console_data, 29, HAL_MAX_DELAY);

      char mot_maxon_dataD[5], mot_maxon_dataG[5];
      for (int i = 0; i < 4; i++) {
        mot_maxon_dataD[i] = rx_data[(offset + 2 + i) % 27];
      }
      mot_maxon_dataD[4] = '\0';
      int target_mm_D = atoi(mot_maxon_dataD);
      if (target_mm_D < 0) target_mm_D = 0;

      int inv_d = rx_data[(offset + 1) % 27] - '0';
      if (inv_d != 0 && inv_d != 1) inv_d = 0;

      for (int i = 0; i < 4; i++) {
        mot_maxon_dataG[i] = rx_data[(offset + 7 + i) % 27];
      }
      mot_maxon_dataG[4] = '\0';
      int target_mm_G = atoi(mot_maxon_dataG);
      if (target_mm_G < 0) target_mm_G = 0;

      int inv_g = rx_data[(offset + 6) % 27] - '0';
      if (inv_g != 0 && inv_g != 1) inv_g = 0;

      float distance_per_count = WHEEL_CIRCUMFERENCE / ENCODER_COUNTS_PER_REV;
      int32_t counts_D = (int32_t)((float)target_mm_D / 1000.0f / distance_per_count);
      int32_t counts_G = (int32_t)((float)target_mm_G / 1000.0f / distance_per_count);
      target_left = origin_left + (inv_g == 0 ? counts_G : -counts_G);
      target_right = origin_right + (inv_d == 0 ? counts_D : -counts_D);

      // Activer le contrôle uniquement si non verrouillé
      if (!control_locked) {
        position_control_active = (target_mm_D > 0 || target_mm_G > 0);
        v_L_base = position_control_active ? low_speed : 0;
        v_R_base = position_control_active ? (int)(low_speed * right_motor_compensation) : 0;
        inv_L = inv_g;
        inv_R = inv_d;
        is_straight = (target_mm_G == target_mm_D) && (inv_L == inv_R);
        if (is_straight) {
          s = (inv_L == 0) ? 1 : -1;
        } else {
          e_sum = 0.0f;
        }
        mot_maxon_both(v_R_base, inv_R, v_L_base, inv_L);
      }

      char servo_goal_data[13];
      servo_goal_data[0] = '<';
      for (int i = 0; i < 12; i++) {
        servo_goal_data[i + 1] = rx_data[(offset + 11 + i) % 27];
      }
      HAL_UART_Transmit(&huart4, (uint8_t *)servo_goal_data, 13, HAL_MAX_DELAY);

      char stepper_data1[2];
      for (int i = 0; i < 2; i++) {
        stepper_data1[i] = rx_data[(offset + 23 + i) % 27];
      }
      HAL_UART_Transmit(&huart3, (uint8_t *)stepper_data1, 2, HAL_MAX_DELAY);

      char stepper_data2[2];
      for (int i = 0; i < 2; i++) {
        stepper_data2[i] = rx_data[(offset + 25 + i) % 27];
      }
      HAL_UART_Transmit(&huart1, (uint8_t *)stepper_data2, 2, HAL_MAX_DELAY);

      if (rx_data[(offset + 26) % 27] == '1') {
        x = 0.0f;
        y = 0.0f;
        theta = 0.0f;
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        prev_left = 0;
        prev_right = 0;
        origin_left = 0;
        origin_right = 0;
        total_dist_left = 0.0f;
        total_dist_right = 0.0f;
        e_sum = 0.0f;
        position_control_active = false;
        control_locked = false;  // Déverrouiller lors de la réinitialisation
        mot_maxon_both(0, 0, 0, 0);
        printf("Odométrie, compteurs d'encodeurs et distances totales réinitialisés à 0\r\n");
      }

      // Vider le tampon rx_data après traitement
      memset(rx_data, 0, sizeof(rx_data));
    }

    HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)rx_data, sizeof(rx_data));

    uint32_t current_time = HAL_GetTick();
    if (current_time - last_update_time >= 10) {
      last_update_time = current_time;

      int16_t curr_left = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
      int16_t curr_right = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);

      int32_t delta_left = (int32_t)curr_left - (int32_t)prev_left;
      int32_t delta_right = (int32_t)curr_right - (int32_t)prev_right;

      if (delta_left < -10000 || delta_left > 10000) {
        delta_left = (curr_left < prev_left) ? (65536 + curr_left - prev_left) : (curr_left - prev_left);
      }
      if (delta_right < -10000 || delta_right > 10000) {
        delta_right = (curr_right < prev_right) ? (65536 + curr_right - prev_right) : (curr_right - prev_right);
      }

      prev_left = curr_left;
      prev_right = curr_right;

      float dist_left = (float)delta_left / ENCODER_COUNTS_PER_REV * WHEEL_CIRCUMFERENCE;
      float dist_right = (float)delta_right / ENCODER_COUNTS_PER_REV * WHEEL_CIRCUMFERENCE;

      total_dist_left += fabs(dist_left);
      total_dist_right += fabs(dist_right);

      if (position_control_active && !control_locked) {
        // Ajouter une tolérance pour éviter les oscillations autour de la cible
        bool left_reached = (s == 1 && curr_left >= target_left - ENCODER_TOLERANCE) ||
                            (s == -1 && curr_left <= target_left + ENCODER_TOLERANCE);
        bool right_reached = (s == 1 && curr_right >= target_right - ENCODER_TOLERANCE) ||
                             (s == -1 && curr_right <= target_right + ENCODER_TOLERANCE);

        int v_L_applied = v_L_base;
        int v_R_applied = v_R_base;

        // Appliquer le contrôleur PI uniquement si les deux roues sont actives et en mouvement droit
        if (is_straight && !left_reached && !right_reached && (v_L_base > 0 || v_R_base > 0)) {
          float e = s * (dist_right - dist_left);
          e_sum += e * 0.01f;
          if (e_sum > 0.1f) e_sum = 0.1f;
          if (e_sum < -0.1f) e_sum = -0.1f;
          int Delta_v_L = (int)(K_p * e + K_i * e_sum);
          int Delta_v_R = -Delta_v_L;
          v_L_applied = v_L_base + Delta_v_L;
          v_R_applied = v_R_base + Delta_v_R;
          if (v_L_applied > 1999) v_L_applied = 1999;
          if (v_L_applied < 0) v_L_applied = 0;
          if (v_R_applied > 1999) v_R_applied = 1999;
          if (v_R_applied < 0) v_R_applied = 0;
        }

        // Arrêter la roue si elle a atteint sa cible et afficher le message une seule fois
        static uint8_t left_reached_count = 0;
        if (left_reached) {
          v_L_applied = 0;
          if (left_reached_count == 0) {
            printf("Roue gauche atteinte: left=%d\r\n", curr_left);
            left_reached_count = 1;
          }
        } else {
          left_reached_count = 0;
        }

        static uint8_t right_reached_count = 0;
        if (right_reached) {
          v_R_applied = 0;
          if (right_reached_count == 0) {
            printf("Roue droite atteinte: right=%d\r\n", curr_right);
            right_reached_count = 1;
          }
        } else {
          right_reached_count = 0;
        }

        // Appliquer les vitesses ajustées
        mot_maxon_both(v_R_applied, inv_R, v_L_applied, inv_L);

        // Débogage : afficher les vitesses appliquées
        static uint8_t speed_print_counter = 0;
        speed_print_counter++;
        if (speed_print_counter >= 10) {
          speed_print_counter = 0;
          printf("Vitesses: v_L_applied=%d, v_R_applied=%d\r\n", v_L_applied, v_R_applied);
        }

        // Arrêter le contrôle de position si les deux roues ont atteint leurs cibles
        if (left_reached && right_reached) {
          position_control_active = false;
          control_locked = true;  // Verrouiller le contrôle
          v_L_base = 0;
          v_R_base = 0;
          e_sum = 0.0f;
          mot_maxon_both(0, 0, 0, 0);  // Forcer l'arrêt des moteurs
          printf("Position cible atteinte: left=%d, right=%d\r\n", curr_left, curr_right);
        }
      }

      float delta_dist = (dist_left + dist_right) / 2.0f;
      float delta_theta = (dist_right - dist_left) / WHEEL_BASE;
      x += delta_dist * cosf(theta);
      y += delta_dist * sinf(theta);
      theta += delta_theta;
      while (theta > M_PI) theta -= 2 * M_PI;
      while (theta < -M_PI) theta += 2 * M_PI;

      static uint8_t odom_send_counter = 0;
      odom_send_counter++;
      if (odom_send_counter >= 10) {
        odom_send_counter = 0;
        char odom_str[64];
        snprintf(odom_str, sizeof(odom_str), "x:%.3f,y:%.3f,theta:%.3f\r\n", x, y, theta);
        HAL_UART_Transmit(&huart5, (uint8_t *)odom_str, strlen(odom_str), HAL_MAX_DELAY);
      }

      static uint8_t print_counter = 0;
      print_counter++;
      if (print_counter >= 10) {
        print_counter = 0;
        printf("Pose: x=%.3f m, y=%.3f m, theta=%.3f rad, left=%d, delta_left=%ld, dist_left=%.3f m, right=%d, delta_right=%ld, dist_right=%.3f m, total_left=%.3f m, total_right=%.3f m\r\n",
               x, y, theta, curr_left, delta_left, dist_left, curr_right, delta_right, dist_right, total_dist_left, total_dist_right);
      }
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function (Encodeur roue droite: PA8, PA9)
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;  // Différent pour détecter la direction
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function (Encodeur roue gauche: PA6, PA7)
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;  // Différent pour détecter la direction
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &hlpuart1) {
    HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)rx_data, sizeof(rx_data));
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
