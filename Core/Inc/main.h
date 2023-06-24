/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define Enc_Z_Pin GPIO_PIN_14
#define Enc_Z_GPIO_Port GPIOC
#define Motor_RPWM_Pin GPIO_PIN_0
#define Motor_RPWM_GPIO_Port GPIOA
#define Motor_LPWM_Pin GPIO_PIN_1
#define Motor_LPWM_GPIO_Port GPIOA
#define Motor_REn_Pin GPIO_PIN_2
#define Motor_REn_GPIO_Port GPIOA
#define Motor_LEn_Pin GPIO_PIN_3
#define Motor_LEn_GPIO_Port GPIOA
#define Enc_A_Pin GPIO_PIN_12
#define Enc_A_GPIO_Port GPIOB
#define Enc_A_EXTI_IRQn EXTI15_10_IRQn
#define Enc_B_Pin GPIO_PIN_13
#define Enc_B_GPIO_Port GPIOB
#define Tx1_motor_Pin GPIO_PIN_9
#define Tx1_motor_GPIO_Port GPIOA
#define Rx1_motor_Pin GPIO_PIN_10
#define Rx1_motor_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
