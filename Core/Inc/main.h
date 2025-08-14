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
#include "stm32g4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FORWORD_SW_Pin GPIO_PIN_0
#define FORWORD_SW_GPIO_Port GPIOF
#define FORWORD_LED_Pin GPIO_PIN_1
#define FORWORD_LED_GPIO_Port GPIOF
#define SW2_BIT0_Pin GPIO_PIN_0
#define SW2_BIT0_GPIO_Port GPIOA
#define SW2_BIT1_Pin GPIO_PIN_1
#define SW2_BIT1_GPIO_Port GPIOA
#define SW2_BIT2_Pin GPIO_PIN_2
#define SW2_BIT2_GPIO_Port GPIOA
#define SW2_BIT3_Pin GPIO_PIN_3
#define SW2_BIT3_GPIO_Port GPIOA
#define REVERSE_SW_Pin GPIO_PIN_4
#define REVERSE_SW_GPIO_Port GPIOA
#define REVERSE_SW_EXTI_IRQn EXTI4_IRQn
#define REVERSE_LED_Pin GPIO_PIN_5
#define REVERSE_LED_GPIO_Port GPIOA
#define STATE_LED_Pin GPIO_PIN_7
#define STATE_LED_GPIO_Port GPIOA
#define Z_Pin GPIO_PIN_0
#define Z_GPIO_Port GPIOB
#define ENCODER_A_Pin GPIO_PIN_8
#define ENCODER_A_GPIO_Port GPIOA
#define ENCODER_B_Pin GPIO_PIN_9
#define ENCODER_B_GPIO_Port GPIOA
#define PHASE_Pin GPIO_PIN_5
#define PHASE_GPIO_Port GPIOB
#define LIMIT_SW1_Pin GPIO_PIN_6
#define LIMIT_SW1_GPIO_Port GPIOB
#define LIMIT_SW1_EXTI_IRQn EXTI9_5_IRQn
#define LIMIT_SW2_Pin GPIO_PIN_7
#define LIMIT_SW2_GPIO_Port GPIOB
#define LIMIT_SW2_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
