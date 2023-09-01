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
#include "stm32f0xx_hal.h"

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
#define INP_1_Pin GPIO_PIN_0
#define INP_1_GPIO_Port GPIOA
#define ISOL_Pin GPIO_PIN_6
#define ISOL_GPIO_Port GPIOA
#define OWR_Pin GPIO_PIN_7
#define OWR_GPIO_Port GPIOA
#define V_M_Pin GPIO_PIN_0
#define V_M_GPIO_Port GPIOB
#define BTN_1_Pin GPIO_PIN_1
#define BTN_1_GPIO_Port GPIOB
#define L__Pin GPIO_PIN_8
#define L__GPIO_Port GPIOA
#define L___Pin GPIO_PIN_10
#define L___GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_11
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_12
#define LED_2_GPIO_Port GPIOA
#define OUT_1_1_Pin GPIO_PIN_3
#define OUT_1_1_GPIO_Port GPIOB
#define OUT_1_2_Pin GPIO_PIN_4
#define OUT_1_2_GPIO_Port GPIOB
#define REL_2_Pin GPIO_PIN_5
#define REL_2_GPIO_Port GPIOB
#define REL_3_Pin GPIO_PIN_6
#define REL_3_GPIO_Port GPIOB
#define REL_4_Pin GPIO_PIN_7
#define REL_4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
