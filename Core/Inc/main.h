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
#define Led1_Pin GPIO_PIN_4
#define Led1_GPIO_Port GPIOA
#define Led2_Pin GPIO_PIN_5
#define Led2_GPIO_Port GPIOA
#define Led3_Pin GPIO_PIN_6
#define Led3_GPIO_Port GPIOA
#define Led4_Pin GPIO_PIN_7
#define Led4_GPIO_Port GPIOA
#define Buton1_Pin GPIO_PIN_12
#define Buton1_GPIO_Port GPIOB
#define Buton1_EXTI_IRQn EXTI4_15_IRQn
#define Buton2_Pin GPIO_PIN_13
#define Buton2_GPIO_Port GPIOB
#define Buton2_EXTI_IRQn EXTI4_15_IRQn
#define Buton3_Pin GPIO_PIN_14
#define Buton3_GPIO_Port GPIOB
#define Buton3_EXTI_IRQn EXTI4_15_IRQn
#define Buton4_Pin GPIO_PIN_15
#define Buton4_GPIO_Port GPIOB
#define Buton4_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */
#define BUTTON_COUNT		4
#define BUTTON_PORT			GPIOB
#define LED_PORT			GPIOA

extern uint16_t Button_Pins[];
extern volatile uint16_t Debouncing_Counter[BUTTON_COUNT];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
