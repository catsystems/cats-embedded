/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define DIO2_Pin GPIO_PIN_4
#define DIO2_GPIO_Port GPIOA
#define DIO2_EXTI_IRQn EXTI4_15_IRQn
#define DIO1_Pin GPIO_PIN_5
#define DIO1_GPIO_Port GPIOA
#define DIO1_EXTI_IRQn EXTI4_15_IRQn
#define CS_Pin GPIO_PIN_0
#define CS_GPIO_Port GPIOB
#define BUSY_Pin GPIO_PIN_1
#define BUSY_GPIO_Port GPIOB
#define FE_EN_Pin GPIO_PIN_8
#define FE_EN_GPIO_Port GPIOA
#define TX_EN_Pin GPIO_PIN_6
#define TX_EN_GPIO_Port GPIOC
#define RX_EN_Pin GPIO_PIN_11
#define RX_EN_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOA
#define HARDWARE_ID_Pin GPIO_PIN_3
#define HARDWARE_ID_GPIO_Port GPIOB
#define INT2_Pin GPIO_PIN_5
#define INT2_GPIO_Port GPIOB
#define INT1_Pin GPIO_PIN_8
#define INT1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
