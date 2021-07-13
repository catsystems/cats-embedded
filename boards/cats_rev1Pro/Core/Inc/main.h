/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l4xx_hal.h"

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
#define BUTTON_Pin         GPIO_PIN_13
#define BUTTON_GPIO_Port   GPIOC
#define V_PYRO1_Pin        GPIO_PIN_0
#define V_PYRO1_GPIO_Port  GPIOC
#define V_PYRO2_Pin        GPIO_PIN_1
#define V_PYRO2_GPIO_Port  GPIOC
#define V_PYRO3_Pin        GPIO_PIN_2
#define V_PYRO3_GPIO_Port  GPIOC
#define V_BAT_Pin          GPIO_PIN_3
#define V_BAT_GPIO_Port    GPIOC
#define SERVO1_Pin         GPIO_PIN_0
#define SERVO1_GPIO_Port   GPIOA
#define SERVO2_Pin         GPIO_PIN_1
#define SERVO2_GPIO_Port   GPIOA
#define USB_DET_Pin        GPIO_PIN_2
#define USB_DET_GPIO_Port  GPIOA
#define LED1_Pin           GPIO_PIN_4
#define LED1_GPIO_Port     GPIOA
#define LED2_Pin           GPIO_PIN_2
#define LED2_GPIO_Port     GPIOB
#define CS_BARO3_Pin       GPIO_PIN_12
#define CS_BARO3_GPIO_Port GPIOB
#define CS_BARO1_Pin       GPIO_PIN_6
#define CS_BARO1_GPIO_Port GPIOC
#define CS_BARO2_Pin       GPIO_PIN_7
#define CS_BARO2_GPIO_Port GPIOC
#define PYRO1_Pin          GPIO_PIN_8
#define PYRO1_GPIO_Port    GPIOC
#define PYRO2_Pin          GPIO_PIN_9
#define PYRO2_GPIO_Port    GPIOC
#define PYRO3_Pin          GPIO_PIN_8
#define PYRO3_GPIO_Port    GPIOA
#define IO2_Pin            GPIO_PIN_15
#define IO2_GPIO_Port      GPIOA
#define IO3_Pin            GPIO_PIN_10
#define IO3_GPIO_Port      GPIOC
#define CS_MAG_Pin         GPIO_PIN_11
#define CS_MAG_GPIO_Port   GPIOC
#define CS_ACC_Pin         GPIO_PIN_12
#define CS_ACC_GPIO_Port   GPIOC
#define CS_IMU2_Pin        GPIO_PIN_2
#define CS_IMU2_GPIO_Port  GPIOD
#define CS_IMU1_Pin        GPIO_PIN_3
#define CS_IMU1_GPIO_Port  GPIOB
#define INT_ACC_Pin        GPIO_PIN_6
#define INT_ACC_GPIO_Port  GPIOB
#define IO1_Pin            GPIO_PIN_7
#define IO1_GPIO_Port      GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
