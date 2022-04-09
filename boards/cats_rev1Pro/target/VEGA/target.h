/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "stm32f4xx_hal.h"

/***** Pin config *****/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define BARO_Pin GPIO_PIN_0
#define BARO_GPIO_Port GPIOB
#define IMU_Pin GPIO_PIN_1
#define IMU_GPIO_Port GPIOB
#define PYRO_EN_Pin GPIO_PIN_2
#define PYRO_EN_GPIO_Port GPIOB
#define FLASH_CS_Pin GPIO_PIN_12
#define FLASH_CS_GPIO_Port GPIOB
#define RF_INT2_Pin GPIO_PIN_13
#define RF_INT2_GPIO_Port GPIOB
#define RF_INT1_Pin GPIO_PIN_8
#define RF_INT1_GPIO_Port GPIOA
#define USB_DET_Pin GPIO_PIN_15
#define USB_DET_GPIO_Port GPIOA
#define IO1_Pin GPIO_PIN_7
#define IO1_GPIO_Port GPIOB
#define PYRO1_Pin GPIO_PIN_8
#define PYRO1_GPIO_Port GPIOB
#define PYRO2_Pin GPIO_PIN_9
#define PYRO2_GPIO_Port GPIOB

/***** Peripherals config *****/
// #define USE_CAN
// #define USE_UART

/* ADC config */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
#define ADC_HANDLE hadc1
#define ADC_NUM_CHANNELS 3

/* RTC config */
extern RTC_HandleTypeDef hrtc;

/* SPI config */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

/* Timer config */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim15;

/* CAN config */
#ifdef USE_CAN
extern CAN_HandleTypeDef hcan1;
#define CAN_HANDLE hcan1
#endif

/* UART config */
#ifdef USE_UART
extern UART_HandleTypeDef huart1;
#endif

/***** Device config *****/
#define FLASH_QSPI_HANDLE hqspi
#define RTC_HANDLE hrtc

#define SERVO_TIMER_HANDLE htim2
#define SERVO_TIMER_CHANNEL_1 TIM_CHANNEL_2
#define SERVO_TIMER_CHANNEL_2 TIM_CHANNEL_1

#define BUZZER_TIMER_HANDLE htim15
#define BUZZER_TIMER_CHANNEL TIM_CHANNEL_2

/* USB config */
#define TIMUsb TIM7
#define CDC_POLLING_INTERVAL 2 // ms
#define TIMUsb_CLK_ENABLE __HAL_RCC_TIM7_CLK_ENABLE
#define TIMUsb_IRQn TIM7_IRQn
#define TIMUsb_IRQHandler TIM7_IRQHandler

/* Sensor config */
#define NUM_IMU           2
#define NUM_MAGNETO       1
#define NUM_ACCELEROMETER 1
#define NUM_BARO          3

#define IMU_SPI_HANDLE hspi1
#define ACCEL_SPI_HANDLE hspi1
#define MAG_SPI_HANDLE hspi1
#define BARO_SPI_HANDLE hspi2

void Error_Handler(void);
void target_pre_init();
void target_init();
