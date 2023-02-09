/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
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

#include "arm_math.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

/***** Pin config *****/
#define LED1_Pin           GPIO_PIN_13
#define LED1_GPIO_Port     GPIOC
#define LED2_Pin           GPIO_PIN_14
#define LED2_GPIO_Port     GPIOC
#define CS_BARO1_Pin       GPIO_PIN_1
#define CS_BARO1_GPIO_Port GPIOB
#define CS_IMU1_Pin        GPIO_PIN_0
#define CS_IMU1_GPIO_Port  GPIOB
#define PYRO_EN_Pin        GPIO_PIN_2
#define PYRO_EN_GPIO_Port  GPIOB
#define FLASH_CS_Pin       GPIO_PIN_12
#define FLASH_CS_GPIO_Port GPIOB
#define RF_INT2_Pin        GPIO_PIN_13
#define RF_INT2_GPIO_Port  GPIOB
#define RF_INT1_Pin        GPIO_PIN_8
#define RF_INT1_GPIO_Port  GPIOA
#define USB_DET_Pin        GPIO_PIN_15
#define USB_DET_GPIO_Port  GPIOA
#define IO1_Pin            GPIO_PIN_7
#define IO1_GPIO_Port      GPIOB
#define PYRO1_Pin          GPIO_PIN_8
#define PYRO1_GPIO_Port    GPIOB
#define PYRO2_Pin          GPIO_PIN_9
#define PYRO2_GPIO_Port    GPIOB

/***** Peripherals config *****/
// #define USE_CAN

/* ADC config */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
#define ADC_HANDLE       hadc1
#define ADC_NUM_CHANNELS 3

/* RTC config */
extern RTC_HandleTypeDef hrtc;

/* SPI config */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

/* Timer config */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* CAN config */
#ifdef USE_CAN
extern CAN_HandleTypeDef hcan1;
#define CAN_HANDLE hcan1
#endif

/* UART config */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/***** Device config *****/

/* Flash Config */
#define FLASH_SPI_HANDLE hspi2

#define TELEMETRY_UART_HANDLE huart1

#define RTC_HANDLE hrtc

#define SERVO_TIMER_HANDLE    htim3
#define SERVO_TIMER_CHANNEL_1 TIM_CHANNEL_1
#define SERVO_TIMER_CHANNEL_2 TIM_CHANNEL_2

#define BUZZER_TIMER_HANDLE  htim4
#define BUZZER_TIMER_CHANNEL TIM_CHANNEL_1

/* USB config */
#define TIMUsb               TIM5
#define CDC_POLLING_INTERVAL 2  // ms
#define TIMUsb_CLK_ENABLE    __HAL_RCC_TIM5_CLK_ENABLE
#define TIMUsb_IRQn          TIM5_IRQn
#define TIMUsb_IRQHandler    TIM5_IRQHandler

/* Sensor config */
#define NUM_IMU  1
#define NUM_BARO 1

#define NUM_PYRO         2
#define NUM_LOW_LEVEL_IO 1

enum class SensorType : uint32_t {
  kInvalid = 0,
  kAcc,
  kGyro,
  kBaro,
};

struct sens_info_t {
  SensorType sens_type;
  float32_t conversion_to_SI;
  float32_t upper_limit;
  float32_t lower_limit;
  float32_t resolution;
};

extern sens_info_t acc_info[NUM_IMU];
extern sens_info_t gyro_info[NUM_IMU];
extern sens_info_t baro_info[NUM_BARO];

#ifdef __cplusplus
extern "C" {
#endif
void SystemClock_Config(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void Error_Handler(void);
void target_pre_init();
bool target_init();
#ifdef __cplusplus
}
#endif
