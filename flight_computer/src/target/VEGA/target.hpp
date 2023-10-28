/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "arm_math.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

/***** Pin config *****/
#define LED1_Pin              GPIO_PIN_13
#define LED1_GPIO_Port        GPIOC
#define LED2_Pin              GPIO_PIN_14
#define LED2_GPIO_Port        GPIOC
#define CS_BARO1_Pin          GPIO_PIN_1
#define CS_BARO1_GPIO_Port    GPIOB
#define CS_IMU1_Pin           GPIO_PIN_0
#define CS_IMU1_GPIO_Port     GPIOB
#define PYRO_EN_Pin           GPIO_PIN_2
#define PYRO_EN_GPIO_Port     GPIOB
#define FLASH_CS_Pin          GPIO_PIN_12
#define FLASH_CS_GPIO_Port    GPIOB
#define TEST_BUTTON_Pin       GPIO_PIN_13
#define TEST_BUTTON_GPIO_Port GPIOB
#define RF_INT1_Pin           GPIO_PIN_8
#define RF_INT1_GPIO_Port     GPIOA
#define USB_DET_Pin           GPIO_PIN_15
#define USB_DET_GPIO_Port     GPIOA
#define IO1_Pin               GPIO_PIN_7
#define IO1_GPIO_Port         GPIOB
#define PYRO1_Pin             GPIO_PIN_8
#define PYRO1_GPIO_Port       GPIOB
#define PYRO2_Pin             GPIO_PIN_9
#define PYRO2_GPIO_Port       GPIOB

/***** Peripherals config *****/
// #define USE_CAN

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
/* ADC config */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
#define ADC_HANDLE hadc1
inline constexpr uint8_t ADC_NUM_CHANNELS = 3;

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
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

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

/* Sensor config */
inline constexpr uint8_t NUM_IMU = 1;
inline constexpr uint8_t NUM_BARO = 1;

inline constexpr uint8_t NUM_PYRO = 2;
inline constexpr uint8_t NUM_LOW_LEVEL_IO = 1;

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

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
extern sens_info_t acc_info[NUM_IMU];
extern sens_info_t gyro_info[NUM_IMU];
extern sens_info_t baro_info[NUM_BARO];
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

#ifdef __cplusplus
extern "C" {
#endif
void SystemClock_Config(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void MX_USB_OTG_FS_PCD_Init(void);

void Error_Handler(void);
void target_pre_init();
bool target_init();
#ifdef __cplusplus
}
#endif
