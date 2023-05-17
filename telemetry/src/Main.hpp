/// CATS Flight Software
/// Copyright (C) 2022 Control and Telemetry Systems
///
/// This program is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// This program is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with this program.  If not, see <https://www.gnu.org/licenses/>.

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"

void Error_Handler(void);

#define DIO2_Pin              GPIO_PIN_4
#define DIO2_GPIO_Port        GPIOA
#define DIO2_EXTI_IRQn        EXTI4_15_IRQn
#define DIO1_Pin              GPIO_PIN_5
#define DIO1_GPIO_Port        GPIOA
#define DIO1_EXTI_IRQn        EXTI4_15_IRQn
#define CS_Pin                GPIO_PIN_0
#define CS_GPIO_Port          GPIOB
#define BUSY_Pin              GPIO_PIN_1
#define BUSY_GPIO_Port        GPIOB
#define FE_EN_Pin             GPIO_PIN_8
#define FE_EN_GPIO_Port       GPIOA
#define TX_EN_Pin             GPIO_PIN_6
#define TX_EN_GPIO_Port       GPIOC
#define RX_EN_Pin             GPIO_PIN_11
#define RX_EN_GPIO_Port       GPIOA
#define LED_Pin               GPIO_PIN_15
#define LED_GPIO_Port         GPIOA
#define INT2_Pin              GPIO_PIN_5
#define INT2_GPIO_Port        GPIOB
#define INT1_Pin              GPIO_PIN_8
#define INT1_GPIO_Port        GPIOB
#define N_RST_Pin             GPIO_PIN_12
#define N_RST_GPIO_Port       GPIOA
#define HARDWARE_ID_Pin       GPIO_PIN_3
#define HARDWARE_ID_GPIO_Port GPIOB

#define RADIO_SPI &hspi1

#ifdef __cplusplus
}
#endif
