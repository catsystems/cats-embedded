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

#include "target.h"

#if defined(CATS_L4)

/* ADC config */
#ifdef USE_ADC
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
#endif

/* CAN config */
#ifdef USE_CAN
CAN_HandleTypeDef hcan1;
#endif

/* QSPI config */
#ifdef USE_QSPI
QSPI_HandleTypeDef hqspi;
#endif

/* RTC config */
#ifdef USE_RTC
RTC_HandleTypeDef hrtc;
#endif

/* SPI config */
#ifdef USE_SPI1
SPI_HandleTypeDef hspi1;
#endif

#ifdef USE_SPI2
SPI_HandleTypeDef hspi2;
#endif

#ifdef USE_TIMER2
TIM_HandleTypeDef htim2;
#endif

#ifdef USE_TIMER15
TIM_HandleTypeDef htim15;
#endif

#ifdef USE_UART
UART_HandleTypeDef huart1;
#endif

#endif