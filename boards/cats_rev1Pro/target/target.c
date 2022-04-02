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

#if defined(CATS_ORION)

/* ADC config */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* QSPI config */
QSPI_HandleTypeDef hqspi;

/* RTC config */
RTC_HandleTypeDef hrtc;

/* SPI config */
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

/* CAN config */
#ifdef USE_CAN
CAN_HandleTypeDef hcan1;
#endif

/* UART config */
#ifdef USE_UART
UART_HandleTypeDef huart1;
#endif

#endif