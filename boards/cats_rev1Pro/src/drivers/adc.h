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

#define ADC_NUM_CHANNELS 4

#include "stm32l4xx_hal.h"

typedef enum {ADC_PYRO1 = 0, ADC_PYRO2, ADC_PYRO3, ADC_BATTERY} adc_channels_e;

void adc_init();
uint32_t adc_get(adc_channels_e channel);

extern ADC_HandleTypeDef hadc1;
