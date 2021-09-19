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

#include "stm32l4xx_hal.h"

/** Exported Types **/

typedef struct buzzer_dev {
  // Hardware Configuration
  TIM_HandleTypeDef *timer;
  uint32_t channel;
  uint16_t arr;
  uint8_t started;
  uint8_t start;
  uint16_t volume;
  uint32_t end_time;
} BUZ;

/** Exported Functions **/

void buzzer_set_volume(BUZ *dev, uint16_t volume);
void buzzer_beep(BUZ *dev, uint32_t duration);
void buzzer_set_freq(BUZ *dev, uint32_t frequency);
void buzzer_start(BUZ *dev);
void buzzer_stop(BUZ *dev);
uint8_t buzzer_update(BUZ *dev);
