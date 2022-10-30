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

#include "target.h"

#include <stdbool.h>

typedef struct servo_dev {
  // Hardware Configuration
  TIM_HandleTypeDef *timer;
  uint32_t channel;
  uint32_t pulse;
  int8_t started;
} SERVO;

void servo_set_position(SERVO *dev, uint16_t angle);
void servo_set_onoff(SERVO *dev, bool status);
void servo_start(SERVO *dev);
void servo_stop(SERVO *dev);
