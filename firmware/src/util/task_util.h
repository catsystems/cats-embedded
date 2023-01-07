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

#include <cstdint>

#include "FreeRTOSConfig.h"

#include "cmsis_os.h"

#define SET_TASK_PARAMS(task, stack_sz)           \
  uint32_t task##_buffer[stack_sz];               \
  StaticTask_t task##_control_block;              \
  const osThreadAttr_t task##_attributes = {      \
      .name = #task,                              \
      .cb_mem = &task##_control_block,            \
      .cb_size = sizeof(task##_control_block),    \
      .stack_mem = &task##_buffer[0],             \
      .stack_size = sizeof(task##_buffer),        \
      .priority = (osPriority_t)osPriorityNormal, \
  };

extern volatile bool rtos_started;

void sysDelay(uint32_t delay);

constexpr uint32_t sysGetTickFreq() { return configTICK_RATE_HZ; }
