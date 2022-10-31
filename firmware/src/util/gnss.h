/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
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

#include <arm_math.h>
#include <stdint.h>

typedef struct {
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
} __attribute__((packed)) gnss_time_t;

typedef struct {
  float32_t lat;
  float32_t lon;
  uint8_t sats;
} __attribute__((packed)) gnss_position_t;

typedef struct {
  gnss_position_t position;
  gnss_time_t time;
} gnss_data_t;
