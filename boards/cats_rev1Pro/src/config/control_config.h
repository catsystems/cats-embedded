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
#include "sensor_config.h"
//#define INCLUDE_NOISE

//#define INCLUDE_SPIKES
//#define INCLUDE_OFFSET

#define USE_ORIENTATION_FILTER
#define USE_ADAPTIVE_GAIN

#define USE_MEDIAN_FILTER
#define MEDIAN_FILTER_SIZE 9

static const float P_INITIAL = 101250.f;   // hPa
static const float GRAVITY = 9.81f;        // m/s^2
static const float TEMPERATURE_0 = 15.0f;  // °C
