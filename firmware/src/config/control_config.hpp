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

#define USE_MEDIAN_FILTER
#define MEDIAN_FILTER_SIZE 9

static const float P_INITIAL = 101250.0f;                   // hPa
static const float GRAVITY = 9.81f;                         // m/s^2
static const float TEMPERATURE_0 = 15.0f;                   // °C
static const float BARO_LIFTOFF_MOV_AVG_SIZE = 500.0f;      // samples -> 5 seconds
static const float BARO_LIFTOFF_FAST_MOV_AVG_SIZE = 10.0f;  // samples -> 5 seconds
