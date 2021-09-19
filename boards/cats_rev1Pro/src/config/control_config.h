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

//#define USE_ORIENTATION_KF
#ifdef USE_ORIENTATION_KF
#define INIT_COV   0.1f
#define NOISE_VEL  0.01f
#define NOISE_POS  0.03f
#define NOISE_BIAS 0.01f
#endif

#if defined(USE_ORIENTATION_FILTER) && defined(USE_ORIENTATION_KF)
#undef USE_ORIENTATION_KF
#endif

#define USE_MEDIAN_FILTER
#define MEDIAN_FILTER_SIZE 9

#define HIGH_G_ACC_INDEX 2
#define NUM_ACC          (NUM_IMU + NUM_ACCELEROMETER)
#define NUM_GYRO         NUM_IMU
#define NUM_PRESSURE     NUM_BARO
#define NUM_TEMPERATURE  NUM_BARO
#define NUM_MAG_FIELD    NUM_MAGNETO