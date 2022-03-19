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

#include "arm_math.h"
#include "target.h"

typedef enum {
  SENS_TYPE_INVALID = 0,
  ICM20601_ID_ACC,
  ICM20601_ID_GYRO,
  MS5607_ID,
  MMC5983MA_ID,
  H3LIS100DL_ID,
  HEHE4 = 0x7FFFFFFF /* TODO <- optimize these enums and remove this guy */
} sens_type_e;

typedef struct {
  sens_type_e sens_type;
  float32_t conversion_to_SI;
  float32_t upper_limit;
  float32_t lower_limit;
  float32_t resolution;
} sens_info_t;

extern sens_info_t acc_info[NUM_IMU + NUM_ACCELEROMETER];
extern sens_info_t gyro_info[NUM_IMU];
extern sens_info_t mag_info[NUM_MAGNETO];
extern sens_info_t baro_info[NUM_BARO];
