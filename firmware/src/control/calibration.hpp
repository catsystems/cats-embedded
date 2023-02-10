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

#include "util/types.hpp"

#define GYRO_NUM_SAME_VALUE   200
#define GYRO_ALLOWED_ERROR_SI 3.0f

void calibrate_imu(const vf32_t *accel_data, calibration_data_t *);
bool compute_gyro_calibration(const vf32_t *gyro_data, calibration_data_t *calibration);
void calibrate_gyro(const calibration_data_t *calibration, vf32_t *gyro_data);
