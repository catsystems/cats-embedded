/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "util/types.hpp"

inline constexpr uint16_t GYRO_NUM_SAME_VALUE = 200;
inline constexpr float GYRO_ALLOWED_ERROR_SI = 3.0F;

void calibrate_imu(const vf32_t *accel_data, calibration_data_t *);
bool compute_gyro_calibration(const vf32_t *gyro_data, calibration_data_t *calibration);
void calibrate_gyro(const calibration_data_t *calibration, vf32_t *gyro_data);
