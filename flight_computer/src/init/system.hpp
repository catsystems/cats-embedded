/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "sensors/lsm6dso32.hpp"
#include "sensors/ms5607.hpp"

#include "config/globals.hpp"

void init_storage();

template <typename TImu, typename TBaro>
void init_devices(TImu& imu, TBaro& barometer) {
  // Initialize the IMU
  uint32_t timeout_counter = 0U;
  while (!imu.Init()) {
    HAL_Delay(10);
    if (++timeout_counter > 20) {
      log_error("IMU initialization failed");
      break;
    }
  }
  if (timeout_counter < 20) {
    imu_initialized[0] = true;
  }

  // Initialize the Barometer
  timeout_counter = 0U;
  while (!barometer.Init()) {
    HAL_Delay(10);
    if (++timeout_counter > 20) {
      log_error("Barometer initialization failed");
      break;
    }
  }
}
