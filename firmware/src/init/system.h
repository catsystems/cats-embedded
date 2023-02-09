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

#include "sensors/lsm6dso32.h"
#include "sensors/ms5607.h"

#include "config/globals.h"

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
