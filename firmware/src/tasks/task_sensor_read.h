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

#include "task.h"

#include "util/log.h"
#include "util/types.h"

namespace task {
[[noreturn]] void task_sensor_read(void *argument);

class SensorRead : public Task<SensorRead> {
 public:
  friend class Task<SensorRead>;
  friend void task_sensor_read(void *argument);

  void Run() override;

  enum class BaroReadoutType {
    kReadBaroTemperature = 1,
    kReadBaroPressure = 2,
  };

  baro_data_t GetBaro(uint8_t index);
  imu_data_t GetImu(uint8_t index);
  magneto_data_t GetMag(uint8_t index);
  accel_data_t GetAccel(uint8_t index);

 private:
  SensorRead() = default;

  imu_data_t m_imu_data[NUM_IMU]{};
  baro_data_t m_baro_data[NUM_BARO]{};
  magneto_data_t m_magneto_data[NUM_MAGNETO]{};
  accel_data_t m_accel_data[NUM_ACCELEROMETER]{};
  BaroReadoutType m_current_readout{BaroReadoutType::kReadBaroTemperature};
};
}  // namespace task
