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

#include "task.h"

#include "sensors/lsm6dso32.h"
#include "sensors/ms5607.h"
#include "util/log.h"
#include "util/types.h"

namespace task {

class SensorRead final : public Task<SensorRead, 512> {
 public:
  SensorRead() = default;
  explicit SensorRead(sensors::Lsm6dso32* imu, sensors::Ms5607* barometer) : m_imu(imu), m_barometer(barometer) {
  }

  [[nodiscard]] baro_data_t GetBaro(uint8_t index) const noexcept;
  [[nodiscard]] imu_data_t GetImu(uint8_t index) const noexcept;
  [[nodiscard]] magneto_data_t GetMag(uint8_t index) const noexcept;
  [[nodiscard]] accel_data_t GetAccel(uint8_t index) const noexcept;

 private:
  [[noreturn]] void Run() noexcept override;

  enum class BaroReadoutType {
    kReadBaroTemperature = 1,
    kReadBaroPressure = 2,
  };

  sensors::Lsm6dso32* m_imu {nullptr};
  sensors::Ms5607* m_barometer {nullptr};

  imu_data_t m_imu_data[NUM_IMU]{};
  baro_data_t m_baro_data[NUM_BARO]{};
  magneto_data_t m_magneto_data[NUM_MAGNETO]{};
  accel_data_t m_accel_data[NUM_ACCELEROMETER]{};
  BaroReadoutType m_current_readout{BaroReadoutType::kReadBaroTemperature};
};

}  // namespace task
