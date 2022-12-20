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

#include <optional>

#include "util/log.h"
#include "util/types.h"

namespace task {
[[noreturn]] void task_preprocessing(void *argument);

class Preprocessing {
 public:
  friend void task_preprocessing(void *argument);

  void Run();

  static Preprocessing &GetInstance() {
    if (!s_instance.has_value()) {
      log_raw("Class Sensor Read Created.");
      s_instance = Preprocessing();
    }
    return *s_instance;
  }
  imu_data_t m_imu_data[NUM_IMU]{};
  baro_data_t m_baro_data[NUM_BARO]{};
  magneto_data_t m_magneto_data[NUM_MAGNETO]{};
  accel_data_t m_accel_data[NUM_ACCELEROMETER]{};

 private:
  SI_data_t m_si_data = {};
  SI_data_t m_si_data_old = {};

#ifdef USE_MEDIAN_FILTER
  median_filter_t filter_data = {};
#endif
  sensor_elimination_t sensor_elimination = {};

  /* Calibration Data including the gyro calibration as the first three values and then the angle and axis are for
   * the linear acceleration calibration */
  calibration_data_t calibration = {.gyro_calib = {.x = 0, .y = 0, .z = 0}, .angle = 1, .axis = 2};
  state_estimation_input_t state_est_input = {.acceleration_z = 0.0f, .height_AGL = 0.0f};
  float32_t height_0 = 0.0f;

  /* Gyro Calib tag */
  bool gyro_calibrated = false;

  /* local fsm enum */
  flight_fsm_e new_fsm_enum = MOVING;
  flight_fsm_e old_fsm_enum = MOVING;

  static std::optional<Preprocessing> s_instance;
};
}  // namespace task