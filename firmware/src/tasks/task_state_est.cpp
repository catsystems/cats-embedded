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

#include "tasks/task_state_est.h"
#include "config/globals.h"

namespace task {

void StateEstimation::GetEstimationInputData() {
  /* After apogee we assume that the linear acceleration is zero. This assumption is true if the parachute has been
   * ejected. If this assumption is not done, the linear acceleration will be bad because of movement of the rocket
   * due to parachute forces. */
  auto &preprocessing_task = Preprocessing::GetInstance();
  state_estimation_input_t input = preprocessing_task.GetEstimationInput();

  if (m_fsm_enum < DROGUE) {
    m_filter.measured_acceleration = input.acceleration_z;
  } else {
    m_filter.measured_acceleration = 0;
  }

  m_filter.measured_AGL = input.height_AGL;

  /* Do Orientation Filter */
  quaternion_kinematics(&m_orientation_filter, preprocessing_task.GetSIData().gyro);
}

estimation_output_t StateEstimation::GetEstimationOutput() const noexcept {
  estimation_output_t output = {};
  output.height = m_filter.x_bar_data[0];
  output.velocity = m_filter.x_bar_data[1];
  output.acceleration = m_filter.measured_acceleration + m_filter.x_bar_data[2];
  return output;
}

/**
 * @brief Function implementing the task_preprocessing thread.
 * @param argument: Not used
 * @retval None
 */
[[noreturn]] void StateEstimation::Run() noexcept {
  osDelay(1000);

  /* Initialize Kalman Filter */
  init_filter_struct(&m_filter);
  initialize_matrices(&m_filter);

  /* initialize Orientation State Estimation */
  init_orientation_filter(&m_orientation_filter);
  reset_orientation_filter(&m_orientation_filter);

  uint32_t tick_count = osKernelGetTickCount();
  uint32_t tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;
  while (1) {
    /* update fsm enum */
    bool fsm_updated = GetNewFsmEnum();

    /* Reset IMU when we go from moving to READY */
    if ((m_fsm_enum == READY) && fsm_updated) {
      reset_kalman(&m_filter);
      reset_orientation_filter(&m_orientation_filter);
    }

    /* Soft reset kalman filter when we go from Ready to thrusting */
    /* Reset Orientation Estimate when going to thrusting */
    if ((m_fsm_enum == THRUSTING) && fsm_updated) {
      soft_reset_kalman(&m_filter);
      reset_orientation_filter(&m_orientation_filter);
    }

    /* Write measurement data into the filter struct */
    GetEstimationInputData();

    /* Do a Kalman Step */
    kalman_step(&m_filter, m_fsm_enum);

    orientation_info_t orientation_info;
    /*
    log_raw("[%lu] KF: q0: %ld; q1: %ld; q2: %ld; q3: %ld", tick_count, (int32_t)(orientation_filter.estimate_data[0] *
    1000), (int32_t)(orientation_filter.estimate_data[1] * 1000), (int32_t)(orientation_filter.estimate_data[2] * 1000),
              (int32_t)(orientation_filter.estimate_data[3] * 1000));
    */
    for (uint8_t i = 0; i < 4; i++) {
      orientation_info.estimated_orientation[i] = (int16_t)(m_orientation_filter.estimate_data[i] * 10000.0f);
    }

    record(tick_count, ORIENTATION_INFO, &orientation_info);

    /* record filtered data */
    filtered_data_info_t filtered_data_info = {
        .filtered_altitude_AGL = m_filter.measured_AGL,
        .filtered_acceleration = m_filter.measured_acceleration,
    };

    record(tick_count, FILTERED_DATA_INFO, &filtered_data_info);

    /* Log KF outputs */
    flight_info_t flight_info = {.height = m_filter.x_bar_data[0],
                                 .velocity = m_filter.x_bar_data[1],
                                 .acceleration = m_filter.measured_acceleration + m_filter.x_bar_data[2]};
    if (m_fsm_enum >= DROGUE) {
      flight_info.acceleration = m_filter.x_bar_data[2];
    }
    record(tick_count, FLIGHT_INFO, &flight_info);

    // log_info("H: %ld; V: %ld; A: %ld; O: %ld", (int32_t)((float)filter.x_bar.pData[0] * 1000),
    //          (int32_t)((float)filter.x_bar.pData[1] * 1000), (int32_t)(filtered_data_info.filtered_acceleration *
    //          1000), (int32_t)((float)filter.x_bar.pData[2] * 1000));
    log_sim("[%lu]: height: %f, velocity: %f, offset: %f", tick_count, static_cast<double>(m_filter.x_bar.pData[0]),
            static_cast<double>(m_filter.x_bar.pData[1]), static_cast<double>(m_filter.x_bar_data[2]));

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

}  // namespace task
