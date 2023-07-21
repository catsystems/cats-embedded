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

#include "target.h"

#include "config/globals.hpp"
#include "control/calibration.hpp"
#include "control/data_processing.hpp"
#include "tasks/task_preprocessing.hpp"

#include "util/task_util.hpp"

#define MAX_NUM_SAME_VALUE 7

namespace task {

state_estimation_input_t Preprocessing::GetEstimationInput() const noexcept { return m_state_est_input; }

SI_data_t Preprocessing::GetSIData() const noexcept { return m_si_data; }

/**
 * @brief Function implementing the task_preprocessing thread.
 * @param argument: Not used
 * @retval None
 */
[[noreturn]] void Preprocessing::Run() noexcept {
  /* Infinite loop */
  uint32_t tick_count = osKernelGetTickCount();
  constexpr uint32_t tick_update = sysGetTickFreq() / CONTROL_SAMPLING_FREQ;
  while (true) {
    /* update fsm enum */
    bool fsm_updated = GetNewFsmEnum();

    /* get new sensor data */
    m_baro_data[0] = m_task_sensor_read.GetBaro(0);
    m_imu_data[0] = m_task_sensor_read.GetImu(0);

    /* Do the sensor elimination */
    CheckSensors();

    /* average and construct SI Data */
    AvgToSi();

    /* Compute gravity when changing to READY */
    if (fsm_updated && (m_fsm_enum == READY)) {
      calibrate_imu(&m_si_data.acc, &m_calibration);
      global_flight_stats.calibration_data.angle = m_calibration.angle;
      global_flight_stats.calibration_data.axis = m_calibration.axis;
    }

    /* calibrate gyro once at startup */
    if (!m_gyro_calibrated) {
      m_gyro_calibrated = compute_gyro_calibration(&m_si_data.gyro, &m_calibration);
    } else {
      calibrate_gyro(&m_calibration, &m_si_data.gyro);
      global_flight_stats.calibration_data.gyro_calib = m_calibration.gyro_calib;
    }

    /* Compute current height constantly before liftoff. If the state is calibrating, the filter is much faster. */
    if (m_fsm_enum == CALIBRATING) {
      m_height_0 = approx_moving_average(calculate_height(m_si_data.pressure), true);
      global_flight_stats.height_0 = m_height_0;
    }
    /* Compute current height constantly before liftoff. If the state is ready, the filter is much slower. */
    if (m_fsm_enum == READY) {
      m_height_0 = approx_moving_average(calculate_height(m_si_data.pressure), false);
      global_flight_stats.height_0 = m_height_0;
    }

    /* Get Sensor Readings already transformed in the right coordinate Frame */
    TransformData();

    /* Check if there is a Calibration Error */
    if (m_fsm_enum == READY) {
      /* This corresponds to an angle of circa 15° */
      if (m_state_est_input.acceleration_z > 0.5F || m_state_est_input.acceleration_z < -0.5F) {
        faulty_calibration_counter++;
      } else {
        faulty_calibration_counter = 0;
      }
      if (faulty_calibration_counter > kMaxFaultyCalib) {
        add_error(CATS_ERR_CALIB);
      } else {
        clear_error(CATS_ERR_CALIB);
      }
    }

#ifdef USE_MEDIAN_FILTER
    /* Filter the data */
    MedianFilter();
#endif

    memcpy(&m_si_data_old, &m_si_data, sizeof(m_si_data));

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

void Preprocessing::AvgToSi() noexcept {
  float32_t counter = 0;
#if NUM_IMU > 0
  /* Reset SI data */
  m_si_data.acc.x = 0;
  m_si_data.acc.y = 0;
  m_si_data.acc.z = 0;
  m_si_data.gyro.x = 0;
  m_si_data.gyro.y = 0;
  m_si_data.gyro.z = 0;

  /* Sum up all non-eliminated IMUs and transform to SI */
  for (int i = 0; i < NUM_IMU; i++) {
    if (m_sensor_elimination.faulty_imu[i] == 0) {
      counter++;
      m_si_data.acc.x += (float32_t)m_imu_data[i].acc.x * acc_info[i].conversion_to_SI;
      m_si_data.acc.y += (float32_t)m_imu_data[i].acc.y * acc_info[i].conversion_to_SI;
      m_si_data.acc.z += (float32_t)m_imu_data[i].acc.z * acc_info[i].conversion_to_SI;
      m_si_data.gyro.x += (float32_t)m_imu_data[i].gyro.x * gyro_info[i].conversion_to_SI;
      m_si_data.gyro.y += (float32_t)m_imu_data[i].gyro.y * gyro_info[i].conversion_to_SI;
      m_si_data.gyro.z += (float32_t)m_imu_data[i].gyro.z * gyro_info[i].conversion_to_SI;
    }
  }

  /* average for SI data */
  if (counter > 0) {
    m_si_data.acc.x /= counter;
    m_si_data.acc.y /= counter;
    m_si_data.acc.z /= counter;
    m_si_data.gyro.x /= counter;
    m_si_data.gyro.y /= counter;
    m_si_data.gyro.z /= counter;
    clear_error(CATS_ERR_FILTER_ACC);
  } else {
    m_si_data.acc = m_si_data_old.acc;
    m_si_data.gyro = m_si_data_old.gyro;
    add_error(CATS_ERR_FILTER_ACC);
  }

#endif

#if NUM_BARO > 0
  counter = 0;
  m_si_data.pressure = 0;
  for (int i = 0; i < NUM_BARO; i++) {
    if (m_sensor_elimination.faulty_baro[i] == 0) {
      counter++;
      m_si_data.pressure += (float32_t)m_baro_data[i].pressure * baro_info[i].conversion_to_SI;
    }
  }
  if (counter > 0) {
    m_si_data.pressure /= counter;
    clear_error(CATS_ERR_FILTER_HEIGHT);
  } else {
    m_si_data.pressure = m_si_data_old.pressure;
    add_error(CATS_ERR_FILTER_HEIGHT);
  }
#endif
}

void Preprocessing::MedianFilter() noexcept {
  /* Insert into array */
  m_filter_data.acc[m_filter_data.counter] = m_state_est_input.acceleration_z;
  m_filter_data.height_AGL[m_filter_data.counter] = m_state_est_input.height_AGL;

  /* Update Counter */
  m_filter_data.counter++;
  m_filter_data.counter = m_filter_data.counter % MEDIAN_FILTER_SIZE;

  /* Filter data */
  m_state_est_input.acceleration_z = median(m_filter_data.acc);
  m_state_est_input.height_AGL = median(m_filter_data.height_AGL);
}

void Preprocessing::TransformData() noexcept {
  /* Get Data from the Sensors */
  /* Use calibration step to get the correct acceleration */
  switch (this->m_calibration.axis) {
    case 0:
      /* Choose X Axis */
      m_state_est_input.acceleration_z = m_si_data.acc.x / m_calibration.angle - GRAVITY;
      break;
    case 1:
      /* Choose Y Axis */
      m_state_est_input.acceleration_z = m_si_data.acc.y / m_calibration.angle - GRAVITY;
      break;
    case 2:
      /* Choose Z Axis */
      m_state_est_input.acceleration_z = m_si_data.acc.z / m_calibration.angle - GRAVITY;
      break;
    default:
      break;
  }
  this->m_state_est_input.height_AGL = calculate_height(m_si_data.pressure) - m_height_0;
}

void Preprocessing::CheckSensors() noexcept {
  cats_error_e status = CATS_ERR_OK;

  /* IMU */
  for (uint8_t i = 0; i < NUM_IMU; i++) {
    status = (cats_error_e)(CheckSensorBounds(i, &acc_info[i]) | CheckSensorFreezing(i, &acc_info[i]));
    /* Check if accel is not faulty anymore */
    if (status == CATS_ERR_OK) {
      m_sensor_elimination.faulty_imu[i] = 0;
      clear_error((cats_error_e)(CATS_ERR_IMU_0 << i));
    } else {
      add_error(status);
    }
  }

  /* Barometer */
  for (uint8_t i = 0; i < NUM_BARO; i++) {
    status = (cats_error_e)(CheckSensorBounds(i, &baro_info[i]) | CheckSensorFreezing(i, &baro_info[i]));
    /* Check if accel is not faulty anymore */
    if (status == CATS_ERR_OK) {
      m_sensor_elimination.faulty_baro[i] = 0;
      clear_error((cats_error_e)(CATS_ERR_BARO_0 << i));
    } else {
      add_error(status);
    }
  }
}

cats_error_e Preprocessing::CheckSensorBounds(uint8_t index, const sens_info_t *sens_info) noexcept {
  cats_error_e status = CATS_ERR_OK;

  switch (sens_info->sens_type) {
    case SensorType::kBaro:
      if ((((float32_t)m_baro_data[index].pressure * sens_info->conversion_to_SI) > sens_info->upper_limit) ||
          (((float32_t)m_baro_data[index].pressure * sens_info->conversion_to_SI) < sens_info->lower_limit)) {
        m_sensor_elimination.faulty_baro[index] = 1;
        status = (cats_error_e)(CATS_ERR_BARO_0 << index);
      }
      break;
    case SensorType::kAcc:
      if ((((float32_t)m_imu_data[index].acc.x * sens_info->conversion_to_SI) > sens_info->upper_limit) ||
          (((float32_t)m_imu_data[index].acc.x * sens_info->conversion_to_SI) < sens_info->lower_limit)) {
        m_sensor_elimination.faulty_imu[index] = 1;
        status = (cats_error_e)(CATS_ERR_IMU_0 << index);
      }
      break;
    default:
      break;
  }

  return status;
}

cats_error_e Preprocessing::CheckSensorFreezing(uint8_t index, const sens_info_t *sens_info) noexcept {
  cats_error_e status = CATS_ERR_OK;

  switch (sens_info->sens_type) {
    case SensorType::kBaro:
      if (m_baro_data[index].pressure == m_sensor_elimination.last_value_baro[index]) {
        m_sensor_elimination.freeze_counter_baro[index]++;
        if (m_sensor_elimination.freeze_counter_baro[index] > MAX_NUM_SAME_VALUE) {
          m_sensor_elimination.faulty_baro[index] = 1;
          status = (cats_error_e)(CATS_ERR_BARO_0 << index);
        }
      } else {
        m_sensor_elimination.last_value_baro[index] = m_baro_data[index].pressure;
        m_sensor_elimination.freeze_counter_baro[index] = 0;
      }
      break;
    case SensorType::kAcc:
      if (m_imu_data[index].acc.x == m_sensor_elimination.last_value_imu[index]) {
        m_sensor_elimination.freeze_counter_imu[index]++;
        if (m_sensor_elimination.freeze_counter_imu[index] > MAX_NUM_SAME_VALUE) {
          m_sensor_elimination.faulty_imu[index] = 1;
          status = (cats_error_e)(CATS_ERR_IMU_0 << index);
        }
      } else {
        m_sensor_elimination.last_value_imu[index] = m_imu_data[index].acc.x;
        m_sensor_elimination.freeze_counter_imu[index] = 0;
      }
      break;
    default:
      break;
  }

  return status;
}

}  // namespace task
