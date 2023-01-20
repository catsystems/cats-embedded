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

#include "control/sensor_elimination.h"
#include "tasks/task_preprocessing.h"

cats_error_e check_sensor_bounds(sensor_elimination_t *elimination, uint8_t index, const sens_info_t *sens_info) {
  cats_error_e status = CATS_ERR_OK;

  const auto &task = task::Preprocessing::GetInstance();

  switch (sens_info->sens_type) {
    case BARO_ID:
      if ((((float32_t)task.m_baro_data[index].pressure * sens_info->conversion_to_SI) > sens_info->upper_limit) ||
          (((float32_t)task.m_baro_data[index].pressure * sens_info->conversion_to_SI) < sens_info->lower_limit)) {
        elimination->faulty_baro[index] = 1;
        status = (cats_error_e)(CATS_ERR_BARO_0 << index);
      }
      break;
    case MAG_ID:
      if (((task.m_magneto_data[index].x * sens_info->conversion_to_SI) > sens_info->upper_limit) ||
          ((task.m_magneto_data[index].x * sens_info->conversion_to_SI) < sens_info->lower_limit)) {
        elimination->faulty_mag[index] = 1;
        status = CATS_ERR_MAG;
      }
      break;
    case IMU_ID_ACC:
      if ((((float32_t)task.m_imu_data[index].acc.x * sens_info->conversion_to_SI) > sens_info->upper_limit) ||
          (((float32_t)task.m_imu_data[index].acc.x * sens_info->conversion_to_SI) < sens_info->lower_limit)) {
        elimination->faulty_imu[index] = 1;
        status = (cats_error_e)(CATS_ERR_IMU_0 << index);
      }
      break;
    case ACC_ID:
      if ((((float32_t)task.m_accel_data[index].x * sens_info->conversion_to_SI) > sens_info->upper_limit) ||
          (((float32_t)task.m_accel_data[index].x * sens_info->conversion_to_SI) < sens_info->lower_limit)) {
        elimination->faulty_acc[index] = 1;
        status = CATS_ERR_ACC;
      }
      break;
    default:
      break;
  }

  return status;
}

cats_error_e check_sensor_freezing(sensor_elimination_t *elimination, uint8_t index, const sens_info_t *sens_info) {
  cats_error_e status = CATS_ERR_OK;

  const auto &task = task::Preprocessing::GetInstance();

  switch (sens_info->sens_type) {
    case BARO_ID:
      if (task.m_baro_data[index].pressure == elimination->last_value_baro[index]) {
        elimination->freeze_counter_baro[index]++;
        if (elimination->freeze_counter_baro[index] > MAX_NUM_SAME_VALUE) {
          elimination->faulty_baro[index] = 1;
          status = (cats_error_e)(CATS_ERR_BARO_0 << index);
        }
      } else {
        elimination->last_value_baro[index] = task.m_baro_data[index].pressure;
        elimination->freeze_counter_baro[index] = 0;
      }
      break;
    case MAG_ID:
      if (task.m_magneto_data[index].x == elimination->last_value_magneto[index]) {
        elimination->freeze_counter_magneto[index]++;
        if (elimination->freeze_counter_magneto[index] > MAX_NUM_SAME_VALUE) {
          elimination->faulty_mag[index] = 1;
          status = CATS_ERR_MAG;
        }
      } else {
        elimination->last_value_magneto[index] = task.m_magneto_data[index].x;
        elimination->freeze_counter_magneto[index] = 0;
      }
      break;
    case IMU_ID_ACC:
      if (task.m_imu_data[index].acc.x == elimination->last_value_imu[index]) {
        elimination->freeze_counter_imu[index]++;
        if (elimination->freeze_counter_imu[index] > MAX_NUM_SAME_VALUE) {
          elimination->faulty_imu[index] = 1;
          status = (cats_error_e)(CATS_ERR_IMU_0 << index);
        }
      } else {
        elimination->last_value_imu[index] = task.m_imu_data[index].acc.x;
        elimination->freeze_counter_imu[index] = 0;
      }
      break;
    case ACC_ID:
      if (task.m_accel_data[index].x == elimination->last_value_accel[index]) {
        elimination->freeze_counter_accel[index]++;
        if (elimination->freeze_counter_accel[index] > MAX_NUM_SAME_VALUE) {
          elimination->faulty_acc[index] = 1;
          status = CATS_ERR_ACC;
        }
      } else {
        elimination->last_value_accel[index] = task.m_accel_data[index].x;
        elimination->freeze_counter_accel[index] = 0;
      }
      break;
    default:
      break;
  }

  return status;
}
