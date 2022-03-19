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
#include "config/globals.h"

static cats_error_e check_sensor_bounds(sensor_elimination_t *elimination, uint8_t index, const sens_info_t *sens_info);
static cats_error_e check_sensor_freezing(sensor_elimination_t *elimination, uint8_t index,
                                          const sens_info_t *sens_info);

void check_sensors(sensor_elimination_t *elimination) {
  cats_error_e status = CATS_ERR_OK;

  /* Accelerometers */
  for (uint8_t i = 0; i < NUM_ACCELEROMETER; i++) {
    status = check_sensor_bounds(elimination, i, &acc_info[NUM_IMU + i]);
    status |= check_sensor_freezing(elimination, i, &acc_info[NUM_IMU + i]);
    /* Check if accel is not faulty anymore */
    if (status == CATS_ERR_OK) {
      elimination->faulty_acc[i] = 0;
      clear_error(CATS_ERR_ACC);
    } else {
      add_error(status);
    }
    status = CATS_ERR_OK;
  }

  /* IMU */
  for (uint8_t i = 0; i < NUM_IMU; i++) {
    status = check_sensor_bounds(elimination, i, &acc_info[i]);
    status |= check_sensor_freezing(elimination, i, &acc_info[i]);
    /* Check if accel is not faulty anymore */
    if (status == CATS_ERR_OK) {
      elimination->faulty_imu[i] = 0;
      clear_error(CATS_ERR_IMU_0 << i);
    } else {
      add_error(status);
    }
    status = CATS_ERR_OK;
  }

  /* Barometer */
  for (uint8_t i = 0; i < NUM_BARO; i++) {
    status = check_sensor_bounds(elimination, i, &baro_info[i]);
    status |= check_sensor_freezing(elimination, i, &baro_info[i]);
    /* Check if accel is not faulty anymore */
    if (status == CATS_ERR_OK) {
      elimination->faulty_baro[i] = 0;
      clear_error(CATS_ERR_BARO_0 << i);
    } else {
      add_error(status);
    }
    status = CATS_ERR_OK;
  }

  /* Magneto */
  for (uint8_t i = 0; i < NUM_MAGNETO; i++) {
    status = check_sensor_bounds(elimination, i, &mag_info[i]);
    status |= check_sensor_freezing(elimination, i, &mag_info[i]);
    /* Check if accel is not faulty anymore */
    if (status == CATS_ERR_OK) {
      elimination->faulty_mag[i] = 0;
      clear_error(CATS_ERR_MAG);
    } else {
      add_error(status);
    }
    status = CATS_ERR_OK;
  }
}

static cats_error_e check_sensor_bounds(sensor_elimination_t *elimination, uint8_t index,
                                        const sens_info_t *sens_info) {
  cats_error_e status = CATS_ERR_OK;

  switch (sens_info->sens_type) {
    case MS5607_ID:
      if ((((float32_t)global_baro[index].pressure * sens_info->conversion_to_SI) > sens_info->upper_limit) ||
          (((float32_t)global_baro[index].pressure * sens_info->conversion_to_SI) < sens_info->lower_limit)) {
        elimination->faulty_baro[index] = 1;
        status = CATS_ERR_BARO_0 << index;
      }
      break;
    case MMC5983MA_ID:
      if (((global_magneto[index].x * sens_info->conversion_to_SI) > sens_info->upper_limit) ||
          ((global_magneto[index].x * sens_info->conversion_to_SI) < sens_info->lower_limit)) {
        elimination->faulty_mag[index] = 1;
        status = CATS_ERR_MAG;
      }
      break;
    case ICM20601_ID_ACC:
      if ((((float32_t)global_imu[index].acc.x * sens_info->conversion_to_SI) > sens_info->upper_limit) ||
          (((float32_t)global_imu[index].acc.x * sens_info->conversion_to_SI) < sens_info->lower_limit)) {
        elimination->faulty_imu[index] = 1;
        status = CATS_ERR_IMU_0 << index;
      }
      break;
    case H3LIS100DL_ID:
      if ((((float32_t)global_accel[index].x * sens_info->conversion_to_SI) > sens_info->upper_limit) ||
          (((float32_t)global_accel[index].x * sens_info->conversion_to_SI) < sens_info->lower_limit)) {
        elimination->faulty_acc[index] = 1;
        status = CATS_ERR_ACC;
      }
      break;
    default:
      break;
  }

  return status;
}

static cats_error_e check_sensor_freezing(sensor_elimination_t *elimination, uint8_t index,
                                          const sens_info_t *sens_info) {
  cats_error_e status = CATS_ERR_OK;

  switch (sens_info->sens_type) {
    case MS5607_ID:
      if (global_baro[index].pressure == elimination->last_value_baro[index]) {
        elimination->freeze_counter_baro[index]++;
        if (elimination->freeze_counter_baro[index] > MAX_NUM_SAME_VALUE) {
          elimination->faulty_baro[index] = 1;
          status = CATS_ERR_BARO_0 << index;
        }
      } else {
        elimination->last_value_baro[index] = global_baro[index].pressure;
        elimination->freeze_counter_baro[index] = 0;
      }
      break;
    case MMC5983MA_ID:
      if (global_magneto[index].x == elimination->last_value_magneto[index]) {
        elimination->freeze_counter_magneto[index]++;
        if (elimination->freeze_counter_magneto[index] > MAX_NUM_SAME_VALUE) {
          elimination->faulty_mag[index] = 1;
          status = CATS_ERR_MAG;
        }
      } else {
        elimination->last_value_magneto[index] = global_magneto[index].x;
        elimination->freeze_counter_magneto[index] = 0;
      }
      break;
    case ICM20601_ID_ACC:
      if (global_imu[index].acc.x == elimination->last_value_imu[index]) {
        elimination->freeze_counter_imu[index]++;
        if (elimination->freeze_counter_imu[index] > MAX_NUM_SAME_VALUE) {
          elimination->faulty_imu[index] = 1;
          status = CATS_ERR_IMU_0 << index;
        }
      } else {
        elimination->last_value_imu[index] = global_imu[index].acc.x;
        elimination->freeze_counter_imu[index] = 0;
      }
      break;
    case H3LIS100DL_ID:
      if (global_accel[index].x == elimination->last_value_accel[index]) {
        elimination->freeze_counter_accel[index]++;
        if (elimination->freeze_counter_accel[index] > MAX_NUM_SAME_VALUE) {
          elimination->faulty_acc[index] = 1;
          status = CATS_ERR_ACC;
        }
      } else {
        elimination->last_value_accel[index] = global_accel[index].x;
        elimination->freeze_counter_accel[index] = 0;
      }
      break;
    default:
      break;
  }

  return status;
}
