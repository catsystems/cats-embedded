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

#include "control/calibration.hpp"
#include "util/log.h"

void calibrate_imu(const vf32_t *accel_data, calibration_data_t *calibration) {
  /* first get the largest vector */
  if (fabsf(accel_data->x) >= fabsf(accel_data->y)) {
    if (fabsf(accel_data->x) >= fabsf(accel_data->z)) {
      calibration->axis = 0;
    } else {
      calibration->axis = 2;
    }
  } else {
    if (fabsf(accel_data->y) >= fabsf(accel_data->z)) {
      calibration->axis = 1;
    } else {
      calibration->axis = 2;
    }
  }

  /* Then get the angle (or here the cos(angle)) between vector and gravity for further use */
  switch (calibration->axis) {
    case 0:
      calibration->angle = accel_data->x / GRAVITY;
      if (fabsf(calibration->angle) < 0.3F) {
        calibration->angle = 0.3F;
      }
      log_info("Calibration chose X Axis with invcos(alpha)*1000 = %ld",
               static_cast<int32_t>(1000 * calibration->angle));
      break;
    case 1:
      calibration->angle = accel_data->y / GRAVITY;
      if (fabsf(calibration->angle) < 0.3F) {
        calibration->angle = 0.3F;
      }
      log_info("Calibration chose Y Axis with invcos(alpha)*1000 = %ld",
               static_cast<int32_t>(1000 * calibration->angle));
      break;
    case 2:
      calibration->angle = accel_data->z / GRAVITY;
      if (fabsf(calibration->angle) < 0.3F) {
        calibration->angle = 0.3F;
      }
      log_info("Calibration chose Z Axis with invcos(alpha)*1000 = %ld",
               static_cast<int32_t>(1000 * calibration->angle));
      break;
    default:
      break;
  }
}

bool compute_gyro_calibration(const vf32_t *gyro_data, calibration_data_t *calibration) {
  static int16_t calibration_counter = 0;
  static vf32_t first_gyro_data = {.x = 0, .y = 0, .z = 0};
  static vf32_t averaged_gyro_data = {.x = 0, .y = 0, .z = 0};

  /* compute gyro error */
  const vf32_t vector_error{.x = fabsf(first_gyro_data.x - gyro_data->x),
                            .y = fabsf(first_gyro_data.y - gyro_data->y),
                            .z = fabsf(first_gyro_data.z - gyro_data->z)};

  /* check if the gyro error is inside the bounds
   * if yes, increase counter and compute averaged gyro data
   * if not, reset counter and reset averaged gyro data
   */
  if ((vector_error.x < GYRO_ALLOWED_ERROR_SI) && (vector_error.y < GYRO_ALLOWED_ERROR_SI) &&
      (vector_error.z < GYRO_ALLOWED_ERROR_SI)) {
    calibration_counter++;
    averaged_gyro_data.x += gyro_data->x / static_cast<float>(GYRO_NUM_SAME_VALUE);
    averaged_gyro_data.y += gyro_data->y / static_cast<float>(GYRO_NUM_SAME_VALUE);
    averaged_gyro_data.z += gyro_data->z / static_cast<float>(GYRO_NUM_SAME_VALUE);
  } else {
    calibration_counter = 0;
    averaged_gyro_data.x = 0;
    averaged_gyro_data.y = 0;
    averaged_gyro_data.z = 0;
    first_gyro_data = *gyro_data;
  }

  /* if the counter achieved the defined value, calibrate gyro */
  if (calibration_counter > GYRO_NUM_SAME_VALUE) {
    memcpy(&calibration->gyro_calib, &averaged_gyro_data, sizeof(averaged_gyro_data));
    return true;
  }

  return false;
}

void calibrate_gyro(const calibration_data_t *calibration, vf32_t *gyro_data) {
  gyro_data->x = gyro_data->x - calibration->gyro_calib.x;
  gyro_data->y = gyro_data->y - calibration->gyro_calib.y;
  gyro_data->z = gyro_data->z - calibration->gyro_calib.z;
}
