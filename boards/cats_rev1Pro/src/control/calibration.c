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

#include "control/calibration.h"
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

  /* Then get the angle (or here the cos(angle)) between vector and gravity for
   * further use */
  switch (calibration->axis) {
    case 0:
      calibration->angle = accel_data->x / GRAVITY;
      log_info("Calibration chose X Axis with invcos(alpha)*1000 = %ld", (int32_t)(1000 * calibration->angle));
      break;
    case 1:
      calibration->angle = accel_data->y / GRAVITY;
      log_info("Calibration chose Y Axis with invcos(alpha)*1000 = %ld", (int32_t)(1000 * calibration->angle));
      break;
    case 2:
      calibration->angle = accel_data->z / GRAVITY;
      log_info("Calibration chose Z Axis with invcos(alpha)*1000 = %ld", (int32_t)(1000 * calibration->angle));
      break;
    default:
      break;
  }
}

void calibrate_magneto(magneto_data_t *magneto_data, magneto_calibration_data_t *calibration_data) {
  float test_radii[10] = {2.0f, 2.1f, 2.2f, 2.3f, 2.4f, 2.5f, 2.6f, 2.7f, 2.8f, 2.9f};
  float test_bias[10] = {2.0f, 2.1f, 2.2f, 2.3f, 2.4f, 2.5f, 2.6f, 2.7f, 2.8f, 2.9f};
  int32_t smallest_indices[4] = {0};
  float smallest_value = 10000;
  float value;
  for (int radius_i = 0; radius_i < 10; radius_i++) {
    for (int bias_x_i = 0; bias_x_i < 10; bias_x_i++) {
      for (int bias_y_i = 0; bias_y_i < 10; bias_y_i++) {
        for (int bias_z_i = 0; bias_z_i < 10; bias_z_i++) {
          value = test_radii[radius_i] * test_radii[radius_i] - magneto_data->x * magneto_data->x -
                  magneto_data->y * magneto_data->y - magneto_data->z * magneto_data->z;
          value += -2.0f * magneto_data->x * test_bias[bias_x_i] - 2.0f * magneto_data->y * test_bias[bias_y_i] -
                   2.0f * magneto_data->z * test_bias[bias_z_i];
          value += -test_bias[bias_x_i] * test_bias[bias_x_i] - test_bias[bias_y_i] * test_bias[bias_y_i] -
                   test_bias[bias_z_i] * test_bias[bias_z_i];
          if (value < smallest_value) {
            smallest_value = value;
            smallest_indices[0] = radius_i;
            smallest_indices[1] = bias_x_i;
            smallest_indices[2] = bias_y_i;
            smallest_indices[3] = bias_z_i;
          }
        }
      }
    }
  }

  calibration_data->magneto_radius = test_radii[smallest_indices[0]];
  calibration_data->magneto_beta[0] = test_radii[smallest_indices[1]];
  calibration_data->magneto_beta[1] = test_radii[smallest_indices[2]];
  calibration_data->magneto_beta[2] = test_radii[smallest_indices[3]];
}
