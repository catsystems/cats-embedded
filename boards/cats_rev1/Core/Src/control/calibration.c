/*
 * calibration.c
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#include "control/calibration.h"
#include "stdlib.h"

void calibrate_imu(imu_data_t *imu_data, calibration_data_t *calibration) {
  /* first get the largest vector */
  if (abs(imu_data->acc_x) >= abs(imu_data->acc_y)) {
    if (abs(imu_data->acc_x) >= abs(imu_data->acc_z)) {
      calibration->axis = 0;
    } else {
      calibration->axis = 2;
    }
  } else {
    if (abs(imu_data->acc_y) >= abs(imu_data->acc_z)) {
      calibration->axis = 1;
    } else {
      calibration->axis = 2;
    }
  }

  /* Then get the angle (or here the cos(angle)) between vector and gravity for
   * further use */
  switch (calibration->axis) {
    case 0:
      calibration->angle = (float)(imu_data->acc_x) / 1024;
      break;
    case 1:
      calibration->angle = (float)(imu_data->acc_y) / 1024;
      break;
    case 2:
      calibration->angle = (float)(imu_data->acc_z) / 1024;
      break;
    default:
      break;
  }
}
