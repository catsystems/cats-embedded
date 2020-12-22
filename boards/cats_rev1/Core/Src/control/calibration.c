/*
 * calibration.c
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#include "control/calibration.h"

void calibrate_imu(imu_data_t *imu_data, calibration_data_t *calibration) {
  /* first get the largest vector */
  if (imu_data->acc_x >= imu_data->acc_y) {
    if (imu_data->acc_x >= imu_data->acc_z) {
      calibration->axis = 0;
    } else {
      calibration->axis = 2;
    }
  } else {
    if (imu_data->acc_y >= imu_data->acc_z) {
      calibration->axis = 1;
    } else {
      calibration->axis = 2;
    }
  }

  /* Then get the angle (or here the cos(angle)) between vector and gravity for
   * further use */
  switch (calibration->axis) {
    case 0:
      calibration->angle = (float)(imu_data->acc_x) / (1024);
      break;
    case 1:
      calibration->angle = (float)(imu_data->acc_y) / (1024);
      break;
    case 2:
      calibration->angle = (float)(imu_data->acc_z) / (1024);
      break;
    default:
      break;
  }
}
