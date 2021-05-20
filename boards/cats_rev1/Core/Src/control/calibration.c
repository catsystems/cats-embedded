/*
 * calibration.c
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#include "control/calibration.h"
#include "util/log.h"
#include "stdlib.h"

void calibrate_imu(imu_data_t *imu_data, calibration_data_t *calibration,
                   sensor_elimination_t *elimination) {
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
      log_info("Calibration chose X Axis with invcos(alpha)*1000 = %ld",
               (int32_t)(1000 * calibration->angle));
      break;
    case 1:
      calibration->angle = (float)(imu_data->acc_y) / 1024;
      log_info("Calibration chose Y Axis with invcos(alpha)*1000 = %ld",
               (int32_t)(1000 * calibration->angle));
      break;
    case 2:
      calibration->angle = (float)(imu_data->acc_z) / 1024;
      log_info("Calibration chose Z Axis with invcos(alpha)*1000 = %ld",
               (int32_t)(1000 * calibration->angle));
      break;
    default:
      break;
  }
}

void calibrate_magneto(magneto_data_t *magneto_data, magneto_calibration_data_t *calibration_data){

    float test_radii[10] = {2.0f, 2.1f, 2.2f, 2.3f, 2.4f, 2.5f, 2.6f, 2.7f, 2.8f, 2.9f};
    float test_bias[10] = {2.0f, 2.1f, 2.2f, 2.3f, 2.4f, 2.5f, 2.6f, 2.7f, 2.8f, 2.9f};
    int32_t smallest_indices[4] = { 0 };
    float smallest_value = 10000;
    float value;
    for(int radius_i = 0; radius_i < 10; radius_i++){
        for(int bias_x_i = 0; bias_x_i < 10; bias_x_i++){
            for(int bias_y_i = 0; bias_y_i < 10; bias_y_i++){
                for(int bias_z_i = 0; bias_z_i < 10; bias_z_i++){
                    value = test_radii[radius_i]*test_radii[radius_i] - magneto_data->magneto_x*magneto_data->magneto_x - magneto_data->magneto_y*magneto_data->magneto_y - magneto_data->magneto_z*magneto_data->magneto_z;
                    value += -2.0f*magneto_data->magneto_x*test_bias[bias_x_i] -2.0f*magneto_data->magneto_y*test_bias[bias_y_i] -2.0f*magneto_data->magneto_z*test_bias[bias_z_i];
                    value += -test_bias[bias_x_i]*test_bias[bias_x_i] - test_bias[bias_y_i]*test_bias[bias_y_i] - test_bias[bias_z_i]*test_bias[bias_z_i];
                    if(value < smallest_value){
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
