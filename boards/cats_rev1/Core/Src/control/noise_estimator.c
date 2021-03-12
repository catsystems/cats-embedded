/*
 * noise_estimator.c
 *
 *  Created on: Mar 12, 2021
 *      Author: jonas
 */

//
// void update_measurements(kalman_filter_t *filter, int32_t initial_pressure) {
//  log_debug("Resetting Kalman Filter...");
//  float x_dash[3] = {0, 0, 0};
//  float P_dash[3][3] = {{10.0f, 0, 0}, {0, 10.0f, 0}, {0, 0, 10.0f}};
//
//  filter->pressure_0 = (float)initial_pressure;
//
//  const size_t flt_3x3_size = 9 * sizeof(float);
//  const size_t flt_3_size = 3 * sizeof(float);
//  memcpy(filter->P_hat, P_dash, flt_3x3_size);
//  memcpy(filter->P_bar, P_dash, flt_3x3_size);
//  memcpy(filter->x_hat, x_dash, flt_3_size);
//  memcpy(filter->x_bar, x_dash, flt_3_size);
//}
