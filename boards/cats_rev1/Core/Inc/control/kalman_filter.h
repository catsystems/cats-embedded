/*
 * kalman_filter.h
 *
 *  Created on: Dec 15, 2020
 *      Author: Jonas
 */

#ifndef INC_CONTROL_KALMAN_FILTER_H_
#define INC_CONTROL_KALMAN_FILTER_H_

#include "util/log.h"
#include "control/math_utils.h"

#endif /* INC_CONTROL_KALMAN_FILTER_H_ */

void initialize_matrices(kalman_filter_t *filter);

void kalman_prediction(kalman_filter_t *filter, state_estimation_data_t *data,
                       sensor_elimination_t *elimination);

void reset_kalman(kalman_filter_t *filter);

cats_status_e kalman_update_full(kalman_filter_t *filter,
                                 state_estimation_data_t *data);

cats_status_e kalman_update_eliminated(kalman_filter_t *filter,
                                       state_estimation_data_t *data,
                                       sensor_elimination_t *elimination);

cats_status_e kalman_step(kalman_filter_t *filter,
                          state_estimation_data_t *data,
                          sensor_elimination_t *elimination);
