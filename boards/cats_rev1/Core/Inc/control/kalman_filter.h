/*
 * kalman_filter.h
 *
 *  Created on: Dec 15, 2020
 *      Author: Jonas
 */

#ifndef INC_CONTROL_KALMAN_FILTER_H_
#define INC_CONTROL_KALMAN_FILTER_H_

#include "util.h"
#include "control/math_utils.h"

#endif /* INC_CONTROL_KALMAN_FILTER_H_ */

void initialize_matrices(kalman_filter_t *filter);

osStatus_t kalman_step(kalman_filter_t *filter, state_estimation_data_t *data);
