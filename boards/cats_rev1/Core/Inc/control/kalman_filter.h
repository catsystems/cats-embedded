/*
 * kalman_filter.h
 *
 *  Created on: Dec 15, 2020
 *      Author: Jonas
 */

#pragma once

#include "util/log.h"
#include "util/types.h"
#include "util/error_handler.h"

#define STD_NOISE_BARO   9.0f    // From data analysis: 2.6f
#define STD_NOISE_IMU    0.004f  // From data analysis: 0.004f
#define STD_NOISE_OFFSET 0.001f

void init_filter_struct(kalman_filter_t *filter);

void initialize_matrices(kalman_filter_t *filter);

void kalman_prediction(kalman_filter_t *filter, state_estimation_data_t *data, sensor_elimination_t *elimination,
                       flight_fsm_e fsm_state);

void reset_kalman(kalman_filter_t *filter, float initial_pressure);

cats_error_e kalman_update_full(kalman_filter_t *filter, state_estimation_data_t *data);

cats_error_e kalman_update_eliminated(kalman_filter_t *filter, state_estimation_data_t *data,
                                      sensor_elimination_t *elimination);

cats_error_e kalman_step(kalman_filter_t *filter, state_estimation_data_t *data, sensor_elimination_t *elimination,
                         flight_fsm_e fsm_state);
