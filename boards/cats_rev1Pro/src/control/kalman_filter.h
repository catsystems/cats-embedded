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

#pragma once

#include "util/log.h"
#include "util/types.h"
#include "util/error_handler.h"

#define STD_NOISE_BARO   9.0f    // From data analysis: 2.6f
#define STD_NOISE_IMU    0.004f  // From data analysis: 0.004f
#define STD_NOISE_OFFSET 0.000001f

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
