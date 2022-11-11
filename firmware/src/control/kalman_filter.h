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

#include "util/error_handler.h"
#include "util/log.h"
#include "util/types.h"

/* The barometric data from the sensor does not agree to the KF assumptions that it has zero offset.
 * Hence, the noise matrices are changed over time. At liftoff, the offset is large and close to apogee
 * it is close to zero. The noise matrices here are therefore not agreeing to the data analysis. */
#define STD_NOISE_BARO         9000.0f    // Tuned by simulations
#define STD_NOISE_BARO_INITIAL 9.0f       // From data analysis: 2.6f m - but in practice this value makes the barometer too trustworthy
#define STD_NOISE_IMU          0.004f     // From data analysis: 0.004f m/s^2
#define STD_NOISE_OFFSET       0.000001f

void init_filter_struct(kalman_filter_t *filter);

void initialize_matrices(kalman_filter_t *filter);

void kalman_prediction(kalman_filter_t *filter);

void reset_kalman(kalman_filter_t *filter);

void soft_reset_kalman(kalman_filter_t *filter);

void kalman_update(kalman_filter_t *filter);

void kalman_step(kalman_filter_t *filter, flight_fsm_e flight_state);
