/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "util/error_handler.hpp"
#include "util/log.h"
#include "util/types.hpp"

/* The barometric data from the sensor does not agree to the KF assumptions that it has zero offset.
 * Hence, the noise matrices are changed over time. At liftoff, the offset is large and close to apogee
 * it is close to zero. The noise matrices here are therefore not agreeing to the data analysis. */
// Tuned by simulations
inline constexpr float STD_NOISE_BARO = 900.0F;

// From data analysis: 2.6f m - but in practice this value makes the barometer too trustworthy
inline constexpr float STD_NOISE_BARO_INITIAL = 9.0F;

// From data analysis: 0.004f m/s^2
inline constexpr float STD_NOISE_IMU = 0.004F;

inline constexpr float STD_NOISE_OFFSET = 0.000001F;

void init_filter_struct(kalman_filter_t *filter);

void initialize_matrices(kalman_filter_t *filter);

void kalman_prediction(kalman_filter_t *filter);

void reset_kalman(kalman_filter_t *filter);

void soft_reset_kalman(kalman_filter_t *filter);

void kalman_update(kalman_filter_t *filter);

void kalman_step(kalman_filter_t *filter, flight_fsm_e flight_state);
