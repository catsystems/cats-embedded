/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
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

#include "util/types.hpp"

#include <cstdint>

/* CALIBRATING */
// num iterations, imu action needs to be 0 for at least 10 seconds
inline constexpr uint16_t TIME_THRESHOLD_CALIB_TO_READY = 1000;

// m/s^2, if the IMU measurement is smaller than 0.6 m/s^2 it is not considered as movement for the transition
// CALIBRATING -> READY
inline constexpr float ALLOWED_ACC_ERROR = 0.6F;

// dps, if the GYRO measurement is smaller than 10 dps it is not considered as movement for the transition CALIBRATING
// -> READY
inline constexpr float ALLOWED_GYRO_ERROR = 10.0F;

/* READY */

// num iterations, if the acceleration is bigger than the threshold for 0.1 s we detect liftoff
inline constexpr uint16_t LIFTOFF_SAFETY_COUNTER = 10;

/* THRUSTING */
// num iterations, acceleration needs to be smaller than 0 for at least 0.1 s for the transition THRUSTING -> COASTING
inline constexpr uint16_t COASTING_SAFETY_COUNTER = 10;

/* COASTING */
// num iterations, velocity needs to be smaller than 0 for at least 0.3 s for the transition COASTING -> DROGUE
inline constexpr uint16_t APOGEE_SAFETY_COUNTER = 30;

/* DROGUE */
// num iterations, height needs to be smaller than user-defined for at least 0.3 s for the transition DROGUE -> MAIN
inline constexpr uint16_t MAIN_SAFETY_COUNTER = 30;
// tick counts [ms]
inline constexpr uint16_t MIN_TICK_COUNTS_BETWEEN_THRUSTING_APOGEE = 1500;

/* MAIN */
// m/s, velocity needs to be smaller than this to detect touchdown
inline constexpr float VELOCITY_BOUND_TOUCHDOWN = 3.0F;

// num iterations, for at least 1s it needs to be smaller
inline constexpr uint16_t TOUCHDOWN_SAFETY_COUNTER = 100;

/* Function which implements the FSM */
void check_flight_phase(flight_fsm_t *fsm_state, vf32_t acc_data, vf32_t gyro_data, estimation_output_t state_data,
                        const control_settings_t *settings);
