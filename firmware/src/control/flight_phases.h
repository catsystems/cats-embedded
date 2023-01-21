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

#include "util/types.h"

/* MOVING */
// num iterations, imu action needs to be 0 for at least 10 seconds
#define TIME_THRESHOLD_MOV_TO_READY 1000

// m/s^2, if the IMU measurement is smaller than 0.6 m/s^2 it is not considered as movement for the transition MOVING ->
// READY
#define ALLOWED_ACC_ERROR 0.6f

// dps, if the GYRO measurement is smaller than 10 dps it is not considered as movement for the transition MOVING ->
// READY
#define ALLOWED_GYRO_ERROR 10.0f

/* READY */
// num iterations, all 10 seconds (2 * TIME_THRESHOLD_READY_TO_MOV / SAMPLING_FREQUENCY), the integration is reset
#define TIME_THRESHOLD_READY_TO_MOV 500

// num iterations, if the acceleration is bigger than the threshold for 0.1 s we detect liftoff
#define LIFTOFF_SAFETY_COUNTER 10

// dps, if the GYRO measurement is smaller than 0.3 dps it is not considered as movement for the transition READY ->
// MOVING
#define GYRO_SENSITIVITY 0.3f

// degrees, if the integrated gyro is bigger than 120°, we go back to moving.
#define ANGLE_MOVE_MAX 120.0f

// num iterations, if the height is bigger than config.control_settings.liftoff_height_agl m for 1 second, detect
// liftoff
#define LIFTOFF_SAFETY_COUNTER_HEIGHT 100

/* THRUSTING */
// num iterations, acceleration needs to be smaller than 0 for at least 0.1 s for the transition THRUSTING -> COASTING
#define COASTING_SAFETY_COUNTER 10

/* COASTING */
// num iterations, velocity needs to be smaller than 0 for at least 0.3 s for the transition COASTING -> DROGUE
#define APOGEE_SAFETY_COUNTER 30

/* DROGUE */
// num iterations, height needs to be smaller than user-defined for at least 0.3 s for the transition DROGUE -> MAIN
#define MAIN_SAFETY_COUNTER               30
#define MIN_TIME_BETWEEN_THRUSTING_APOGEE 1500

/* MAIN */
// m/s, velocity needs to be smaller than this to detect touchdown
#define VELOCITY_BOUND_TOUCHDOWN 3.0f

// num iterations, for at least 1s it needs to be smaller
#define TOUCHDOWN_SAFETY_COUNTER 100

/* Function which implements the FSM */
void check_flight_phase(flight_fsm_t *fsm_state, vf32_t acc_data, vf32_t gyro_data, estimation_output_t state_data,
                        float32_t height_AGL, bool ready_transition_allowed, const control_settings_t *settings);
