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
#define TIME_THRESHOLD_MOV_TO_READY     1000   // num iterations, imu action needs to be 0 for at least 10 seconds
#define ALLOWED_ACC_ERROR               0.6f   // m/s^2, if the IMU measurement is smaller than 0.6 m/s^2 it is not considered as movement for the transition MOVING -> READY
#define ALLOWED_GYRO_ERROR              10.0f  // dps, if the GYRO measurement is smaller than 10 dps it is not considered as movement for the transition MOVING -> READY

/* READY */
#define TIME_THRESHOLD_READY_TO_MOV     500     // num iterations, all 10 seconds (2 * TIME_THRESHOLD_READY_TO_MOV / SAMPLING_FREQUENCY), the integration is reset
#define LIFTOFF_SAFETY_COUNTER          10      // num iterations, if the acceleration is bigger than the threshold for 0.1 s we detect liftoff
#define GYRO_SENSITIVITY                0.3f    // dps, if the GYRO measurement is smaller than 0.3 dps it is not considered as movement for the transition READY -> MOVING
#define ANGLE_MOVE_MAX                  120.0f  // degrees, if the integrated gyro is bigger than 120°, we go back to moving.
#define LIFTOFF_SAFETY_COUNTER_HEIGHT   100     // num iterations, if the height is bigger than 50 m for 1 second, detect liftoff
#define LIFTOFF_HEIGHT_AGL              50.0f   // m, if the height is bigger than 50 m for 1 second, detect liftoff

/* THRUSTING */
#define COASTING_SAFETY_COUNTER         10      // num iterations, acceleration needs to be smaller than 0 for at least 0.1 s for the transition THRUSTING -> COASTING

/* COASTING */
#define APOGEE_SAFETY_COUNTER           30      // num iterations, velocity needs to be smaller than 0 for at least 0.3 s for the transition COASTING -> DROGUE

/* DROGUE */
#define MAIN_SAFETY_COUNTER             30      // num iterations, height needs to be smaller than user-defined for at least 0.3 s for the transition DROGUE -> MAIN

/* MAIN */
#define VELOCITY_BOUND_TOUCHDOWN        2.0f    // m/s, velocity needs to be smaller than this to detect touchdown
#define TOUCHDOWN_SAFETY_COUNTER        100     // num iterations, for at least 1s it needs to be smaller

/* Function which implements the FSM */
void check_flight_phase(flight_fsm_t *fsm_state, vf32_t *acc_data, vf32_t *gyro_data, estimation_output_t *state_data, float32_t height_AGL, bool ready_transition_allowed,
                        const control_settings_t *settings);
