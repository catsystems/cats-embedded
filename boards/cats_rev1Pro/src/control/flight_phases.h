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

/* Config */
#define SAMPLING_FREQUENCY 100

/* MOVING */
#define TIME_THRESHOLD_MOV_TO_READY 1000
#define ALLOWED_ACC_ERROR           200
#define ALLOWED_GYRO_ERROR          50
#define MOV_LIFTOFF_THRESHOLD       5000.0f
#define MOV_LIFTOFF_SAFETY_COUNTER  100

/* READY */
#define TIME_THRESHOLD_READY_TO_MOV 500
#define LIFTOFF_SAFETY_COUNTER      10
#define GYRO_SENSITIVITY            0.1f
#define ANGLE_MOVE_MAX              60.0f

/* THRUSTING 1 */
#define COASTING_SAFETY_COUNTER 30

/* COASTING */
#define APOGEE_SAFETY_COUNTER 30

/* APOGEE */
#define PARACHUTE_DESCENT_SPEED  (-100.0f)
#define PARACHUTE_SAFETY_COUNTER 50

/* DROGUE */
#define MAIN_SAFETY_COUNTER 10

/* MAIN */
#define VELOCITY_BOUND_TOUCHDOWN 4.0f
#define TOUCHDOWN_SAFETY_COUNTER 100

void check_flight_phase(flight_fsm_t *fsm_state, imu_data_t *imu_data, estimation_output_t *state_data,
                        control_settings_t *settings);
