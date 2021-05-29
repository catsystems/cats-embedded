/*
 * flight_phases.h
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#pragma once

#include "util/types.h"

/* Config */
#define SAMPLING_FREQUENCY 100

/* MOVING */
#define TIME_THRESHOLD_MOV_TO_IDLE 1000
#define ALLOWED_ACC_ERROR          200
#define ALLOWED_GYRO_ERROR         50

/* IDLE */
#define TIME_THRESHOLD_IDLE_TO_MOV    500
#define LIFTOFF_ACC_THRESHOLD         2000 /* one g -> 1024. Gravity Included! */
#define LIFTOFF_ACC_THRESHOLD_SQUARED (LIFTOFF_ACC_THRESHOLD * LIFTOFF_ACC_THRESHOLD)
#define LIFTOFF_SAFETY_COUNTER        10
#define GYRO_SENSITIVITY              0.1f
#define ANGLE_MOVE_MAX                60.0f

/* THRUSTING 1 */
#define COASTING_SAFETY_COUNTER 30

/* COASTING */
#define APOGEE_SAFETY_COUNTER 10

/* APOGEE */
#define PARACHUTE_DESCENT_SPEED  (-100.0f)
#define PARACHUTE_SAFETY_COUNTER 50
#define BALLISTIC_SAFETY_COUNTER 300

/* DROGUE */
#define HEIGHT_AGL_MAIN     30.0f
#define MAIN_SAFETY_COUNTER 10

/* MAIN */
#define VELOCITY_BOUND_TOUCHDOWN 4.0f
#define TOUCHDOWN_SAFETY_COUNTER 100

void check_flight_phase(flight_fsm_t *fsm_state, imu_data_t *imu_data, estimation_output_t *state_data,
                        control_settings_t *settings);
