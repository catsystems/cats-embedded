/*
 * flight_phases.h
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#ifndef INC_CONTROL_FLIGHT_PHASES_H_
#define INC_CONTROL_FLIGHT_PHASES_H_

#include "util/types.h"

/* Config */
#define SAMPLING_FREQUENCY 100;

/* MOVING */
#define TIME_THRESHOLD_MOV_TO_IDLE 1000
#define ALLOWED_ACC_ERROR          40
#define ALLOWED_GYRO_ERROR         10

/* IDLE */
#define TIME_THRESHOLD_IDLE_TO_MOV 500
#define LIFTOFF_ACC_THRESHOLD      3072
#define LIFTOFF_ACC_THRESHOLD_SQUARED \
  (LIFTOFF_ACC_THRESHOLD * LIFTOFF_ACC_THRESHOLD)
#define LIFTOFF_SAFETY_COUNTER 10
#define GYRO_SENSITIVITY       0.1f
#define ANGLE_MOVE_MAX         60.0f

/* THRUSTING 1 */
#define COASTING_SAFETY_COUNTER 30

/* COASTING */
#define APOGEE_SAFETY_COUNTER 30

/* APOGEE */
#define PARACHUTE_DESCENT_SPEED  -10
#define PARACHUTE_SAFETY_COUNTER 300
#define BALISTIC_SAFETY_COUNTER  300

/* BALLISTIC AND PARACHUTE */
#define HEIGHT_ERROR_BOUND       3
#define TOUCHDOWN_SAFETY_COUNTER 1000

void check_flight_phase(flight_fsm_t *fsm_state, imu_data_t *imu_data,
                        estimation_output_t *state_data);

#endif /* INC_CONTROL_FLIGHT_PHASES_H_ */
