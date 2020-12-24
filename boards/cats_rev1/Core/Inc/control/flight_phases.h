/*
 * flight_phases.h
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#ifndef INC_CONTROL_FLIGHT_PHASES_H_
#define INC_CONTROL_FLIGHT_PHASES_H_

#include "util/types.h"

/* MOVING */
#define TIME_THRESHOLD_MOV_TO_IDLE 1000
#define ALLOWED_ACC_ERROR          40
#define ALLOWED_GYRO_ERROR         10

/* IDLE */
#define TIME_THRESHOLD_IDLE_TO_MOV 500

void check_flight_phase(flight_fsm_t *fsm_state, imu_data_t *imu_data);

#endif /* INC_CONTROL_FLIGHT_PHASES_H_ */
