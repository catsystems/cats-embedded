/*
 * task_flight_fsm.h
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#ifndef INC_TASKS_TASK_FLIGHT_FSM_H_
#define INC_TASKS_TASK_FLIGHT_FSM_H_

#include "util/types.h"

#define FLIGHT_FSM_SAMPLING_FREQ 100

extern imu_data_t global_imu[3];
extern flight_fsm_t global_flight_state;

void task_flight_fsm(void *argument);

#endif /* INC_TASKS_TASK_FLIGHT_FSM_H_ */
