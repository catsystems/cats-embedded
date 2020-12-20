/*
 * task_state_est.h
 *
 *  Created on: Nov 1, 2019
 *      Author: Jonas
 */

#ifndef INC_TASK_STATE_EST_H_
#define INC_TASK_STATE_EST_H_

/* Includes */
#include "util.h"

/* Constants */
#define STATE_EST_SAMPLING_FREQ 100
#define P_INITIAL               101250
#define GRAVITY                 9.81f

extern baro_data_t global_baro[3];
extern imu_data_t global_imu[3];

/* Tasks */
void vTaskStateEst(void *argument);

#endif /* INC_TASK_STATE_EST_H_ */
