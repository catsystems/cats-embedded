/*
 * task_baro_read.h
 *
 *  Created on: Nov 1, 2019
 *      Author: Jonas
 */

#ifndef INC_TASK_BARO_READ_H_
#define INC_TASK_BARO_READ_H_

/* Includes */
#include "util/types.h"

/* Constants */
#define BARO_SAMPLING_FREQ 100

/* Tasks */
void task_baro_read(void *argument);

#endif /* INC_TASK_BARO_READ_H_ */
