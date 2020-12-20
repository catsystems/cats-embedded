/*
 * task_baro_read.h
 *
 *  Created on: Nov 1, 2019
 *      Author: Jonas
 */

#ifndef INC_TASK_BARO_READ_H_
#define INC_TASK_BARO_READ_H_

/* Includes */
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "util.h"

/* Constants */
#define BARO_SAMPLING_FREQ 300

/* Extern */
extern baro_data_t global_baro[3];

/* Tasks */
void vTaskBaroRead(void *argument);

#endif /* INC_TASK_BARO_READ_H_ */
