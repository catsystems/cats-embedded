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
#define BARO_SAMPLING_FREQ 100

/* Extern */
extern I2C_HandleTypeDef hi2c1;
extern baro_data_t baro_data_to_mb;
extern osMutexId_t baro_mutex;

/* Tasks */
void vTaskBaroRead(void *argument);

#endif /* INC_TASK_BARO_READ_H_ */
