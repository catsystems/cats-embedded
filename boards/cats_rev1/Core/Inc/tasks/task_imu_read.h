/*
 * task_imu_read.h
 *
 *  Created on: Nov 3, 2019
 *      Author: Jonas
 */

#ifndef INC_TASK_IMU_READ_H_
#define INC_TASK_IMU_READ_H_

/* Includes */
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "util.h"

/* Constants */
#define IMU20601_SAMPLING_FREQ 300

extern imu_data_t global_imu[3];

#endif /* INC_TASK_IMU_READ_H_ */
