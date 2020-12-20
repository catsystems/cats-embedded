/*
 * sensor_elemination.h
 *
 *  Created on: Dec 20, 2020
 *      Author: Jonas
 */

#ifndef INC_CONTROL_SENSOR_ELEMINATION_H_
#define INC_CONTROL_SENSOR_ELEMINATION_H_

#include "util.h"


#define UPPER_BOUND_ACC 320					/* in m/s^2 */
#define LOWER_BOUND_ACC	-320				/* in m/s^2 */
#define UPPER_BOUND_PRESSURE 200000			/* in Pa */
#define LOWER_BOUND_PRESSURE 10000			/* in Pa */
#define UPPER_BOUND_TEMPERATURE 100			/* in °C */
#define LOWER_BOUND_TEMPERATURE -50			/* in °C */
#define MAX_NUM_SAME_VALUE 100				/* maximum times a value is allowed to be the same value before we assume that it is faulty */
#define MAJ_VOTE_NUM_VALUES 100				/* The number of times a value has to be outvoted to count as faulty sensor */
#define MAJ_VOTE_IMU_ERROR 10				/* The error in m/s^2 that has to be between 2 imu's and the third to count as faulty imu */
#define MAJ_VOTE_PRESSURE_ERROR 10000			/* The error in Pa that has to be between 2 baros and the third to count as faulty baro */
#define MAJ_VOTE_TEMPERATURE_ERROR 10		/* The error in °C that has to be between 2 baros and the third to count as faulty baro */



void check_sensors(state_estimation_data_t *data, sensor_elemination_t *elemination);

#endif /* INC_CONTROL_SENSOR_ELEMINATION_H_ */
