/*
 * sensor_elimination.h
 *
 *  Created on: Dec 20, 2020
 *      Author: Jonas
 */

#pragma once

#include "util/types.h"

/* in m/s^2 */
#define UPPER_BOUND_ACC 320
/* in m/s^2 */
#define LOWER_BOUND_ACC (-320)
/* in Pa */
#define UPPER_BOUND_PRESSURE 200000
/* in Pa */
#define LOWER_BOUND_PRESSURE 10
/* in °C */
#define UPPER_BOUND_TEMPERATURE 100
/* in °C */
#define LOWER_BOUND_TEMPERATURE (-50)
/* maximum times a value is allowed to be the same value before we assume that
 * it is faulty */
#define MAX_NUM_SAME_VALUE_PRESSURE 1000
/* maximum times a value is allowed to be the same value before we assume that
 * it is faulty */
#define MAX_NUM_SAME_VALUE_TEMPERATURE 5000
/* maximum times a value is allowed to be the same value before we assume that
 * it is faulty */
#define MAX_NUM_SAME_VALUE_IMU 100
/* The number of times a value has to be outvoted to count as faulty sensor */
#define MAJ_VOTE_NUM_VALUES 5
/* The error in m/s^2 that has to be between 2 imus and the third to count as
 * faulty imu */
#define MAJ_VOTE_IMU_ERROR 3
/* The error in Pa that has to be between 2 baros and the third to count as
 * faulty baro */
#define MAJ_VOTE_PRESSURE_ERROR 1000
/* The error in °C that has to be between 2 baros and the third to count as
 * faulty baro */
#define MAJ_VOTE_TEMPERATURE_ERROR 20

cats_error_e check_sensors(state_estimation_data_t *data, sensor_elimination_t *elimination);
cats_error_e check_imus_no_faults(state_estimation_data_t *data, sensor_elimination_t *elimination);
cats_error_e check_baros_no_faults(state_estimation_data_t *data, sensor_elimination_t *elimination);
cats_error_e check_imus_1_fault(state_estimation_data_t *data, sensor_elimination_t *elimination);
cats_error_e check_baros_1_fault(state_estimation_data_t *data, sensor_elimination_t *elimination);
