/*
 * sensor_elemination.c
 *
 *  Created on: Dec 20, 2020
 *      Author: Jonas
 */


#include "control/sensor_elemination.h"
#include <stdlib.h>

void check_sensors(state_estimation_data_t *data, sensor_elemination_t *elemination){

	/* Check bounds */
	for(int i = 0; i < 3; i++){

		/* Check IMU */
		if((data->acceleration[i] > UPPER_BOUND_ACC) ||
				(data->acceleration[i] < LOWER_BOUND_ACC)){
			elemination->faulty_imu[i] = 1;
		}

		/* Check Baro */
		if((data->pressure[i] > UPPER_BOUND_PRESSURE) ||
				(data->pressure[i] < LOWER_BOUND_PRESSURE) ||
				(data->temperature[i] > UPPER_BOUND_TEMPERATURE) ||
				(data->temperature[i] < LOWER_BOUND_TEMPERATURE)){
			elemination->faulty_baro[i] = 1;
		}
	}

	/* Check freezing of Sensor */
	for(int i = 0; i < 3; i++){
		/* Acceleration */
		if(data->acceleration[i] == elemination->last_value[i]){
			elemination->num_freeze[i] += 1;
			if(elemination->num_freeze[i] > MAX_NUM_SAME_VALUE){
				elemination->faulty_imu[i] = 1;
			}
		}
		else{
			elemination->last_value[i] = data->acceleration[i];
			elemination->num_freeze[i] = 0;
		}
		/* Pressure */
		if(data->pressure[i] == elemination->last_value[i+3]){
			elemination->num_freeze[i+3] += 1;
			if(elemination->num_freeze[i+3] > MAX_NUM_SAME_VALUE){
				elemination->faulty_baro[i] = 1;
			}
		}
		else{
			elemination->last_value[i+3] = data->pressure[i];
			elemination->num_freeze[i+3] = 0;
		}
		/* Temperature */
		if(data->temperature[i] == elemination->last_value[i+6]){
			elemination->num_freeze[i+6] += 1;
			if(elemination->num_freeze[i+6] > MAX_NUM_SAME_VALUE){
				elemination->faulty_baro[i] = 1;
			}
		}
		else{
			elemination->last_value[i+6] = data->temperature[i];
			elemination->num_freeze[i+6] = 0;
		}
	}

	/* Do Majority Voting */
	float avg[3] = { 0 };

	/* Acceleration */
	avg[0] = abs(data->acceleration[0] - data->acceleration[1]);
	avg[1] = abs(data->acceleration[1] - data->acceleration[2]);
	avg[2] = abs(data->acceleration[0] - data->acceleration[2]);
	if((avg[0] > MAJ_VOTE_IMU_ERROR) && (avg[2] > MAJ_VOTE_IMU_ERROR)){
		/* IMU 0 probably faulty */
		elemination->num_maj_vote[0] += 1;
		if(elemination->num_maj_vote[0] > MAJ_VOTE_NUM_VALUES){
			elemination->faulty_imu[0] = 1;
		}
	}
	else{
		elemination->num_maj_vote[0] = 0;
	}
	if((avg[0] > MAJ_VOTE_IMU_ERROR) && (avg[1] > MAJ_VOTE_IMU_ERROR)){
		/* IMU 1 probably faulty */
		elemination->num_maj_vote[1] += 1;
		if(elemination->num_maj_vote[1] > MAJ_VOTE_NUM_VALUES){
			elemination->faulty_imu[1] = 1;
		}
	}
	else{
		elemination->num_maj_vote[1] = 0;
	}
	if((avg[1] > MAJ_VOTE_IMU_ERROR) && (avg[2] > MAJ_VOTE_IMU_ERROR)){
		/* IMU 2 probably faulty */
		elemination->num_maj_vote[2] += 1;
		if(elemination->num_maj_vote[2] > MAJ_VOTE_NUM_VALUES){
			elemination->faulty_imu[2] = 1;
		}
	}
	else{
		elemination->num_maj_vote[2] = 0;
	}

	/* Barometer */
	avg[0] = abs(data->pressure[0] - data->pressure[1]);
	avg[1] = abs(data->pressure[1] - data->pressure[2]);
	avg[2] = abs(data->pressure[0] - data->pressure[2]);
	if((avg[0] > MAJ_VOTE_PRESSURE_ERROR) && (avg[2] > MAJ_VOTE_PRESSURE_ERROR)){
		/* Baro 0 probably faulty */
		elemination->num_maj_vote[3] += 1;
		if(elemination->num_maj_vote[3] > MAJ_VOTE_NUM_VALUES){
			elemination->faulty_baro[0] = 1;
		}
	}
	else{
		elemination->num_maj_vote[3] = 0;
	}
	if((avg[0] > MAJ_VOTE_PRESSURE_ERROR) && (avg[1] > MAJ_VOTE_PRESSURE_ERROR)){
		/* Baro 1 probably faulty */
		elemination->num_maj_vote[4] += 1;
		if(elemination->num_maj_vote[4] > MAJ_VOTE_NUM_VALUES){
			elemination->faulty_baro[1] = 1;
		}
	}
	else{
		elemination->num_maj_vote[4] = 0;
	}
	if((avg[1] > MAJ_VOTE_PRESSURE_ERROR) && (avg[2] > MAJ_VOTE_PRESSURE_ERROR)){
		/* Baro 2 probably faulty */
		elemination->num_maj_vote[5] += 1;
		if(elemination->num_maj_vote[5] > MAJ_VOTE_NUM_VALUES){
			elemination->faulty_baro[2] = 1;
		}
	}
	else{
		elemination->num_maj_vote[5] = 0;
	}

	/* Temperature */
	avg[0] = abs(data->temperature[0] - data->temperature[1]);
	avg[1] = abs(data->temperature[1] - data->temperature[2]);
	avg[2] = abs(data->temperature[0] - data->temperature[2]);
	if((avg[0] > MAJ_VOTE_TEMPERATURE_ERROR) && (avg[2] > MAJ_VOTE_TEMPERATURE_ERROR)){
		/* Baro 0 probably faulty */
		elemination->num_maj_vote[6] += 1;
		if(elemination->num_maj_vote[6] > MAJ_VOTE_NUM_VALUES){
			elemination->faulty_baro[0] = 1;
		}
	}
	else{
		elemination->num_maj_vote[6] = 0;
	}
	if((avg[0] > MAJ_VOTE_TEMPERATURE_ERROR) && (avg[1] > MAJ_VOTE_TEMPERATURE_ERROR)){
		/* Baro 1 probably faulty */
		elemination->num_maj_vote[7] += 1;
		if(elemination->num_maj_vote[7] > MAJ_VOTE_NUM_VALUES){
			elemination->faulty_baro[1] = 1;
		}
	}
	else{
		elemination->num_maj_vote[7] = 0;
	}
	if((avg[1] > MAJ_VOTE_TEMPERATURE_ERROR) && (avg[2] > MAJ_VOTE_TEMPERATURE_ERROR)){
		/* Baro 2 probably faulty */
		elemination->num_maj_vote[8] += 1;
		if(elemination->num_maj_vote[8] > MAJ_VOTE_NUM_VALUES){
			elemination->faulty_baro[2] = 1;
		}
	}
	else{
		elemination->num_maj_vote[8] = 0;
	}


}
