/*
 * task_baro_read.c
 *
 *  Created on: Nov 1, 2019
 *      Author: Jonas
 */

#include "tasks/task_state_est.h"

#include <stdio.h>
#include <math.h>


float calculate_height(float pressure_initial, float pressure, float temperature);
/**
 * @brief Function implementing the task_baro_read thread.
 * @param argument: Not used
 * @retval None
 */
void vTaskStateEst(void *argument) {
	/* For periodic update */
	uint32_t tick_count, tick_update;

	state_estimation_data_t state_data = { 0 };
	kalman_filter_t filter;
	filter.t_sampl = 1/(float)(STATE_EST_SAMPLING_FREQ);

	initialize_matrices(&filter);

	/* Infinite loop */
	tick_count = osKernelGetTickCount();
	tick_update = osKernelGetTickFreq() / STATE_EST_SAMPLING_FREQ;

	osDelay(5000);



	while (1) {
		tick_count += tick_update;

		state_data.acceleration[0] = (float)(global_imu[0].acc_z)/(1024);
		state_data.acceleration[1] = (float)(global_imu[1].acc_z)/(1024);
		state_data.acceleration[2] = (float)(global_imu[2].acc_z)/(1024);

		state_data.pressure[0] = (float)(global_baro[0].pressure);
		state_data.pressure[1] = (float)(global_baro[1].pressure);
		state_data.pressure[2] = (float)(global_baro[2].pressure);

		state_data.temperature[0] = (float)(global_baro[0].temperature)/100;
		state_data.temperature[1] = (float)(global_baro[1].temperature)/100;
		state_data.temperature[2] = (float)(global_baro[2].temperature)/100;

		state_data.calculated_AGL[0] = calculate_height(P0, state_data.pressure[0], state_data.temperature[0]);
		state_data.calculated_AGL[1] = calculate_height(P0, state_data.pressure[1], state_data.temperature[1]);
		state_data.calculated_AGL[2] = calculate_height(P0, state_data.pressure[2], state_data.temperature[2]);

		kalman_step(&filter, &state_data);

		UsbPrint("Height %ld: Velocity: %ld \n",
						(int32_t)(filter.x_bar[0]), (int32_t)(filter.x_bar[1]));


		osDelayUntil(tick_count);

	}
}

float calculate_height(float pressure_initial, float pressure, float temperature){

	return ((pow(pressure_initial/pressure,(1/5.257))-1)*(temperature+273.15)/0.0065);
}


