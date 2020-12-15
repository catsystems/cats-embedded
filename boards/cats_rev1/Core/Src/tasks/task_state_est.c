/*
 * task_baro_read.c
 *
 *  Created on: Nov 1, 2019
 *      Author: Jonas
 */

#include "tasks/task_state_est.h"



/**
 * @brief Function implementing the task_baro_read thread.
 * @param argument: Not used
 * @retval None
 */
void vTaskStateEst(void *argument) {
	/* For periodic update */
	uint32_t tick_count, tick_update;


	kalman_filter_t filter;
	filter.t_sampl = STATE_EST_SAMPLING_FREQ;

	initialize_matrices(&filter);

	/* Infinite loop */
	tick_count = osKernelGetTickCount();
	tick_update = osKernelGetTickFreq() / STATE_EST_SAMPLING_FREQ;



	while (1) {
		tick_count += tick_update;

	}
}


