/*
 * task_baro_read.c
 *
 *  Created on: Nov 1, 2019
 *      Author: Jonas
 */

#include "drivers/ms5607.h"
#include "tasks/task_baro_read.h"

void vInitBaro();
void vReadBaro(int32_t *temperature, int32_t *pressure, int32_t id);

MS5607 MS1 = MS5607_INIT1();
MS5607 MS2 = MS5607_INIT2();
MS5607 MS3 = MS5607_INIT3();
/**
 * @brief Function implementing the task_baro_read thread.
 * @param argument: Not used
 * @retval None
 */
void vTaskBaroRead(void *argument) {
	/* For periodic update */
	uint32_t tick_count, tick_update;
	/* actual measurements from sensor */
	int32_t temperature;
	int32_t pressure;

	vInitBaro();
	int32_t baro_idx = 0;

	/* Infinite loop */
	tick_count = osKernelGetTickCount();
	tick_update = osKernelGetTickFreq() / BARO_SAMPLING_FREQ;
	while (1) {
		tick_count += tick_update;
		vReadBaro(&temperature, &pressure, baro_idx);

//		UsbPrint("BARO %ld: %ld; T: %ld; t: %ld\n", baro_idx, pressure,
//				temperature, tick_count);


		global_baro[baro_idx].pressure = pressure;
		global_baro[baro_idx].temperature = temperature;
		global_baro[baro_idx].ts = tick_count;

		baro_idx = (baro_idx + 1) % 3;
		osDelayUntil(tick_count);
	}
}

void vInitBaro() {
	ms5607_init(&MS1);
	ms5607_init(&MS2);
	ms5607_init(&MS3);
}

void vReadBaro(int32_t *temperature, int32_t *pressure, int32_t id) {
	switch(id){
		case 0:
			ms5607_read_pres_temp(&MS1, temperature, pressure);
		break;
		case 1:
			ms5607_read_pres_temp(&MS2, temperature, pressure);
		break;
		case 2:
			ms5607_read_pres_temp(&MS3, temperature, pressure);
		break;
		default:
		break;
	}

}

