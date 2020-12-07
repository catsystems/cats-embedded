/*
 * task_baro_read.c
 *
 *  Created on: Nov 1, 2019
 *      Author: Jonas
 */

#include "drivers/ms5607.h"
#include "tasks/task_baro_read.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

void vInitBaro();
void vReadBaro(int32_t *temperature, int32_t *pressure);

MS5607 MS = MS5607_INIT();
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

	/* Infinite loop */
	tick_count = osKernelGetTickCount();
	tick_update = osKernelGetTickFreq() / BARO_SAMPLING_FREQ;
	while (1) {
		tick_count += tick_update;
		vReadBaro(&temperature, &pressure);

		UsbPrint("P: %ld; T: %ld; t: %ld\n", pressure,
				temperature, tick_count);

		//TODO HIE AUE STUFF WO MUES GMACHT WERDE MIT DENE DATE

		/* If the Mutex is acquired we write the data into the right variable */
//		if (osMutexAcquire(baro_mutex, BARO_MUTEX_TIMEOUT) == osOK) {
//			baro_data_to_mb.temperature = temperature;
//			baro_data_to_mb.pressure = pressure;
//			baro_data_to_mb.ts = tick_count;
//			osMutexRelease(baro_mutex);
//		}

		osDelayUntil(tick_count);
	}
}

void vInitBaro() {
	ms5607_init(&MS);
}

void vReadBaro(int32_t *temperature, int32_t *pressure) {
	ms5607_read_pres_temp(&MS, temperature, pressure);
}

