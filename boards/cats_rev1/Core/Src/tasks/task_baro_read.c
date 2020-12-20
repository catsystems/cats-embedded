/*
 * task_baro_read.c
 *
 *  Created on: Nov 1, 2019
 *      Author: Jonas
 */

#include "drivers/ms5607.h"
#include "tasks/task_baro_read.h"

void vInitBaro();
void vPrepareTemp();
void vPreparePres();
void vReadPres(int32_t *pressure);
void vReadTemp(int32_t *temperature);


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
  int32_t temperature[3];
  int32_t pressure[3];

  vInitBaro();

  /* Infinite loop */
  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / BARO_SAMPLING_FREQ;
  while (1) {
    tick_count += tick_update;
    vPrepareTemp();
    osDelay(1);
    vReadTemp(temperature);
    vPreparePres();
    osDelay(1);
    vReadPres(pressure);


    //		UsbPrint("BARO %ld: %ld; T: %ld; t: %ld\n", baro_idx, pressure,
    //				temperature, tick_count);
    for(int i = 0; i < 3; i++){
    	global_baro[i].pressure = pressure[i];
		global_baro[i].temperature = temperature[i];
		global_baro[i].ts = tick_count;
    }

    osDelayUntil(tick_count);
  }
}

void vInitBaro() {
  ms5607_init(&MS1);
  ms5607_init(&MS2);
  ms5607_init(&MS3);
}

void vPrepareTemp() {
	ms5607_prepare_temp(&MS1);
	ms5607_prepare_temp(&MS2);
	ms5607_prepare_temp(&MS3);
}

void vPreparePres() {
	ms5607_prepare_pres(&MS1);
	ms5607_prepare_pres(&MS2);
	ms5607_prepare_pres(&MS3);
}

void vReadPres(int32_t *pressure){
	ms5607_read_pres(&MS1, &pressure[0]);
	ms5607_read_pres(&MS2, &pressure[1]);
	ms5607_read_pres(&MS3, &pressure[2]);
}

void vReadTemp(int32_t *temperature){
	ms5607_read_temp(&MS1, &temperature[0]);
	ms5607_read_temp(&MS2, &temperature[1]);
	ms5607_read_temp(&MS3, &temperature[2]);
}


