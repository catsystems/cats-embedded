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
void vGetTempPres(int32_t *temperature, int32_t *pressure);
void vRead();

#define MS_TIMEOUT 5

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

    // Phase 1, get the temperature
    vPrepareTemp();
    osDelay(1);
    vRead();
    while (ms5607_busy()) {
      osDelay(1);
    }

    // Phase 2, get the pressure
    vPreparePres();
    osDelay(1);
    vRead();
    while (ms5607_busy()) {
      osDelay(1);
    }
    vGetTempPres(temperature, pressure);

    //		UsbPrint("BARO %ld: %ld; T: %ld; t: %ld\n", baro_idx, pressure,
    //				temperature, tick_count);
    for (int i = 0; i < 3; i++) {
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

void vRead() {
  int counter = 0;
  while ((ms5607_try_readout(&MS1) == 0) && (MS_TIMEOUT >= counter)) {
    counter++;
    osDelay(1);
  }
  counter = 0;
  while ((ms5607_try_readout(&MS2) == 0) && (MS_TIMEOUT >= counter)) {
    counter++;
    osDelay(1);
  }
  counter = 0;
  while ((ms5607_try_readout(&MS3) == 0) && (MS_TIMEOUT >= counter)) {
    counter++;
    osDelay(1);
  }
}

void vGetTempPres(int32_t *temperature, int32_t *pressure) {
  ms5607_get_temp_pres(&MS1, &temperature[0], &pressure[0]);
  ms5607_get_temp_pres(&MS2, &temperature[1], &pressure[1]);
  ms5607_get_temp_pres(&MS3, &temperature[2], &pressure[2]);
}
