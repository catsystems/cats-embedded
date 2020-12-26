/*
 * task_baro_read.c
 *
 *  Created on: Nov 1, 2019
 *      Author: Jonas
 */

#include "drivers/ms5607.h"
#include "tasks/task_baro_read.h"

#include "tracing/trcRecorder.h"
#include "util/log.h"

#if (configUSE_TRACE_FACILITY == 1)
traceString baro_channel;
#endif

static void init_baro();
static void prepare_temp();
static void prepare_pres();
static void get_temp_pres(int32_t *temperature, int32_t *pressure);
static void read_baro();

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

  init_baro();

#if (configUSE_TRACE_FACILITY == 1)
  baro_channel = xTraceRegisterString("Baro Channel");
#endif

  /* Infinite loop */
  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / BARO_SAMPLING_FREQ;
  while (1) {
    tick_count += tick_update;

    // Phase 1, get the temperature
    prepare_temp();
    osDelay(1);
    read_baro();
    while (ms5607_busy()) {
      osDelay(1);
    }

    // Phase 2, get the pressure
    prepare_pres();
    osDelay(1);
    read_baro();
    while (ms5607_busy()) {
      osDelay(1);
    }
    get_temp_pres(temperature, pressure);

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

void init_baro() {
  ms5607_init(&MS1);
  trace_print(baro_channel, "Init baro 1");
  ms5607_init(&MS2);
  trace_print(baro_channel, "Init baro 2");
  ms5607_init(&MS3);
  trace_print(baro_channel, "Init baro 3");
}

void prepare_temp() {
  trace_print(baro_channel, "Prepare temp 1 start");
  ms5607_prepare_temp(&MS1);
  trace_print(baro_channel, "Prepare temp 1 end");
  trace_print(baro_channel, "Prepare temp 2 start");
  ms5607_prepare_temp(&MS2);
  trace_print(baro_channel, "Prepare temp 2 end");
  trace_print(baro_channel, "Prepare temp 3 start");
  ms5607_prepare_temp(&MS3);
  trace_print(baro_channel, "Prepare temp 3 end");
}

void prepare_pres() {
  trace_print(baro_channel, "Prepare pres 1 start");
  ms5607_prepare_pres(&MS1);
  trace_print(baro_channel, "Prepare pres 1 end");
  trace_print(baro_channel, "Prepare pres 2 start");
  ms5607_prepare_pres(&MS2);
  trace_print(baro_channel, "Prepare pres 2 end");
  trace_print(baro_channel, "Prepare pres 3 start");
  ms5607_prepare_pres(&MS3);
  trace_print(baro_channel, "Prepare pres 3 end");
}

void read_baro() {
  trace_print(baro_channel, "Read baro 1 start");
  int counter = 0;
  while ((ms5607_try_readout(&MS1) == 0) && (MS_TIMEOUT >= counter)) {
    counter++;
    osDelay(1);
  }
  trace_printf(baro_channel, "Read baro 1 end, counter: %d", counter);

  trace_print(baro_channel, "Read baro 2 start");
  counter = 0;
  while ((ms5607_try_readout(&MS2) == 0) && (MS_TIMEOUT >= counter)) {
    counter++;
    osDelay(1);
  }
  trace_printf(baro_channel, "Read baro 2 end, counter: %d", counter);

  trace_print(baro_channel, "Read baro 3 start");
  counter = 0;
  while ((ms5607_try_readout(&MS3) == 0) && (MS_TIMEOUT >= counter)) {
    counter++;
    osDelay(1);
  }
  trace_printf(baro_channel, "Read baro 3 end, counter: %d", counter);
}

void get_temp_pres(int32_t *temperature, int32_t *pressure) {
  trace_print(baro_channel, "Get temp pres 1 start");
  ms5607_get_temp_pres(&MS1, &temperature[0], &pressure[0]);
  trace_print(baro_channel, "Get temp pres 1 end");

  trace_print(baro_channel, "Get temp pres 2 start");
  ms5607_get_temp_pres(&MS2, &temperature[1], &pressure[1]);
  trace_print(baro_channel, "Get temp pres 2 end");

  trace_print(baro_channel, "Get temp pres 3 start");
  ms5607_get_temp_pres(&MS3, &temperature[2], &pressure[2]);
  trace_print(baro_channel, "Get temp pres 3 end");
}
