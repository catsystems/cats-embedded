/*
 * task_baro_read.c
 *
 *  Created on: Nov 1, 2019
 *      Author: Jonas
 */

#include "drivers/ms5607.h"
#include "tasks/task_baro_read.h"
#include "util/log.h"
#include "util/recorder.h"
#include "config/globals.h"

/** Private Constants **/

static const int_fast8_t MS_TIMEOUT = 5;

/** Private Function Declarations **/

static void prepare_temp();
static void prepare_pres();
static void get_temp_pres(int32_t *temperature, int32_t *pressure);
static void read_baro();

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_baro_read thread.
 * @param argument: Not used
 * @retval None
 */
void task_baro_read(void *argument) {
  /* For periodic update */
  uint32_t tick_count, tick_update;
  /* actual measurements from sensor */
  int32_t temperature[3];
  int32_t pressure[3];

  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;

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

      record(BARO0 + i, &(global_baro[i]));
    }

    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/

static void prepare_temp() {
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

static void prepare_pres() {
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

static void read_baro() {
  trace_print(baro_channel, "Read baro 1 start");
  int counter = 0;
  while ((ms5607_try_readout(&MS1) == false) && (MS_TIMEOUT >= counter)) {
    ++counter;
    osDelay(1);
  }
  trace_printf(baro_channel, "Read baro 1 end, counter: %d", counter);

  trace_print(baro_channel, "Read baro 2 start");
  counter = 0;
  while ((ms5607_try_readout(&MS2) == false) && (MS_TIMEOUT >= counter)) {
    ++counter;
    osDelay(1);
  }
  trace_printf(baro_channel, "Read baro 2 end, counter: %d", counter);

  trace_print(baro_channel, "Read baro 3 start");
  counter = 0;
  while ((ms5607_try_readout(&MS3) == false) && (MS_TIMEOUT >= counter)) {
    ++counter;
    osDelay(1);
  }
  trace_printf(baro_channel, "Read baro 3 end, counter: %d", counter);
}

static void get_temp_pres(int32_t *temperature, int32_t *pressure) {
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
