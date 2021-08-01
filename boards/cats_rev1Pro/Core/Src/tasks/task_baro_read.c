/*
 * task_baro_read.c
 *
 *  Created on: Nov 1, 2019
 *      Author: Jonas
 */

#include "sensors/ms5607.h"
#include "tasks/task_baro_read.h"
#include "util/log.h"
#include "util/recorder.h"
#include "config/globals.h"

/** Private Constants **/
enum {	READ_BARO_TEMPERATURE = 1,
		READ_BARO_PRESSURE = 2,
};

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
  uint32_t stage = READ_BARO_TEMPERATURE;
  /* actual measurements from sensor */
  int32_t temperature[3];
  int32_t pressure[3];

  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / (2*CONTROL_SAMPLING_FREQ);
  prepare_temp();
  osDelay(5);
  while (1) {
    tick_count += tick_update;
    // Phase 1, get the temperature

    // Readout the register
    read_baro();

    // Prepare new readout
    if (stage == READ_BARO_TEMPERATURE) {
    	prepare_pres();
    	stage = READ_BARO_PRESSURE;
    } else {
    	prepare_temp();
    	stage = READ_BARO_TEMPERATURE;

    	get_temp_pres(temperature, pressure);
    	for (int i = 0; i < 3; i++) {
    	      global_baro[i].pressure = pressure[i];
    	      global_baro[i].temperature = temperature[i];
    	      global_baro[i].ts = tick_count;

    	      record(BARO0 << i, &(global_baro[i]));
    	}
    }

    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/

static void prepare_temp() {
  ms5607_prepare_temp(&MS1);
  ms5607_prepare_temp(&MS2);
  ms5607_prepare_temp(&MS3);
}
//
static void prepare_pres() {
  ms5607_prepare_pres(&MS1);
  ms5607_prepare_pres(&MS2);
  ms5607_prepare_pres(&MS3);
}

static void read_baro() {
  ms5607_read_raw(&MS1);
  ms5607_read_raw(&MS2);
  ms5607_read_raw(&MS3);
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
