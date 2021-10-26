/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "sensors/ms5607.h"
#include "tasks/task_baro_read.h"
#include "util/log.h"
#include "util/recorder.h"
#include "config/globals.h"

/** Private Constants **/
enum {
  READ_BARO_TEMPERATURE = 1,
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
  int32_t temperature[NUM_BARO];
  int32_t pressure[NUM_BARO];

  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / (2 * CONTROL_SAMPLING_FREQ);
  prepare_temp();
  osDelay(5);
  while (1) {
    tick_count += tick_update;
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
      // log_info("P1: %ld; P2: %ld; P3: %ld; T1: %ld; T2: %ld; T3: %ld", pressure[0], pressure[1], pressure[2],
      // temperature[0], temperature[1], temperature[2]);

      for (int i = 0; i < NUM_BARO; i++) {
        global_baro[i].pressure = pressure[i];
        global_baro[i].temperature = temperature[i];
        global_baro[i].ts = tick_count;

        record(add_id_to_record_type(BARO, i), &(global_baro[i]));
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
  ms5607_get_temp_pres(&MS1, &temperature[0], &pressure[0]);
  ms5607_get_temp_pres(&MS2, &temperature[1], &pressure[1]);
  ms5607_get_temp_pres(&MS3, &temperature[2], &pressure[2]);
}
