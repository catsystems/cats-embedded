/*
 * task_magneto_read.c
 *
 *  Created on: Jun 30, 2021
 *      Author: jonas
 */

#include "cmsis_os.h"
#include "tasks/task_magneto_read.h"
#include "sensors/mmc5983ma.h"
#include "util/recorder.h"
#include "config/globals.h"
#include "util/log.h"

#include <string.h>

/** Private Constants **/

/** Private Function Declarations **/

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_baro_read thread.
 * @param argument: Not used
 * @retval None
 */
void task_magneto_read(void *argument) {
  uint32_t tick_count, tick_update;

  /* initialize data variables */
  magneto_data_t magneto_data;
  float data[3];

  /* Infinite loop */
  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;

  while (1) {
    tick_count += tick_update;
    mmc5983ma_read_calibrated(&MAG, data);
    magneto_data.magneto_x = data[0];
    magneto_data.magneto_y = data[1];
    magneto_data.magneto_z = data[2];
    magneto_data.ts = osKernelGetTickCount();

    global_magneto = magneto_data;

    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/
