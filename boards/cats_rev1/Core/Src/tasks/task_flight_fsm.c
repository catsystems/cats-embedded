/*
 * task_flight_fsm.c
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#include "tasks/task_flight_fsm.h"
#include "util.h"

/**
 * @brief Function implementing the task_state_est thread.
 * @param argument: Not used
 * @retval None
 */
void vTaskFlightFSM(void *argument) {
  /* For periodic update */
  uint32_t tick_count, tick_update;

  flight_fsm_e fsm_state = MOVING;

  /* Infinite loop */
  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / FLIGHT_FSM_SAMPLING_FREQ;

  osDelay(5000);

  while (1) {
    tick_count += tick_update;

    osDelayUntil(tick_count);
  }
}
