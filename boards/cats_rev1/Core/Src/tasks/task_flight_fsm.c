/*
 * task_flight_fsm.c
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#include "cmsis_os.h"
#include "config/globals.h"
#include "tasks/task_flight_fsm.h"
#include "control/flight_phases.h"

/** Private Constants **/

static const int_fast8_t FLIGHT_FSM_SAMPLING_FREQ = 100;

/** Private Function Declarations **/

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_state_est thread.
 * @param argument: Not used
 * @retval None
 */
void task_flight_fsm(void *argument) {
  /* For periodic update */
  uint32_t tick_count, tick_update;

  flight_fsm_t fsm_state = {.flight_state = MOVING};
  imu_data_t local_imu = {0};

  /* Infinite loop */
  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / FLIGHT_FSM_SAMPLING_FREQ;

  osDelay(1000);

  while (1) {
    tick_count += tick_update;

    local_imu = global_imu[0];

    check_flight_phase(&fsm_state, &local_imu);

    global_flight_state = fsm_state;

    //    usb_print("Phase: %ld Memory: %ld\n", fsm_state.flight_state,
    //    fsm_state.memory);

    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/