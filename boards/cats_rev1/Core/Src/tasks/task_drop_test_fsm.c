/*
 * task_flight_fsm.c
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#include "cmsis_os.h"
#include "config/globals.h"
#include "util/log.h"
#include "control/drop_test_phases.h"
#include "tasks/task_drop_test_fsm.h"
#include "config/cats_config.h"

/** Private Constants **/

/** Private Function Declarations **/

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_state_est thread.
 * @param argument: Not used
 * @retval None
 */
void task_drop_test_fsm(void *argument) {
  /* For periodic update */
  uint32_t tick_count, tick_update;

  drop_test_fsm_t fsm_state = {.flight_state = DT_IDLE};
  imu_data_t local_imu = {0};

  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;

  while (1) {
    tick_count += tick_update;
    /* Todo: Do not take that IMU*/
    local_imu = global_imu[1];

    check_drop_test_phase(&fsm_state, &local_imu, &dt_telemetry_trigger);

    global_drop_test_state = fsm_state;

    if (fsm_state.state_changed == 1) {
      log_error("State Changed to %s",
                drop_test_fsm_map[fsm_state.flight_state]);
      flight_state_t flight_state = {
          .ts = osKernelGetTickCount(),
          .flight_or_drop_state.drop_state = fsm_state.flight_state};
      record(FLIGHT_STATE, &flight_state);
    }

    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/
