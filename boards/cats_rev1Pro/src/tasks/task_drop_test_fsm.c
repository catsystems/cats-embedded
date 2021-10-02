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
_Noreturn void task_drop_test_fsm(__attribute__((unused)) void *argument) {
  /* For periodic update */
  uint32_t tick_count, tick_update;

  drop_test_fsm_t fsm_state = {.flight_state = DT_IDLE};
  imu_data_t local_imu;

  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;

  while (1) {
    /* Todo: Do not take that IMU */
    local_imu = global_imu[1];

    check_drop_test_phase(&fsm_state, &local_imu, &dt_telemetry_trigger);

    global_drop_test_state = fsm_state;

    if (fsm_state.state_changed == 1) {
      log_error("State Changed to %s", drop_test_fsm_map[fsm_state.flight_state]);
      flight_state_t flight_state = {.ts = osKernelGetTickCount(),
                                     .flight_or_drop_state.drop_state = fsm_state.flight_state};
      record(FLIGHT_STATE, &flight_state);
    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/
