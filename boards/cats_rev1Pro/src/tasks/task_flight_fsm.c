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
#include "tasks/task_flight_fsm.h"
#include "control/flight_phases.h"
#include "config/cats_config.h"
#include "tasks/task_peripherals.h"

/**
 * @brief Function implementing the task_flight_fsm thread.
 * @param argument: Not used
 * @retval None
 */
_Noreturn void task_flight_fsm(__attribute__((unused)) void *argument) {
  const control_settings_t settings = global_cats_config.config.control_settings;

  trigger_event(EV_MOVING);

  uint32_t tick_count = osKernelGetTickCount();
  uint32_t tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;
  while (1) {
    /* Check Flight Phases */
    check_flight_phase(&global_flight_state, &global_SI_data.acc, &global_SI_data.gyro, &global_estimation_data,
                       &settings);

    if (global_flight_state.state_changed) {
      log_error("State Changed FlightFSM to %s", flight_fsm_map[global_flight_state.flight_state]);
      flight_state_t flight_state = {.flight_or_drop_state.flight_state = global_flight_state.flight_state};
      record(tick_count, FLIGHT_STATE, &flight_state);
    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/
