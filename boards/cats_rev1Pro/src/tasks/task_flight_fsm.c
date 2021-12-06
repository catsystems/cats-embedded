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

/** Private Constants **/

/** Private Function Declarations **/

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_state_est thread.
 * @param argument: Not used
 * @retval None
 */
_Noreturn void task_flight_fsm(__attribute__((unused)) void *argument) {
  /* For periodic update */
  uint32_t tick_count, tick_update;

  control_settings_t settings = global_cats_config.config.control_settings;

  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;

  float max_v = 0;
  float max_a = 0;
  float max_h = 0;
  trigger_event(EV_MOVING);
  // osDelay(1000);

  while (1) {

    /* Check Flight Phases */
    check_flight_phase(&global_flight_state, &global_SI_data.accel, &global_SI_data.gyro, &global_estimation_data, &settings);

    // Keep track of max speed, velocity and acceleration for flight stats
    if (global_flight_state.flight_state >= THRUSTING_1 && global_flight_state.flight_state <= APOGEE) {
      if (max_v < global_estimation_data.velocity) max_v = global_estimation_data.velocity;
      if (max_a < global_estimation_data.acceleration) max_a = global_estimation_data.acceleration;
      if (max_h < global_estimation_data.height) max_h = global_estimation_data.height;
    }
    if (global_flight_state.state_changed == 1) {
      log_error("State Changed FlightFSM to %s", flight_fsm_map[global_flight_state.flight_state]);
      flight_state_t flight_state = {.ts = osKernelGetTickCount(),
                                     .flight_or_drop_state.flight_state = global_flight_state.flight_state};
      record(FLIGHT_STATE, &flight_state);

      // When we are in any flight state update the flash sector with last
      // flight phase
      if (global_flight_state.flight_state == TOUCHDOWN) {
        // TODO - create a stats file
        //        cs_set_flight_phase(fsm_state.flight_state);
        //        cs_set_max_altitude(max_h);
        //        cs_set_max_velocity(max_v);
        //        cs_set_max_acceleration(max_a);
        //        cs_save();
      }
    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/
