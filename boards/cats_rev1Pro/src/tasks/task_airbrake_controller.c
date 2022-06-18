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

#include "tasks/task_airbrake_controller.h"
#include "util/log.h"
#include "config/globals.h"
#include "control/lqr_controller.h"

_Noreturn void task_airbrake_controller(__attribute__((unused)) void *argument) {

  /* Initialize the control_data struct */
  control_data_t control_data = {0};
  control_init(&control_data);


  /* Infinite loop */
  uint32_t tick_count = osKernelGetTickCount();
  uint32_t tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;

  while (1) {
  /* Tick Update */
  tick_count += tick_update;

  /* Call the Controller */
  compute_control_input(&control_data, global_flight_state.flight_state, &global_estimation_data);
  log_info("Control_Output: %ld", (int32_t)((float)control_data.control_input * 1000));


  /* Sleep */
  osDelayUntil(tick_count);
  }
}