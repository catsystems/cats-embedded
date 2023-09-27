/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
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

#include "tasks/task_flight_fsm.hpp"
#include "cmsis_os.h"
#include "config/cats_config.hpp"
#include "config/globals.hpp"
#include "control/flight_phases.hpp"
#include "tasks/task_peripherals.hpp"
#include "util/enum_str_maps.hpp"
#include "util/log.h"
#include "util/task_util.hpp"

namespace task {

[[noreturn]] void FlightFsm::Run() noexcept {
  const control_settings_t settings = global_cats_config.control_settings;

  fsm_flag_id = osEventFlagsNew(nullptr);
  osEventFlagsSet(fsm_flag_id, CALIBRATING);

  trigger_event(EV_CALIBRATE);

  flight_fsm_t flight_state = {.flight_state = CALIBRATING};

  uint32_t tick_count = osKernelGetTickCount();
  constexpr uint32_t tick_update = sysGetTickFreq() / CONTROL_SAMPLING_FREQ;
  while (true) {
    /* Check Flight Phases */
    check_flight_phase(&flight_state, m_task_preprocessing.GetSIData().acc, m_task_preprocessing.GetSIData().gyro,
                       m_task_state_estimation.GetEstimationOutput(), &settings);

    if (flight_state.state_changed) {
      log_info("State Changed FlightFSM to %s", GetStr(flight_state.flight_state, fsm_map));
      log_sim("State Changed FlightFSM to %s", GetStr(flight_state.flight_state, fsm_map));
      record(tick_count, FLIGHT_STATE, &flight_state.flight_state);
    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

}  // namespace task
