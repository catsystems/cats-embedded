/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

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
