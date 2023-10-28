/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "tasks/task_buzzer.hpp"
#include "config/cats_config.hpp"

namespace task {

void Buzzer::Beep(BeepCode code) const {
  // Clear pending event flags
  osEventFlagsClear(m_buzzer_event_id, 0xFFU);
  osEventFlagsSet(m_buzzer_event_id, static_cast<uint32_t>(code));
}

[[noreturn]] void Buzzer::Run() noexcept {
  while (true) {
    // Wait for event
    auto id = osEventFlagsWait(m_buzzer_event_id, 0xFFU, osFlagsWaitAny, osWaitForever);
    osEventFlagsClear(m_buzzer_event_id, id);
    uint32_t duration = 0;
    for (int i = 0; i < s_status_code_length[id]; i++) {
      const char pitch = s_status_codes[id][i];
      if (pitch >= 'A' && pitch <= 'H') {
        m_buzzer.SetFrequency(s_frequency_lookup[pitch - 'A']);
        duration = static_cast<uint32_t>(Duration::kLongBeep);
      } else if (pitch >= 'a' && pitch <= 'h') {
        m_buzzer.SetFrequency(s_frequency_lookup[pitch - 'a']);
        duration = static_cast<uint32_t>(Duration::kShortBeep);
      }
      m_buzzer.Beep(duration);
      osDelay(static_cast<uint32_t>(Duration::kPause));
    }

    // Wait at least kPause ms before buzzing again
    osDelay(static_cast<uint32_t>(Duration::kPause));
  }
}

}  // namespace task
