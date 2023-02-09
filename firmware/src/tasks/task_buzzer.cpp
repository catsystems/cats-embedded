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
      char pitch = s_status_codes[id][i];
      if (pitch >= 'A' && pitch <= 'H') {
        m_buzzer.SetFrequency(s_frequency_lookup[pitch - 'A']);
        duration = static_cast<uint32_t>(Duration::kLongBeep);
      } else if (pitch >= 'a' && pitch <= 'h') {
        m_buzzer.SetFrequency(s_frequency_lookup[pitch - 'a']);
        duration = static_cast<uint32_t>(Duration::kShortBeep);
      }
      m_buzzer.Beep(duration);
      osDelay(static_cast<uint32_t>(Duration::kShortPause));
    }

    // Wait at least 1s before buzzing again
    osDelay(static_cast<uint32_t>(Duration::kLongPause));
  }
}

}  // namespace task
