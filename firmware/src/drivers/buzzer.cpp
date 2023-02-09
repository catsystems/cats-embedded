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

#include "buzzer.h"

namespace driver {

void Buzzer::SetVolume(uint16_t volume) {
  const uint16_t pwm_depth = m_pwm_channel.GetPwmDepth();
  // 100% volume = 50% pwm
  const uint32_t pwm_ticks = (pwm_depth / 200U) * volume;
  m_pwm_channel.SetDutyCycleTicks(pwm_ticks);
}

void Buzzer::Beep(uint32_t duration) {
  m_pwm_channel.Start();
  osDelay(duration);
  m_pwm_channel.Stop();
}

}  // namespace driver
