/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "buzzer.hpp"

namespace driver {

void Buzzer::SetVolume(uint16_t volume) {
  const uint16_t pwm_depth = m_pwm_channel.GetPwmDepth();
  // Limit volume to 100%
  volume = std::min<uint16_t>(100U, volume);
  // 100% volume = 50% pwm
  const uint32_t pwm_ticks = (static_cast<uint32_t>(pwm_depth) / 200U) * volume;
  m_pwm_channel.SetDutyCycleTicks(pwm_ticks);
}

void Buzzer::Beep(uint32_t duration) {
  m_pwm_channel.Start();
  osDelay(duration);
  m_pwm_channel.Stop();
}

}  // namespace driver
