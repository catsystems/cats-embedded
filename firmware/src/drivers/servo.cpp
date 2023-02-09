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

#include "servo.h"

namespace driver {

void Servo::SetPosition(uint16_t ticks) {
  if (ticks > 1000) {
    ticks = 1000U;
  }

  // Get the output depth
  const auto depth = static_cast<float32_t>(m_pwm_channel.GetPwmDepth());

  // Servos operate between 4% and 11% duty cycle -> 7% usage of the pwm depth
  const float32_t single_tick = (depth * 0.07F) / 1000.0F;
  const auto output = static_cast<uint32_t>(single_tick * static_cast<float32_t>(ticks) + 0.04F * depth);
  m_pwm_channel.SetDutyCycleTicks(output);
}

}  // namespace driver
