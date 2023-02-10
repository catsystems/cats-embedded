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

#include "pwm.hpp"

namespace driver {

bool Pwm::SetFrequency(uint32_t frequency) {
  // Calculate the prescaler to get as close to the pwm depth as possible
  uint32_t psc = SystemCoreClock / (m_depth * frequency);

  // If the frequency requested is too high we can not set the pwm, reduce the PWM depth
  if (psc < 1U) {
    return false;
  }

  // Update the period to reduce prescaler error
  m_period = (SystemCoreClock / (psc * frequency)) - 1;

  // Update timer prescaler and period
  m_timer.Init.Prescaler = psc - 1;
  m_timer.Init.Period = m_period;

  HAL_TIM_PWM_Init(&m_timer);

  // Update pulse width since frequency changed
  UpdateDutyCycleTicks();

  return true;
}

void Pwm::SetPwmDepth(const uint16_t depth) {
  Stop();
  m_depth = depth;
}

bool Pwm::SetDutyCycleTicks(const uint32_t dutyCycleTicks) {
  if (dutyCycleTicks > m_depth) {
    return false;
  }
  m_duty_cycle_ticks = dutyCycleTicks;
  UpdateDutyCycleTicks();
  return true;
}

void Pwm::UpdateDutyCycleTicks() {
  // calculate pulse duration
  uint32_t pulse = static_cast<uint32_t>((static_cast<float32_t>(m_period) / static_cast<float32_t>(m_depth)) *
                                         (static_cast<float32_t>(m_duty_cycle_ticks))) -
                   1U;

  // Setup pwm channel
  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  // Write configuration to hardware
  HAL_TIM_PWM_ConfigChannel(&m_timer, &sConfigOC, m_channel);

  // If the pwm was running before entering, start it again
  if (m_started) {
    Start();
  }
}

}  // namespace driver
