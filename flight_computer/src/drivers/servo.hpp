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

#pragma once

#include "drivers/pwm.hpp"
#include "target.h"

namespace driver {

class Servo {
 public:
  /** Constructor
   *
   * @param pwm_channel Reference to the pwm channel @injected
   * @param frequency Pwm frequency
   */
  Servo(Pwm& pwm_channel, uint32_t frequency) : m_pwm_channel(pwm_channel) { m_pwm_channel.SetFrequency(frequency); }

  /** Set the servo position in ticks
   *
   * @param ticks 0 - 1000
   */
  void SetPosition(uint16_t ticks);

  /** Start pwm generation for the servo channel
   */
  void Start() { m_pwm_channel.Start(); }

  /** Stop pwm generation for the servo channel
   */
  void Stop() { m_pwm_channel.Stop(); }

  /** Stop the pwm generation and set the pin high
   */
  void SetDigitalOn() {
    // TODO implement this
  }

  /** Stop the pwm generation and set the pin low
   */
  void SetDigitalOff() {
    // TODO implement this
  }

 private:
  /// Reference to the pwm
  Pwm& m_pwm_channel;
};

}  // namespace driver
