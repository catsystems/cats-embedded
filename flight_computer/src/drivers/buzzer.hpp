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

#include "cmsis_os.h"
#include "drivers/pwm.hpp"
#include "target.hpp"

namespace driver {

class Buzzer {
 public:
  /** Constructor
   *
   * @param pwm_channel reference to the pwm channel @injected
   */
  explicit Buzzer(Pwm &pwm_channel) : m_pwm_channel(pwm_channel) {}

  /** Set the buzzer volume
   *
   * @param volume 0 - 100 %
   */
  void SetVolume(uint16_t volume);

  /** Set the buzzer frequency
   *
   * @param frequency 200 - 10'000 Hz
   */
  void SetFrequency(uint32_t frequency) { m_pwm_channel.SetFrequency(frequency); }

  /** Start beeping
   *
   * @note the Beep returns after the beep duration, freertos delay function is used to not block the rest of the system
   *
   * @param duration beep for specified duration
   */
  void Beep(uint32_t duration);

  /** Stop beeping
   */
  void Stop() { m_pwm_channel.Stop(); }

 private:
  /// Reference to the pwm channel
  Pwm &m_pwm_channel;
};

}  // namespace driver
