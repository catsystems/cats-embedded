/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "drivers/pwm.hpp"
#include "target.hpp"

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
