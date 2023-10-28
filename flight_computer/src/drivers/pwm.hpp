/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "target.hpp"

namespace driver {

class Pwm {
 public:
  /** Constructor
   *
   * @param timer Reference to the HAL timer @injected
   * @param channel Channel number
   */
  Pwm(TIM_HandleTypeDef& timer, uint32_t channel) : m_timer(timer), m_channel(channel) {}

  /** Set the frequency of the pwm generator
   *
   * @param frequency the frequency in Hz
   * @return true on success
   */
  bool SetFrequency(uint32_t frequency);

  /** Set the pwm depth, after calling this function, the pwm generation will be stopped and the frequency needs to be
   * set again
   *
   * @param depth pwm depth in ticks
   */
  void SetPwmDepth(uint16_t depth);

  /** Get the pwm depth, the number of possible positions
   *
   * @return pwm depth
   */
  uint16_t GetPwmDepth() const { return m_depth; }

  /** Set the duty cycle ticks, the pwm depth can be set and read out with the get / set pwm depth commands
   *
   * @param dutyCycleTicks duty cycle in pwm ticks
   * @return true on success
   */
  bool SetDutyCycleTicks(uint32_t dutyCycleTicks);

  /** Start the pwm generation
   */
  void Start() {
    m_started = true;
    HAL_TIM_PWM_Start(&m_timer, m_channel);
  }

  /** Stop the pwm generation
   */
  void Stop() {
    m_started = false;
    HAL_TIM_PWM_Stop(&m_timer, m_channel);
  }

 private:
  /** Update the duty cycle ticks
   */
  void UpdateDutyCycleTicks();

  /// Reference to the timer
  TIM_HandleTypeDef& m_timer;
  /// Timer channel number
  uint32_t m_channel;
  /// Timer period
  uint32_t m_period{0U};
  /// Timer duty cycle in ticks
  uint32_t m_duty_cycle_ticks{0U};
  /// Started flag
  bool m_started{false};
  /// Duty cycle depth
  uint16_t m_depth{10000U};
};

}  // namespace driver
