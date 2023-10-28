/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "target.hpp"

namespace driver {

class OutputPin {
 public:
  /** Constructor
   *
   * @param port Pointer to the port
   * @param pin Pin number
   */
  OutputPin(GPIO_TypeDef* port, uint16_t pin) : m_port{port}, m_pin{static_cast<uint16_t>(1U << pin)} {
    // TODO actually init the pin here
  }

  /** Set output high */
  void SetHigh() { m_port->BSRR = m_pin; }

  /** Set output low */
  void SetLow() { m_port->BSRR = (m_pin << 16U); }

  /** Toggle output */
  void Toggle() {
    if ((m_port->ODR & m_pin) == m_pin) {
      SetLow();
    } else {
      SetHigh();
    }
  }

 private:
  /// Pointer to the port
  GPIO_TypeDef* m_port;
  /// Pin mask
  const uint16_t m_pin;
};

class InputPin {
 public:
  /** Constructor
   *
   * @param port Pointer to the port
   * @param pin Pin number
   */
  InputPin(GPIO_TypeDef* port, uint16_t pin) : m_port{port}, m_pin{static_cast<uint16_t>(1U << pin)} {
    // TODO actually init the pin here
  }

  /** Get pin status */
  [[nodiscard]] bool GetState() const { return static_cast<bool>(m_port->IDR & m_pin); }

 private:
  /// Pointer to the port
  const GPIO_TypeDef* m_port;
  /// Pin mask
  const uint16_t m_pin;
};

}  // namespace driver
