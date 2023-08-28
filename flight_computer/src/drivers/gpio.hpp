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
