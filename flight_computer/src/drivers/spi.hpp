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

class Spi {
 public:
  explicit Spi(SPI_HandleTypeDef* spi_handle, uint32_t timeout = 5U) : m_spi_handle(spi_handle), m_timeout(timeout) {}

  void Transfer(uint8_t* data, const size_t length) { HAL_SPI_Transmit(m_spi_handle, data, length, m_timeout); }

  void Receive(uint8_t* data, const size_t length) { HAL_SPI_Receive(m_spi_handle, data, length, m_timeout); }

 private:
  /// Pointer to the SPI handle
  SPI_HandleTypeDef* const m_spi_handle;
  /// SPI transfer timeout
  const uint32_t m_timeout;
};

}  // namespace driver
