/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

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
