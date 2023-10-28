/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "drivers/gpio.hpp"
#include "drivers/spi.hpp"

#include <cstdint>
#include <vector>

namespace sensor {

class Ms5607 {
 public:
  /// Request measurement enum
  enum class Request : uint8_t {
    kPressure = 0x40,
    kTemperature = 0x50,
  };

  /** Constructor
   *
   * @param spi reference the the SPI interface @injected
   * @param cs reference the the chip select output pin @injected
   */
  Ms5607(driver::Spi &spi, driver::OutputPin &cs) : m_spi{spi}, m_cs{cs} { m_cs.SetHigh(); }

  /** Initialize the sensor
   *
   * @return true on success
   */
  [[nodiscard]] bool Init() {
    // Send reset request
    WriteCommand(static_cast<uint8_t>(Command::kReset));
    HAL_Delay(3U);
    // Readout the sensor calibration
    for (uint8_t i = 0U; i < 6U; i++) {
      uint8_t c[2];
      ReadData(static_cast<uint8_t>(Command::kPromRead) + (2U * (i + 1)), c, 2U);
      m_coefficients[i] = (static_cast<uint16_t>(c[0]) << 8U) + static_cast<uint16_t>(c[1]);
    }
    // Check if we received invalid data
    if ((m_coefficients[0] == 0U) || (m_coefficients[0] == UINT16_MAX)) {
      // NOLINTNEXTLINE(readability-simplify-boolean-expr)
      return false;
    }
    return true;
  }

  /** Prepare a new measurement
   *
   * @param req The measurement to perform
   */
  void Prepare(const Request req) {
    m_last_request = req;

    WriteCommand(static_cast<uint8_t>(req) | static_cast<uint8_t>(Osr::kOsr1024));
  }

  /** Readout the requested measurement */
  void Read() {
    if (m_last_request == Request::kPressure) {
      ReadData(static_cast<uint8_t>(Command::kAdcRead), m_pressure, 3U);
    } else {
      ReadData(static_cast<uint8_t>(Command::kAdcRead), m_temperature, 3U);
    }
  }

  /** Get the measurement results
   *
   * @note Only call this fucntion after preparing and reading out both the air pressure and the temperature
   *
   * @param pressure Reference to the pressure
   * @param temperature Reference to the temperature
   */
  void GetMeasurement(int32_t &pressure, int32_t &temperature) {
    const uint32_t d1 = (static_cast<uint32_t>(m_pressure[0]) << 16U) + (static_cast<uint32_t>(m_pressure[1]) << 8U) +
                        static_cast<uint32_t>(m_pressure[2]);
    const auto d2 =
        static_cast<int32_t>((static_cast<uint32_t>(m_temperature[0]) << 16U) +
                             (static_cast<uint32_t>(m_temperature[1]) << 8U) + static_cast<uint32_t>(m_temperature[2]));

    // NOLINTBEGIN(hicpp-signed-bitwise)
    // Calculate compensated temperature
    const int64_t dT = d2 - static_cast<int32_t>(static_cast<uint32_t>(m_coefficients[4]) << 8U);
    temperature = static_cast<int32_t>(2000) + ((dT * static_cast<int64_t>(m_coefficients[5])) >> 23U);

    // Calculate temperature compensated air pressure
    const int64_t off = (static_cast<int64_t>(m_coefficients[1]) << 17U) +
                        (static_cast<int64_t>(m_coefficients[3] * static_cast<int64_t>(dT)) >> 6U);
    const int64_t sens = (static_cast<int64_t>(m_coefficients[0]) << 16U) +
                         (static_cast<int64_t>(m_coefficients[2] * static_cast<int64_t>(dT)) >> 7U);
    // NOLINTEND(hicpp-signed-bitwise)

    pressure = static_cast<int32_t>((((d1 * sens) >> 21U) - off) >> 15U);
  }

 private:
  /** Read data from a register of the IMU
   *
   * @param reg register to read
   * @param data pointer to data to store
   * @param length length of read data
   */
  void ReadData(uint8_t reg, uint8_t *const data, const size_t length) {
    // Read from the sensor
    m_cs.SetLow();
    m_spi.Transfer(&reg, 1U);
    m_spi.Receive(data, length);
    m_cs.SetHigh();
  }

  /** Write to a data register of the IMU
   *
   * @param reg register to write
   * @param data data to write
   * @param length length of write data
   */
  void WriteCommand(uint8_t reg) {
    // Transfer the data
    m_cs.SetLow();
    m_spi.Transfer(&reg, 1U);
    m_cs.SetHigh();
  }

  /// Sensor commands enum
  enum class Command : uint8_t {
    kAdcRead = 0x00,
    kReset = 0x1E,
    kPromRead = 0xA0,
  };

  /// Sensor over sampling rate enum
  enum class Osr : uint8_t {
    kOsr256 = 0x00,
    kOsr512 = 0x02,
    kOsr1024 = 0x04,
    kOsr2048 = 0x06,
    kOsr4096 = 0x08,
  };

  /// Reference to the spi interface
  driver::Spi &m_spi;
  /// Reference to the chip select pin
  driver::OutputPin &m_cs;
  /// The last measurement request
  Request m_last_request{Request::kPressure};
  /// The raw pressure data
  uint8_t m_pressure[3]{};
  /// The raw temperature data
  uint8_t m_temperature[3]{};
  /// The barometer calibration coefficients
  uint16_t m_coefficients[6]{};
};

}  // namespace sensor
