/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "drivers/gpio.hpp"
#include "drivers/spi.hpp"
#include "util/log.h"

#include <cstdint>
#include <cstring>
#include <vector>

namespace sensor {

class Lsm6dso32 {
 public:
  /** Constructor
   *
   * @param spi reference the the SPI interface @injected
   * @param cs reference the the chip select output pin @injected
   */
  Lsm6dso32(driver::Spi& spi, driver::OutputPin& cs) : m_spi{spi}, m_cs{cs} { m_cs.SetHigh(); }

  /** Initialize the sensor
   *
   * @return true on success
   */
  [[nodiscard]] bool Init() {
    uint8_t temp = 0U;
    // First check the WHO AM I register to determine if sensor responds
    ReadRegister(static_cast<uint8_t>(Register::kWhoAmI), &temp, 1U);
    if (temp != static_cast<uint8_t>(0x6CU)) {
      return false;
    }

    // Configure Accelerometer
    temp = static_cast<uint8_t>(ImuOdr::kOdr104Hz) | static_cast<uint8_t>(AccelerometerFs::kFs32G);
    WriteRegister(static_cast<uint8_t>(Register::kCtrl1Xl), &temp, 1U);

    // Configure Gyroscope
    temp = static_cast<uint8_t>(ImuOdr::kOdr104Hz) | static_cast<uint8_t>(GyroscopeFs::kFs2000Dps);
    WriteRegister(static_cast<uint8_t>(Register::kCtrl2G), &temp, 1U);

    return true;
  }

  /** Read raw gyroscope data from the sensor
   *
   * @param data pointer to write the raw data to, needs to be of size 3!
   */
  void ReadGyroRaw(int16_t* data) {
    // The bit representation inside the imu is exactly the same order as the int16 data register, therefore we want to
    // reinterpret cast the data
    // NOLINTNEXTLINE
    ReadRegister(static_cast<uint8_t>(Register::kOutXLG), reinterpret_cast<uint8_t*>(data), 6U);
  }

  /** Read raw accelerometer data from the sensor
   *
   * @param data pointer to write the raw data to, needs to be of size 3!
   */
  void ReadAccelRaw(int16_t* data) {
    // The bit representation inside the imu is exactly the same order as the int16 data register, therefore we want to
    // reinterpret cast the data
    // NOLINTNEXTLINE
    ReadRegister(static_cast<uint8_t>(Register::kOutXLA), reinterpret_cast<uint8_t*>(data), 6U);
  }

 private:
  /** Read data from a register of the IMU
   *
   * @param reg register to read
   * @param data pointer to data to store
   * @param length length of read data
   */
  void ReadRegister(const uint8_t reg, uint8_t* const data, const size_t length) {
    // Set the read flag of the register
    uint8_t read_reg = reg | static_cast<uint8_t>(0x80U);
    // Read from the sensor
    m_cs.SetLow();
    m_spi.Transfer(&read_reg, 1U);
    m_spi.Receive(data, length);
    m_cs.SetHigh();
  }

  /** Write to a data register of the IMU
   *
   * @param reg register to write
   * @param data data to write
   * @param length length of write data
   */
  void WriteRegister(const uint8_t reg, const uint8_t* data, const size_t length) {
    // Concatenate the register and the data
    // NOLINTNEXTLINE(cppcoreguidelines-init-variables) linter is confused
    std::vector<uint8_t> concatenated{};
    concatenated.push_back(reg);
    concatenated.insert(concatenated.end(), data, data + length);
    // Transfer the data
    m_cs.SetLow();
    m_spi.Transfer(concatenated.data(), length + 1U);
    m_cs.SetHigh();
  }

  /// Scoped sensor register enum
  enum class Register : uint8_t {
    kWhoAmI = 0x0F,
    kCtrl1Xl = 0x10,
    kCtrl2G = 0x11,
    kOutXLG = 0x22,
    kOutXLA = 0x28,
  };

  /// Scoped sensor output data rate enum
  enum class ImuOdr : uint8_t {
    kOdr1Hz6 = 0xB0,
    kOdr12Hz5 = 0x10,
    kOdr26Hz = 0x20,
    kOdr52Hz = 0x30,
    kOdr104Hz = 0x40,
    kOdr208Hz = 0x50,
    kOdr416Hz = 0x60,
    kOdr833Hz = 0x70,
    kOdr1kHz66 = 0x80,
    kOdr3kHz33 = 0x90,
    kOdr6kHz66 = 0xA0,
  };

  /// Scoped accelerometer full scale enum
  enum class AccelerometerFs : uint8_t {
    kFs4G = 0x00,
    kFs8G = 0x08,
    kFs16G = 0x0C,
    kFs32G = 0x04,
  };

  /// Scoped gyroscope full scale enum
  enum class GyroscopeFs : uint8_t {
    kFs250Dps = 0x00,
    kFs500Dps = 0x04,
    kFs1000Dps = 0x08,
    kFs2000Dps = 0x0C,
  };

  /// Reference to the spi interface
  driver::Spi& m_spi;
  /// Reference to the chip select pin
  driver::OutputPin& m_cs;
};

}  // namespace sensor
