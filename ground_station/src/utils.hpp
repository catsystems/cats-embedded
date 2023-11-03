/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Additional notice:
/// This file was adapted from Florian Baumgartner's ESP32 IoT Framework
/// (https://github.com/FlorianBaumgartner/ESP32_IoT_Framework), released under MIT License.

#pragma once

#include "telemetry/telemetry.hpp"

#include <Arduino.h>
#include <SdFat.h>

constexpr float PI_F = static_cast<float>(PI);

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern FatFileSystem fatfs;

class Utils {
 public:
  explicit Utils() = default;
  bool begin(uint32_t watchdogTimeout = 0, const char *labelName = "DRIVE", bool forceFormat = false);
  static void startBootloader();
  static void startWatchdog(uint32_t seconds);
  static void feedWatchdog();
  static bool isUpdated(bool clearFlag = true);
  static bool isConnected();
  static int32_t getFlashMemoryUsage();
  static void streamUsb(Telemetry *link, uint8_t link_idx);
  static bool format(const char *labelName);
  inline const char *getSerialNumber() { return serial; }

  explicit operator bool() const { return mscReady; }

  template <typename T>
  [[nodiscard]] constexpr static T MetersToFeet(T meters) {
    return static_cast<T>(meters * 3.28084);
  }

 private:
  const char *serial = "0";
  volatile bool mscReady = false;

  static void update(void *pvParameter);
};
