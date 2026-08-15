/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

class IClock {
 public:
  IClock() = default;
  IClock(const IClock&) = delete;
  IClock& operator=(const IClock&) = delete;
  IClock(IClock&&) = delete;
  IClock& operator=(IClock&&) = delete;
  virtual ~IClock() = default;
  [[nodiscard]] virtual uint64_t nowMs() const = 0;
  [[nodiscard]] virtual uint8_t hour() const { return 0; }
  [[nodiscard]] virtual uint8_t minute() const { return 0; }
};

#ifndef GS_SIMULATOR_WASM
#include <Arduino.h>
#include <TimeLib.h>

class FirmwareClock final : public IClock {
 public:
  [[nodiscard]] uint64_t nowMs() const override { return millis(); }
  [[nodiscard]] uint8_t hour() const override { return static_cast<uint8_t>(::hour()); }
  [[nodiscard]] uint8_t minute() const override { return static_cast<uint8_t>(::minute()); }
};
#endif
