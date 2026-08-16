/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Adafruit_GFX.h>
#ifndef GS_SIMULATOR_WASM
#include <Adafruit_SharpMem.h>
#endif

#include <cstdint>

// The HMI only depends on this small display contract.  Platform runtimes own
// the concrete display and the transport-specific begin/clear/present calls.
class IDisplay {
 public:
  IDisplay() = default;
  IDisplay(const IDisplay&) = delete;
  IDisplay& operator=(const IDisplay&) = delete;
  IDisplay(IDisplay&&) = delete;
  IDisplay& operator=(IDisplay&&) = delete;
  virtual ~IDisplay() = default;

  virtual Adafruit_GFX& gfx() = 0;
  virtual void begin() = 0;
  virtual void clear() = 0;
  virtual void clearBuffer() = 0;
  virtual void present() = 0;
};

#ifndef GS_SIMULATOR_WASM
class FirmwareDisplay final : public IDisplay {
 public:
  FirmwareDisplay() : display(&SPI, SHARP_SS, 400, 240) {}

  Adafruit_GFX& gfx() override { return display; }
  void begin() override { display.begin(); }
  void clear() override { display.clearDisplay(); }
  void clearBuffer() override { display.clearDisplayBuffer(); }
  void present() override { display.refresh(); }

 private:
  static constexpr uint8_t SHARP_SS = 34;

  Adafruit_SharpMem display;
};
#endif
