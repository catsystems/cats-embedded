/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

#include "hmi/settings.hpp"

enum ReceiverTelemetryMode_e : bool { SINGLE = false, DUAL = true };

enum class UnitSystem : uint8_t { kMetric = 0, kImperial = 1 };

struct mag_calib_t {
  int32_t mag_offset_x;
  int32_t mag_offset_y;
  int32_t mag_offset_z;
  int32_t mag_scale_x;
  int32_t mag_scale_y;
  int32_t mag_scale_z;
};

// Maximum number of characters for link & test phrases
inline constexpr uint32_t kMaxPhraseLen = 16;

struct systemConfig_t {
  int16_t timeZoneOffset;
  bool neverStopLogging;
  ReceiverTelemetryMode_e receiverMode;
  char linkPhrase1[kMaxPhraseLen + 1];
  char linkPhrase2[kMaxPhraseLen + 1];
  char testingPhrase[kMaxPhraseLen + 1];
  mag_calib_t mag_calib;
  UnitSystem unitSystem;
};

class Config {
 public:
  Config() = default;

  void save();
  void load();

  systemConfig_t config = {};

 private:
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern Config systemConfig;
