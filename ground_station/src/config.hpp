#pragma once

#include "systemParser.hpp"

#include <cstdint>

enum ReceiverTelemetryMode_e : bool { SINGLE = false, DUAL = true };

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
