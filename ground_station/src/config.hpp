#pragma once

#include <cstdint>

typedef enum : bool { SINGLE = 0, DUAL = 1 } ReceiverTelemetryMode_e;

// Maximum number of characters for link & test phrases
inline constexpr uint32_t kMaxPhraseLen = 16;

struct systemConfig_t {
  int16_t timeZoneOffset;
  uint8_t neverStopLogging;
  ReceiverTelemetryMode_e receiverMode;
  char linkPhrase1[kMaxPhraseLen + 1];
  char linkPhrase2[kMaxPhraseLen + 1];
  char testingPhrase[kMaxPhraseLen + 1];
};

class Config {
 public:
  Config() {}

  void save();
  void load();

  systemConfig_t config = {};

 private:
};

extern Config systemConfig;
