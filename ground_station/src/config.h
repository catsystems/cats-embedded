#pragma once

#include <cstdint>

typedef enum : bool { SINGLE = 0, DUAL = 1 } ReceiverTelemetryMode_e;

struct systemConfig_t {
  int16_t timeZoneOffset;
  uint8_t neverStopLogging;
  ReceiverTelemetryMode_e receiverMode;
  char linkPhrase1[9];
  char linkPhrase2[9];
  char testingPhrase[9];
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
