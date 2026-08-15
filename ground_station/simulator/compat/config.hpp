#pragma once

#include <cstdint>

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "simulator"
#endif

enum ReceiverTelemetryMode_e : bool { SINGLE = false, DUAL = true };
enum class UnitSystem : uint8_t { kMetric = 0, kImperial = 1 };

struct mag_calib_t {
  int32_t mag_offset_x = 0;
  int32_t mag_offset_y = 0;
  int32_t mag_offset_z = 0;
  int32_t mag_scale_x = 1000;
  int32_t mag_scale_y = 1000;
  int32_t mag_scale_z = 1000;
};

inline constexpr uint32_t kMaxPhraseLen = 16;

struct systemConfig_t {
  int16_t timeZoneOffset = 0;
  bool neverStopLogging = false;
  bool startupAnimation = true;
  ReceiverTelemetryMode_e receiverMode = SINGLE;
  char linkPhrase1[kMaxPhraseLen + 1] = {};
  char linkPhrase2[kMaxPhraseLen + 1] = {};
  char testingPhrase[kMaxPhraseLen + 1] = {};
  mag_calib_t mag_calib{};
  UnitSystem unitSystem = UnitSystem::kMetric;
};

class Config {
 public:
  void save() {}
  void load() {}
  systemConfig_t config{};
};

extern Config systemConfig;
