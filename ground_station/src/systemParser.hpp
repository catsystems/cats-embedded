/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <ArduinoJson.h>

#include "config.hpp"

constexpr uint32_t MAX_SYSTEM_FILE_SIZE = 1 * 1024UL;

class SystemParser {
 public:
  SystemParser();
  bool loadFile(const char* path);

  bool setLinkPhrase1(const char* phrase);
  bool setLinkPhrase2(const char* phrase);
  bool setTestingPhrase(const char* phrase);
  bool setNeverStopLoggingFlag(bool flag);
  bool setTimeZone(int16_t timezone);
  bool setTelemetryMode(bool mode);
  bool setMagCalib(mag_calib_t calib);
  bool setUnitSystem(UnitSystem unit_system);

  bool getLinkPhrase1(char* phrase);
  bool getLinkPhrase2(char* phrase);
  bool getTestingPhrase(char* phrase);
  bool getNeverStopLoggingFlag(bool& flag);
  bool getTimeZone(int16_t& timezone);
  bool getTelemetryMode(bool& mode);
  bool getMagCalib(mag_calib_t& calib);
  bool getUnitSystem(UnitSystem& unit_system);

  bool saveFile(const char* path = nullptr);

 private:
  StaticJsonDocument<MAX_SYSTEM_FILE_SIZE> doc;
  const char* filePath{nullptr};
};
