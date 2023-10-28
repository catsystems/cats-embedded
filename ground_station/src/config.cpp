
/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "config.hpp"

#include "console.hpp"
#include "systemParser.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
SystemParser systemParser{};
Config systemConfig{};
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

void Config::save() {
  systemParser.setTestingPhrase(config.testingPhrase);
  systemParser.setLinkPhrase1(config.linkPhrase1);
  systemParser.setLinkPhrase2(config.linkPhrase2);
  systemParser.setTelemetryMode(static_cast<bool>(config.receiverMode));
  systemParser.setNeverStopLoggingFlag(config.neverStopLogging);
  systemParser.setTimeZone(config.timeZoneOffset);
  systemParser.setMagCalib(config.mag_calib);
  systemParser.saveFile("/config.json");
}

void Config::load() {
  systemParser.loadFile("/config.json");
  console.log.println("Load config file");
  bool mode{false};
  bool stop{false};
  if (!systemParser.getTestingPhrase(config.testingPhrase)) {
    strncpy(config.testingPhrase, "", 1);
    console.error.println("Failed");
  } else {
    console.log.println(config.testingPhrase);
  }
  if (!systemParser.getLinkPhrase1(config.linkPhrase1)) {
    strncpy(config.linkPhrase1, "", 1);
    console.error.println("Failed");
  } else {
    console.log.println(config.linkPhrase1);
  }
  if (!systemParser.getLinkPhrase2(config.linkPhrase2)) {
    strncpy(config.linkPhrase2, "", 1);
    console.error.println("Failed");
  } else {
    console.log.println(config.linkPhrase2);
  }
  if (!systemParser.getTelemetryMode(mode)) {
    mode = false;
  } else {
    console.log.println(static_cast<uint32_t>(mode));
  }
  if (!systemParser.getNeverStopLoggingFlag(stop)) {
    config.neverStopLogging = false;
  } else {
    console.log.println(static_cast<uint32_t>(config.neverStopLogging));
  }
  if (!systemParser.getTimeZone(config.timeZoneOffset)) {
    config.timeZoneOffset = 0;
  } else {
    console.log.println(config.timeZoneOffset);
  }
  if (!systemParser.getMagCalib(config.mag_calib)) {
    config.mag_calib.mag_offset_x = 0;
    config.mag_calib.mag_offset_y = 0;
    config.mag_calib.mag_offset_z = 0;

    config.mag_calib.mag_scale_x = 1000;
    config.mag_calib.mag_scale_y = 1000;
    config.mag_calib.mag_scale_z = 1000;
  } else {
    console.log.println(config.timeZoneOffset);
  }

  config.neverStopLogging = stop;
  config.receiverMode = static_cast<ReceiverTelemetryMode_e>(mode);
}
