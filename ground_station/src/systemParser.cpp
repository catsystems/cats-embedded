/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "systemParser.hpp"

#include <cstdint>

#include <SdFat.h>

#include "USB.h"
#include "config.hpp"
#include "console.hpp"
#include "utils.hpp"

SystemParser::SystemParser() = default;

/**
 * @brief Load a system configuration file
 *
 * @param path is the path of the file
 * @return true on success
 * @return false on error
 */
bool SystemParser::loadFile(const char* path) {
  filePath = path;
  // NOLINTNEXTLINE(cppcoreguidelines-init-variables) something is wrong with this 'File' type
  File file = fatfs.open(filePath);

  if (!file) {
    console.warning.println("[PARSER] Open file failed");
    return false;
  }

  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    file.close();
    console.warning.printf("[PARSER] Failed to read file, using default configuration: %s\n", error.c_str());
    return false;
  }

  file.close();
  return true;
}

// NOLINTBEGIN(readability-convert-member-functions-to-static,readability-simplify-boolean-expr)

bool SystemParser::setNeverStopLoggingFlag(bool flag) {
  doc["never_stop_logging"] = flag;
  return true;
}

bool SystemParser::setTimeZone(int16_t timezone) {
  doc["timezone"] = timezone;
  return true;
}

bool SystemParser::setTelemetryMode(bool mode) {
  doc["telemetry_mode"] = mode;
  return true;
}

bool SystemParser::setLinkPhrase1(const char* phrase) {
  if (phrase == nullptr) {
    return false;
  }
  doc["link_phrase_1"] = phrase;
  return true;
}

bool SystemParser::setLinkPhrase2(const char* phrase) {
  if (phrase == nullptr) {
    return false;
  }
  doc["link_phrase_2"] = phrase;
  return true;
}

bool SystemParser::setTestingPhrase(const char* phrase) {
  if (phrase == nullptr) {
    return false;
  }
  doc["testing_phrase"] = phrase;
  return true;
}

bool SystemParser::setMagCalib(mag_calib_t calib) {
  doc["mag_o_x"] = calib.mag_offset_x;
  doc["mag_o_y"] = calib.mag_offset_y;
  doc["mag_o_z"] = calib.mag_offset_z;

  doc["mag_s_x"] = calib.mag_scale_x;
  doc["mag_s_y"] = calib.mag_scale_y;
  doc["mag_s_z"] = calib.mag_scale_z;

  return true;
}

bool SystemParser::setUnitSystem(UnitSystem unit_system) {
  doc["unit_system"] = unit_map[static_cast<uint8_t>(unit_system)];
  return true;
}

bool SystemParser::getLinkPhrase1(char* phrase) {
  if (doc.containsKey("link_phrase_1") && phrase != nullptr) {
    strncpy(phrase, doc["link_phrase_1"].as<const char*>(), kMaxPhraseLen + 1);
    return true;
  }
  return false;
}

bool SystemParser::getLinkPhrase2(char* phrase) {
  if (doc.containsKey("link_phrase_2") && phrase != nullptr) {
    strncpy(phrase, doc["link_phrase_2"].as<const char*>(), kMaxPhraseLen + 1);
    return true;
  }
  return false;
}

bool SystemParser::getTestingPhrase(char* phrase) {
  if (doc.containsKey("testing_phrase") && phrase != nullptr) {
    strncpy(phrase, doc["testing_phrase"].as<const char*>(), kMaxPhraseLen + 1);
    return true;
  }
  return false;
}

bool SystemParser::getNeverStopLoggingFlag(bool& flag) {
  if (doc.containsKey("never_stop_logging")) {
    flag = doc["never_stop_logging"].as<bool>();
    return true;
  }
  return false;
}

bool SystemParser::getTimeZone(int16_t& timezone) {
  if (doc.containsKey("timezone")) {
    timezone = doc["timezone"].as<int16_t>();
    return true;
  }
  return false;
}

bool SystemParser::getTelemetryMode(bool& mode) {
  if (doc.containsKey("telemetry_mode")) {
    mode = doc["telemetry_mode"].as<bool>();
    return true;
  }
  return false;
}

bool SystemParser::getMagCalib(mag_calib_t& calib) {
  if (doc.containsKey("mag_o_x")) {
    calib.mag_offset_x = doc["mag_o_x"].as<int32_t>();
    calib.mag_offset_y = doc["mag_o_y"].as<int32_t>();
    calib.mag_offset_z = doc["mag_o_z"].as<int32_t>();

    calib.mag_scale_x = doc["mag_s_x"].as<int32_t>();
    calib.mag_scale_y = doc["mag_s_y"].as<int32_t>();
    calib.mag_scale_z = doc["mag_s_z"].as<int32_t>();
    return true;
  }
  return false;
}

bool SystemParser::getUnitSystem(UnitSystem& unit_system) {
  if (doc.containsKey("unit_system")) {
    if (doc["unit_system"].as<const char*>() ==
        std::string_view{unit_map[static_cast<uint8_t>(UnitSystem::kImperial)]}) {
      unit_system = UnitSystem::kImperial;
    } else {
      unit_system = UnitSystem::kMetric;
    }
    return true;
  }
  return false;
}

// NOLINTEND(readability-convert-member-functions-to-static,readability-simplify-boolean-expr)

/**
 * @brief Save the current loaded system config as a file
 *
 * @param path to location
 * @return true on success
 * @return false on error
 */
bool SystemParser::saveFile(const char* path) {
  if (path != nullptr) {
    filePath = path;
  }
  if (fatfs.exists(filePath)) {
    if (!fatfs.remove(filePath)) {
      console.warning.println("[PARSER] Could not remove file");
      return false;
    }
  }
  // NOLINTNEXTLINE(cppcoreguidelines-init-variables) something is wrong with this 'File' type
  File file = fatfs.open(filePath, FILE_WRITE);
  if (!file) {
    console.warning.println("[PARSER] Open file failed");
    return false;
  }
  if (serializeJson(doc, file) == 0) {
    file.close();
    console.warning.println("[PARSER] Failed to write to file");
    return false;
  }
  file.close();
  return true;
}
