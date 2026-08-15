/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "systemParser.hpp"

#include <cstdint>
#include <cstring>
#include <string_view>

#include <SdFat.h>

#include "USB.h"
#include "config.hpp"
#include "console.hpp"
#include "utils.hpp"

namespace {

bool isValidPhrase(JsonVariantConst value) {
  if (!value.is<const char*>()) {
    return false;
  }
  const char* phrase = value.as<const char*>();
  return phrase != nullptr && strlen(phrase) <= kMaxPhraseLen;
}

bool isValidUnitSystem(JsonVariantConst value) {
  if (!value.is<const char*>()) {
    return false;
  }
  const auto unit = std::string_view{value.as<const char*>()};
  return unit == unit_map[static_cast<uint8_t>(UnitSystem::kMetric)] ||
         unit == unit_map[static_cast<uint8_t>(UnitSystem::kImperial)];
}

bool hasValidTypes(const JsonDocument& document) {
  // The guard keeps malformed root documents out before iterating their keys.
  if (!document.is<JsonObjectConst>()) {
    return false;  // NOLINT(readability-simplify-boolean-expr)
  }

  for (JsonPairConst pair : document.as<JsonObjectConst>()) {
    const std::string_view key{pair.key().c_str()};
    const JsonVariantConst value = pair.value();

    if (key == "link_phrase_1" || key == "link_phrase_2" || key == "testing_phrase") {
      if (!isValidPhrase(value)) {
        return false;
      }
    } else if (key == "never_stop_logging" || key == "startup_animation" || key == "telemetry_mode") {
      if (!value.is<bool>()) {
        return false;
      }
    } else if (key == "timezone") {
      if (!value.is<int16_t>()) {
        return false;
      }
    } else if (key == "mag_o_x" || key == "mag_o_y" || key == "mag_o_z" || key == "mag_s_x" || key == "mag_s_y" ||
               key == "mag_s_z") {
      if (!value.is<int32_t>()) {
        return false;
      }
    } else if (key == "unit_system" && !isValidUnitSystem(value)) {
      return false;
    }
  }
  return true;
}

bool copyPhrase(const JsonDocument& document, const char* key, char* phrase) {
  if (phrase == nullptr) {
    return false;
  }
  const JsonVariantConst value = document[key];
  if (!isValidPhrase(value)) {
    return false;
  }
  const char* source = value.as<const char*>();
  memcpy(phrase, source, strlen(source) + 1);
  return true;
}

}  // namespace

SystemParser::SystemParser() = default;

/**
 * @brief Load a system configuration file
 *
 * @param path is the path of the file
 * @return true on success
 * @return false on error
 */
bool SystemParser::loadFile(const char* path) {
  if (path == nullptr || path[0] == '\0') {
    console.warning.println("[PARSER] Invalid file path");
    return false;
  }
  filePath = path;
  // NOLINTNEXTLINE(cppcoreguidelines-init-variables) something is wrong with this 'File' type
  File file = fatfs.open(filePath);

  if (!file) {
    console.warning.println("[PARSER] Open file failed");
    return false;
  }

  if (file.size() > MAX_SYSTEM_FILE_SIZE) {
    file.close();
    doc.clear();
    console.warning.println("[PARSER] Configuration file is too large");
    return false;
  }

  doc.clear();
  const DeserializationError error = deserializeJson(doc, file);
  file.close();
  if (error) {
    doc.clear();
    console.warning.printf("[PARSER] Failed to read file, using default configuration: %s\n", error.c_str());
    return false;
  }

  if (doc.overflowed()) {
    doc.clear();
    console.warning.println("[PARSER] Configuration allocation failed");
    return false;
  }

  if (!hasValidTypes(doc)) {
    doc.clear();
    console.warning.println("[PARSER] Configuration has an invalid value type");
    return false;
  }

  return true;
}

// NOLINTBEGIN(readability-convert-member-functions-to-static,readability-simplify-boolean-expr)

bool SystemParser::setNeverStopLoggingFlag(bool flag) {
  doc["never_stop_logging"] = flag;
  return !doc.overflowed();
}

bool SystemParser::setStartupAnimationFlag(bool flag) {
  doc["startup_animation"] = flag;
  return !doc.overflowed();
}

bool SystemParser::setTimeZone(int16_t timezone) {
  doc["timezone"] = timezone;
  return !doc.overflowed();
}

bool SystemParser::setTelemetryMode(bool mode) {
  doc["telemetry_mode"] = mode;
  return !doc.overflowed();
}

bool SystemParser::setLinkPhrase1(const char* phrase) {
  if (phrase == nullptr || strlen(phrase) > kMaxPhraseLen) {
    return false;
  }
  doc["link_phrase_1"] = phrase;
  return !doc.overflowed();
}

bool SystemParser::setLinkPhrase2(const char* phrase) {
  if (phrase == nullptr || strlen(phrase) > kMaxPhraseLen) {
    return false;
  }
  doc["link_phrase_2"] = phrase;
  return !doc.overflowed();
}

bool SystemParser::setTestingPhrase(const char* phrase) {
  if (phrase == nullptr || strlen(phrase) > kMaxPhraseLen) {
    return false;
  }
  doc["testing_phrase"] = phrase;
  return !doc.overflowed();
}

bool SystemParser::setMagCalib(mag_calib_t calib) {
  doc["mag_o_x"] = calib.mag_offset_x;
  doc["mag_o_y"] = calib.mag_offset_y;
  doc["mag_o_z"] = calib.mag_offset_z;

  doc["mag_s_x"] = calib.mag_scale_x;
  doc["mag_s_y"] = calib.mag_scale_y;
  doc["mag_s_z"] = calib.mag_scale_z;

  return !doc.overflowed();
}

bool SystemParser::setUnitSystem(UnitSystem unit_system) {
  const auto unitIndex = static_cast<uint8_t>(unit_system);
  if (unitIndex > static_cast<uint8_t>(UnitSystem::kImperial)) {
    return false;
  }
  doc["unit_system"] = unit_map[unitIndex];
  return !doc.overflowed();
}

bool SystemParser::getLinkPhrase1(char* phrase) { return copyPhrase(doc, "link_phrase_1", phrase); }

bool SystemParser::getLinkPhrase2(char* phrase) { return copyPhrase(doc, "link_phrase_2", phrase); }

bool SystemParser::getTestingPhrase(char* phrase) { return copyPhrase(doc, "testing_phrase", phrase); }

bool SystemParser::getNeverStopLoggingFlag(bool& flag) {
  const JsonVariantConst value = doc["never_stop_logging"];
  if (!value.is<bool>()) {
    return false;
  }
  flag = value.as<bool>();
  return true;
}

bool SystemParser::getStartupAnimationFlag(bool& flag) {
  const JsonVariantConst value = doc["startup_animation"];
  if (!value.is<bool>()) {
    return false;
  }
  flag = value.as<bool>();
  return true;
}

bool SystemParser::getTimeZone(int16_t& timezone) {
  const JsonVariantConst value = doc["timezone"];
  if (!value.is<int16_t>()) {
    return false;
  }
  timezone = value.as<int16_t>();
  return true;
}

bool SystemParser::getTelemetryMode(bool& mode) {
  const JsonVariantConst value = doc["telemetry_mode"];
  if (!value.is<bool>()) {
    return false;
  }
  mode = value.as<bool>();
  return true;
}

bool SystemParser::getMagCalib(mag_calib_t& calib) {
  constexpr const char* keys[] = {"mag_o_x", "mag_o_y", "mag_o_z", "mag_s_x", "mag_s_y", "mag_s_z"};
  // The explicit loop returns on the first incorrectly typed calibration value.
  // NOLINTNEXTLINE(readability-use-anyofallof)
  for (const char* key : keys) {
    if (!doc[key].is<int32_t>()) {
      return false;
    }
  }

  calib.mag_offset_x = doc["mag_o_x"].as<int32_t>();
  calib.mag_offset_y = doc["mag_o_y"].as<int32_t>();
  calib.mag_offset_z = doc["mag_o_z"].as<int32_t>();
  calib.mag_scale_x = doc["mag_s_x"].as<int32_t>();
  calib.mag_scale_y = doc["mag_s_y"].as<int32_t>();
  calib.mag_scale_z = doc["mag_s_z"].as<int32_t>();
  return true;
}

bool SystemParser::getUnitSystem(UnitSystem& unit_system) {
  const JsonVariantConst value = doc["unit_system"];
  if (!isValidUnitSystem(value)) {
    return false;
  }
  if (std::string_view{value.as<const char*>()} ==
      std::string_view{unit_map[static_cast<uint8_t>(UnitSystem::kImperial)]}) {
    unit_system = UnitSystem::kImperial;
  } else {
    unit_system = UnitSystem::kMetric;
  }
  return true;
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
  if (filePath == nullptr || filePath[0] == '\0') {
    console.warning.println("[PARSER] Invalid file path");
    return false;
  }
  if (doc.overflowed()) {
    console.warning.println("[PARSER] Configuration allocation failed");
    return false;
  }
  const auto serializedSize = measureJson(doc);
  if (serializedSize == 0 || serializedSize > MAX_SYSTEM_FILE_SIZE) {
    console.warning.println("[PARSER] Configuration exceeds the file size limit");
    return false;
  }
  if (fatfs.exists(filePath)) {
    if (!fatfs.remove(filePath)) {
      console.warning.println("[PARSER] Could not remove file");
      return false;
    }
  }
  // FILE_WRITE is an SdFat macro composed from signed POSIX flag constants.
  // NOLINTNEXTLINE(hicpp-signed-bitwise)
  auto file = fatfs.open(filePath, FILE_WRITE);
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
