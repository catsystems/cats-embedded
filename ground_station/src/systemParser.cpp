/******************************************************************************
 * file    systemParser.cpp
 *******************************************************************************
 * brief   JSON-File Parser for system configuration
 *******************************************************************************
 * author  Florian Baumgartner
 * version 1.0
 * date    2022-08-02
 *******************************************************************************
 * MIT License
 *
 * Copyright (c) 2022 Crelin - Florian Baumgartner
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#include "systemParser.hpp"
#include "USB.h"
#include "config.hpp"
#include "console.hpp"
#include "utils.hpp"

SystemParser::SystemParser(void) {}

/**
 * @brief Load a system configuration file
 *
 * @param path is the path of the file
 * @return true on success
 * @return false on error
 */
bool SystemParser::loadFile(const char* path) {
  filePath = path;
  File file = fatfs.open(filePath);

  if (!file) {
    console.error.println("[PARSER] Open file failed");
    return false;
  }

  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    file.close();
    console.error.printf("[PARSER] Failed to read file, using default configuration: %s\n", error.c_str());
    return false;
  }

  file.close();
  return true;
}

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
  if (phrase == NULL) {
    return false;
  }
  doc["link_phrase_1"] = phrase;
  return true;
}

bool SystemParser::setLinkPhrase2(const char* phrase) {
  if (phrase == NULL) {
    return false;
  }
  doc["link_phrase_2"] = phrase;
  return true;
}

bool SystemParser::setTestingPhrase(const char* phrase) {
  if (phrase == NULL) {
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

/**
 * @brief Save the current loaded system config as a file
 *
 * @param path to location
 * @return true on success
 * @return false on error
 */
bool SystemParser::saveFile(const char* path) {
  console.log.println("[PARSER] Store file");
  if (path != NULL) {
    filePath = path;
  }
  if (fatfs.exists(filePath)) {
    if (!fatfs.remove(filePath)) {
      console.error.println("[PARSER] Could not remove file");
      return false;
    }
  }
  File file = fatfs.open(filePath, FILE_WRITE);
  if (!file) {
    console.error.println("[PARSER] Open file failed");
    return false;
  }
  if (serializeJson(doc, file) == 0) {
    file.close();
    console.error.println("[PARSER] Failed to write to file");
    return false;
  }
  file.close();
  return true;
}
