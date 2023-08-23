/******************************************************************************
 * file    systemParser.hpp
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

#pragma once

#include <ArduinoJson.h>

constexpr uint32_t MAX_SYSTEM_FILE_SIZE = 1 * 1024UL;

struct mag_calib_t {
  int32_t mag_offset_x;
  int32_t mag_offset_y;
  int32_t mag_offset_z;
  int32_t mag_scale_x;
  int32_t mag_scale_y;
  int32_t mag_scale_z;
};

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

  bool getLinkPhrase1(char* phrase);
  bool getLinkPhrase2(char* phrase);
  bool getTestingPhrase(char* phrase);
  bool getNeverStopLoggingFlag(bool& flag);
  bool getTimeZone(int16_t& timezone);
  bool getTelemetryMode(bool& mode);
  bool getMagCalib(mag_calib_t& calib);

  bool saveFile(const char* path = nullptr);

 private:
  StaticJsonDocument<MAX_SYSTEM_FILE_SIZE> doc;
  const char* filePath{nullptr};
};
