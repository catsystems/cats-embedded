/******************************************************************************
 * file    utils.hpp
 *******************************************************************************
 * brief   General utilities for file system support, MSC, configuration, etc.
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

#include <Arduino.h>
#include <SdFat.h>

constexpr float PI_F = static_cast<float>(PI);

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern FatFileSystem fatfs;

class Utils {
 public:
  explicit Utils() = default;
  bool begin(uint32_t watchdogTimeout = 0, const char *labelName = "DRIVE", bool forceFormat = false);
  static void startBootloader();
  static void startWatchdog(uint32_t seconds);
  static void feedWatchdog();
  static bool isUpdated(bool clearFlag = true);
  static bool isConnected();
  static int32_t getFlashMemoryUsage();
  static bool format(const char *labelName);
  inline const char *getSerialNumber() { return serial; }

  explicit operator bool() const { return mscReady; }

 private:
  const char *serial = "0";
  volatile bool mscReady = false;

  static void update(void *pvParameter);
};
