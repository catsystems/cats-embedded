/******************************************************************************
* file    systemParser.h
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

#include <Arduino.h>
#include <ArduinoJson.h>

#define MAX_SYSTEM_FILE_SIZE 1 * 1024

class SystemParser {
 public:
  SystemParser(void);
  bool loadFile(const char* path);
  uint16_t getUsbVid(void);
  uint16_t getUsbPid(void);
  const char* getUsbSerial(void);
  const char* getSsid(void);
  const char* getPassword(void);

 private:
  StaticJsonDocument<MAX_SYSTEM_FILE_SIZE> doc;
  const char* filePath;

  bool saveFile(const char* path = NULL);
};
