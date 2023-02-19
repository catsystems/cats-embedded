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

#include "systemParser.h"
#include "console.h"
#include "utils.h"
#include "USB.h"

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

  if(!file)
  {
    console.error.println("[PARSER] Open file failed");
    return false;
  }

  DeserializationError error = deserializeJson(doc, file);
  if(error)
  {
    file.close();
    console.error.printf("[PARSER] Failed to read file, using default configuration: %d\n", error);
    return false;
  }

  file.close();
  return true;
}

bool SystemParser::setNeverStopLoggingFlag(bool flag) {
  doc["never_stop_logging"] = flag;
  return true;
}

bool SystemParser::setTimeZone(int16_t timezone){
  doc["timezone"] = timezone;
  return true;
}

bool SystemParser::setTelemetryMode(bool mode){
  doc["telemetry_mode"] = mode;
  return true;
}

bool SystemParser::setLinkPhrase1(const char* phrase){
  if (phrase == NULL) {
    return false;
  }
  doc["link_phrase_1"] = phrase;
  return true;
}

bool SystemParser::setLinkPhrase2(const char* phrase){
  if (phrase == NULL) {
    return false;
  }
  doc["link_phrase_2"] = phrase;
  return true;
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
  if(path != NULL)
  {
    filePath = path;
  }
  if(fatfs.exists(filePath))
  {
    if(!fatfs.remove(filePath)) 
    {
      console.error.println("[PARSER] Could not remove file");
      return false;
    }
  }
  File file = fatfs.open(filePath, FILE_WRITE);
  if(!file)
  {
    console.error.println("[PARSER] Open file failed");
    return false;
  }
  if(serializeJson(doc, file) == 0)
  {
    file.close();
    console.error.println("[PARSER] Failed to write to file");
    return false;
  }
  file.close();
  return true;
}
