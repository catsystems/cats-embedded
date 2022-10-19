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

/**
 * @brief Get the USB Vender Identifier
 *
 * @return uin16_t with the USB VID
 */
uint16_t SystemParser::getUsbVid(void)
{
  if(doc.containsKey("usb_vid"))
  {
    return (uint16_t)strtol(doc["usb_vid"].as<const char*>(), NULL, 0);
  }
  return -1;
}

/**
 * @brief Get the USB Product Identifier
 *
 * @return uin16_t with the USB PID
 */
uint16_t SystemParser::getUsbPid(void)
{
  if(doc.containsKey("usb_pid"))
  {
    return (uint16_t)strtol(doc["usb_pid"].as<const char*>(), NULL, 0);
  }
  return -1;
}

/**
 * @brief Get the USB Serial Number
 *
 * @return const char* with the serial number as string
 */
const char* SystemParser::getUsbSerial(void)
{
  if(doc.containsKey("usb_serial"))
  {
    return doc["usb_serial"].as<const char*>();
  }
  return "";
}

/**
 * @brief Get the name of the access point
 *
 * @return const char* with the name
 */
const char* SystemParser::getSsid(void)
{
  if(doc.containsKey("ssid"))
  {
    return doc["ssid"].as<const char*>();
  }
  return "";
}

/**
 * @brief get the password of the access point
 *
 * @return const char* with the password
 */
const char* SystemParser::getPassword(void)
{
  if(doc.containsKey("password"))
  {
    return doc["password"].as<const char*>();
  }
  return "";
}

/**
 * @brief Save the current loaded system config as a file
 *
 * @param path to location
 * @return true on success
 * @return false on error
 */
bool SystemParser::saveFile(const char* path) {
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
