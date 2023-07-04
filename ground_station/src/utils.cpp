/******************************************************************************
 * file    utils.cpp
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

#include "utils.hpp"
#include "Adafruit_SPIFlash.h"
#include "Adafruit_TinyUSB.h"
#include "SPI.h"
#include "console.hpp"
#include "systemParser.hpp"

#include "diskio.h"
#include "ff.h"

#include "esp_private/system_internal.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void msc_flush_cb(void);
static int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize);
static int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize);
static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static volatile bool updated = false;
static volatile bool connected = false;

Adafruit_USBD_MSC usb_msc;
Adafruit_FlashTransport_ESP32 flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem fatfs;

bool Utils::begin(uint32_t watchdogTimeout, const char *labelName, bool forceFormat) {
  bool status = true;

  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  if (watchdogTimeout > 0) {
    startWatchdog(watchdogTimeout);
  }

  if (!flash.begin()) {
    console.error.println("[UTILS] Could not initialize SPI Flash");
    status = false;
  }
  if (!fatfs.begin(&flash) || forceFormat)  // Check if disk must be formated
  {
    console.log.println("[UTILS] SPI Flash needs to be formatted");
    if (!format(labelName)) {
      console.error.println("[UTILS] Could not format SPI Flash");
      status = false;
    }
  }
  delay(200);

  uint16_t vid = USB_VID;
  uint16_t pid = USB_PID;

  USB.VID(vid);
  USB.PID(pid);
  USB.serialNumber(serial);
  USB.enableDFU();
  USB.productName(USB_PRODUCT);
  USB.manufacturerName(USB_MANUFACTURER);
  USB.onEvent(usbEventCallback);
  USB.begin();

  usb_msc.setID(USB_MANUFACTURER, USB_PRODUCT, FIRMWARE_VERSION);
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb,
                               msc_flush_cb);  // Set callback
  usb_msc.setCapacity(flash.size() / 512,
                      512);  // Set disk size, block size should be 512
                             // regardless of spi flash page size
  usb_msc.begin();

  xTaskCreate(update, "task_utils", 2048, this, 1, NULL);
  delay(200);  // TODO: Check if delay helps
  return status;
}

void Utils::startBootloader(void) {
  const uint16_t APP_REQUEST_UF2_RESET_HINT = 0x11F2;
  esp_reset_reason();
  esp_reset_reason_set_hint((esp_reset_reason_t)APP_REQUEST_UF2_RESET_HINT);
  esp_restart();
}

void Utils::startWatchdog(uint32_t seconds) {
  esp_task_wdt_init(seconds, true);  // Enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);            // Add current thread to WDT watch
  esp_task_wdt_reset();
  console.log.printf("[SYSTEM] Watchdog started with timeout of %lu s\n", seconds);
}

void Utils::feedWatchdog(void) { esp_task_wdt_reset(); }

void Utils::update(void *pvParameter) {
  Utils *ref = (Utils *)pvParameter;

  TickType_t mscTimer = 0;
  bool connectedOld = false;
  while (true) {
    TickType_t task_last_tick = xTaskGetTickCount();

    connected = connected && USB;
    if (connected && !connectedOld)  // Check if USB Host is connected
    {
      mscTimer = xTaskGetTickCount() + MSC_STARTUP_DELAY;
    }
    if ((!connected && connectedOld) ||
        (xTaskGetTickCount() > mscTimer))  // USB connected (after delay) or just disconnected
    {
      ref->mscReady = connected;            // Make sure that mscReady can only be set if
                                            // USB Host is connected
      usb_msc.setUnitReady(ref->mscReady);  // Set MSC ready for read/write
                                            // (shows drive to host PC)
      console.enable(connected);
      mscTimer = 0;
    }
    connectedOld = connected;

    vTaskDelayUntil(&task_last_tick, (const TickType_t)1000 / TASK_UTILS_FREQ);
  }
  vTaskDelete(NULL);
}

bool Utils::isUpdated(bool clearFlag) {
  bool status = updated;
  if (clearFlag) updated = false;
  return status;
}

bool Utils::isConnected(void) { return connected; }

bool Utils::format(const char *labelName) {
  static FATFS elmchanFatfs;
  static uint8_t workbuf[4096];  // Working buffer for f_fdisk function.

  console.log.println("[UTILS] Partitioning flash with 1 primary partition...");
  static DWORD plist[] = {100, 0, 0, 0};  // 1 primary partition with 100% of space.
  static uint8_t buf[512] = {0};          // Working buffer for f_fdisk function.
  static FRESULT r = f_fdisk(0, plist,
                             buf);  // Partition the flash with 1 partition that takes the entire space.
  if (r != FR_OK) {
    console.error.printf("[UTILS] Error, f_fdisk failed with error code: %d\n", r);
    return 0;
  }
  console.println("[UTILS] Partitioned flash!");
  console.println(
      "[UTILS] Creating and formatting FAT filesystem (this takes "
      "~60 seconds)...");
  r = f_mkfs("", FM_FAT | FM_SFD, 0, workbuf,
             sizeof(workbuf));  // Make filesystem.
  if (r != FR_OK) {
    console.error.printf("[UTILS] Error, f_mkfs failed with error code: %d\n", r);
    return 0;
  }

  r = f_mount(&elmchanFatfs, "0:", 1);  // mount to set disk label
  if (r != FR_OK) {
    console.error.printf("[UTILS] Error, f_mount failed with error code: %d\n", r);
    return 0;
  }
  console.log.printf("[UTILS] Setting disk label to: %s\n", labelName);
  r = f_setlabel(labelName);  // Setting label
  if (r != FR_OK) {
    console.error.printf("[UTILS] Error, f_setlabel failed with error code: %d\n", r);
    return 0;
  }
  f_unmount("0:");     // unmount
  flash.syncBlocks();  // sync to make sure all data is written to flash
  console.ok.println("[UTILS] Formatted flash!");
  if (!fatfs.begin(&flash))  // Check new filesystem
  {
    console.error.println("[UTILS] Error, failed to mount newly formatted filesystem!");
    return 0;
  }
  console.ok.println("[UTILS] Flash chip successfully formatted with new empty filesystem!");
  yield();
  return 1;
}

int32_t Utils::getFlashMemoryUsage() {
  uint32_t num_clusters = fatfs.clusterCount() - 2;
  uint32_t available_clusters = fatfs.freeClusterCount();

  double percentage = 100 - (static_cast<double>(available_clusters) / num_clusters) * 100;

  return static_cast<int32_t>(std::ceil(percentage));
}

void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  if (event_base == ARDUINO_USB_EVENTS) {
    // arduino_usb_event_data_t *data = (arduino_usb_event_data_t *)event_data;
    if (event_id == ARDUINO_USB_STARTED_EVENT || event_id == ARDUINO_USB_RESUME_EVENT) {
      connected = true;
    }
    if (event_id == ARDUINO_USB_STOPPED_EVENT || event_id == ARDUINO_USB_SUSPEND_EVENT) {
      connected = false;
    }
  }
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
static int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
  return flash.readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
static int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  return flash.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and
// accepted by host). Used to flush any pending cache.
static void msc_flush_cb(void) {
  flash.syncBlocks();  // sync with flash
  fatfs.cacheClear();  // clear file system's cache to force refresh
  updated = true;
}

//--------------------------------------------------------------------+
// fatfs diskio
//--------------------------------------------------------------------+
extern "C" {
DSTATUS disk_status(BYTE pdrv) {
  (void)pdrv;
  return 0;
}

DSTATUS disk_initialize(BYTE pdrv) {
  (void)pdrv;
  return 0;
}

DRESULT disk_read(BYTE pdrv,     // Physical drive nmuber to identify the drive
                  BYTE *buff,    // Data buffer to store read data
                  DWORD sector,  // Start sector in LBA
                  UINT count     // Number of sectors to read
) {
  (void)pdrv;
  return flash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv,         // Physical drive nmuber to identify the drive
                   const BYTE *buff,  // Data to be written
                   DWORD sector,      // Start sector in LBA
                   UINT count         // Number of sectors to write
) {
  (void)pdrv;
  return flash.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv,  // Physical drive nmuber (0..)
                   BYTE cmd,   // Control code
                   void *buff  // Buffer to send/receive control data
) {
  (void)pdrv;

  switch (cmd) {
    case CTRL_SYNC:
      flash.syncBlocks();
      return RES_OK;

    case GET_SECTOR_COUNT:
      *((DWORD *)buff) = flash.size() / 512;
      return RES_OK;

    case GET_SECTOR_SIZE:
      *((WORD *)buff) = 512;
      return RES_OK;

    case GET_BLOCK_SIZE:
      *((DWORD *)buff) = 8;  // erase block size in units of sector size
      return RES_OK;

    default:
      return RES_PARERR;
  }
}
}