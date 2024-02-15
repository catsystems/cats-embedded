/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Additional notice:
/// This file was adapted from Florian Baumgartner's ESP32 IoT Framework
/// (https://github.com/FlorianBaumgartner/ESP32_IoT_Framework), released under MIT License.

/// General utilities for file system support, MSC, configuration, etc.

#include "utils.hpp"
#include "Adafruit_SPIFlash.h"
#include "Adafruit_TinyUSB.h"
#include "SPI.h"
#include "console.hpp"
#include "systemParser.hpp"

#include "diskio.h"
#include "ff.h"

#include <esp_private/system_internal.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

constexpr uint8_t BOOT_BUTTON = 0;
constexpr uint8_t TASK_UTILS_FREQ = 5;        // [Hz]
constexpr uint16_t MSC_STARTUP_DELAY = 2000;  // [ms]

static void msc_flush_cb();
static int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize);
static int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize);
static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static volatile bool updated = false;
static volatile bool connected = false;
Adafruit_USBD_MSC usb_msc;
Adafruit_FlashTransport_ESP32 flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem fatfs;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

bool Utils::begin(uint32_t watchdogTimeout, const char *labelName, bool forceFormat) {
  bool status = true;

  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  if (watchdogTimeout > 0) {
    startWatchdog(watchdogTimeout);
  }

  if (!flash.begin()) {
    console.warning.println("[UTILS] Could not initialize SPI Flash");
    status = false;
  }
  if (!fatfs.begin(&flash) || forceFormat)  // Check if disk must be formated
  {
    if (!format(labelName)) {
      console.warning.println("[UTILS] Could not format SPI Flash");
      status = false;
    }
  }
  delay(200);

  const uint16_t vid = USB_VID;
  const uint16_t pid = USB_PID;

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

  xTaskCreate(update, "task_utils", 2048, this, 1, nullptr);
  delay(200);  // TODO: Check if delay helps
  return status;
}

void Utils::startBootloader() {
  const uint16_t APP_REQUEST_UF2_RESET_HINT = 0x11F2;
  esp_reset_reason();
  esp_reset_reason_set_hint(static_cast<esp_reset_reason_t>(APP_REQUEST_UF2_RESET_HINT));
  esp_restart();
}

void Utils::startWatchdog(uint32_t seconds) {
  esp_task_wdt_init(seconds, true);  // Enable panic so ESP32 restarts
  esp_task_wdt_add(nullptr);         // Add current thread to WDT watch
  esp_task_wdt_reset();
}

void Utils::feedWatchdog() { esp_task_wdt_reset(); }

void Utils::update(void *pvParameter) {
  auto *ref = static_cast<Utils *>(pvParameter);

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

    vTaskDelayUntil(&task_last_tick, static_cast<TickType_t>(1000) / TASK_UTILS_FREQ);
  }
  vTaskDelete(nullptr);
}

bool Utils::isUpdated(bool clearFlag) {
  const bool status = updated;
  if (clearFlag) {
    updated = false;
  }
  return status;
}

bool Utils::isConnected() { return connected; }

bool Utils::format(const char *labelName) {
  static FATFS elmchanFatfs;
  static uint8_t workbuf[4096];  // Working buffer for f_fdisk function.

  static DWORD plist[] = {100, 0, 0, 0};  // 1 primary partition with 100% of space.
  static uint8_t buf[512] = {0};          // Working buffer for f_fdisk function.
  static FRESULT r = f_fdisk(0, plist,
                             buf);  // Partition the flash with 1 partition that takes the entire space.
  if (r != FR_OK) {
    console.warning.printf("[UTILS] Error, f_fdisk failed with error code: %d\n", r);
    return false;
  }
  // NOLINTNEXTLINE(hicpp-signed-bitwise)
  r = f_mkfs("", FM_FAT | FM_SFD, 0, workbuf,
             sizeof(workbuf));  // Make filesystem.
  if (r != FR_OK) {
    console.warning.printf("[UTILS] Error, f_mkfs failed with error code: %d\n", r);
    return false;
  }

  r = f_mount(&elmchanFatfs, "0:", 1);  // mount to set disk label
  if (r != FR_OK) {
    console.warning.printf("[UTILS] Error, f_mount failed with error code: %d\n", r);
    return false;
  }

  r = f_setlabel(labelName);  // Setting label
  if (r != FR_OK) {
    console.warning.printf("[UTILS] Error, f_setlabel failed with error code: %d\n", r);
    return false;
  }
  f_unmount("0:");           // unmount
  flash.syncBlocks();        // sync to make sure all data is written to flash
  if (!fatfs.begin(&flash))  // Check new filesystem
  {
    console.warning.println("[UTILS] Error, failed to mount newly formatted filesystem!");
    return false;
  }
  yield();
  return true;
}

int32_t Utils::getFlashMemoryUsage() {
  const uint32_t num_clusters = fatfs.clusterCount() - 2;
  const uint32_t available_clusters = fatfs.freeClusterCount();

  const double percentage = (static_cast<double>(available_clusters) / static_cast<double>(num_clusters)) * 100.0;

  return static_cast<int32_t>(std::ceil(percentage));
}

void Utils::streamUsb(Telemetry *link, uint8_t link_idx) {
  char print_char[150];

  const int32_t time_int = link->data.ts() / 10;
  const int32_t time_prec = link->data.ts() - time_int * 10;

  const auto lat_int = static_cast<int32_t>(link->data.lat());
  const auto lat_prec = static_cast<int32_t>((link->data.lat() - static_cast<float>(lat_int)) * 100000.0F);

  const auto lon_int = static_cast<int32_t>(link->data.lon());
  const auto lon_prec = static_cast<int32_t>((link->data.lon() - static_cast<float>(lon_int)) * 100000.0F);

  const auto voltage_int = static_cast<int32_t>(link->data.voltage());
  const auto voltage_prec = static_cast<int32_t>((link->data.voltage() - static_cast<float>(voltage_int)) * 10.0F);

  std::snprintf(print_char, 150,
                "Link %d: Ts: %ld.%ld, State: %d, Lat: %ld.%05ld, Lon: %ld.%05ld, Alt: %ld, Vel: %d, V: %ld.%ld",
                link_idx, time_int, time_prec, link->data.state(), lat_int, lat_prec, lon_int, lon_prec,
                link->data.altitude(), link->data.velocity(), voltage_int, voltage_prec);
  console.log.println(print_char);
}

void usbEventCallback(void *arg [[maybe_unused]], esp_event_base_t event_base, int32_t event_id,
                      void *event_data [[maybe_unused]]) {
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
  return flash.readBlocks(lba, static_cast<uint8_t *>(buffer), bufsize / 512) ? static_cast<int32_t>(bufsize) : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
static int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  return flash.writeBlocks(lba, buffer, bufsize / 512) ? static_cast<int32_t>(bufsize) : -1;
}

// Callback invoked when WRITE10 command is completed (status received and
// accepted by host). Used to flush any pending cache.
static void msc_flush_cb() {
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
