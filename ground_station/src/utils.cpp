/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Additional notice:
/// This file was adapted from Florian Baumgartner's ESP32 IoT Framework
/// (https://github.com/FlorianBaumgartner/ESP32_IoT_Framework), released under MIT License.

/// General utilities for file system support, MSC, configuration, etc.

#include "utils.hpp"
#include "Adafruit_SPIFlash.h"
#include "SPI.h"
#include "USBMSC.h"
#include "console.hpp"
#include "systemParser.hpp"
#include "telemetry/telemetry.hpp"

// clang-format off: diskio.h uses the FatFs types declared by ff.h.
// NOLINTNEXTLINE(llvm-include-order)
#include "ff.h"
#include "diskio.h"
// clang-format on

#include <esp_private/system_internal.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>

constexpr uint8_t BOOT_BUTTON = 0;
constexpr uint8_t TASK_UTILS_FREQ = 5;        // [Hz]
constexpr uint16_t MSC_STARTUP_DELAY = 2000;  // [ms]
constexpr uint32_t MSC_BLOCK_SIZE = 512;

enum class UsbConnectionState : uint8_t {
  Disconnected,
  Active,
  Suspended,
};

bool deadlineReached(TickType_t now, TickType_t deadline) { return static_cast<int32_t>(now - deadline) >= 0; }

static void msc_flush_cb();
static int32_t coreMscRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize);
static int32_t coreMscWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize);
static bool coreMscStartStop(uint8_t power_condition, bool start, bool load_eject);
static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static volatile bool updated = false;
static volatile UsbConnectionState usbConnectionState = UsbConnectionState::Disconnected;
static volatile UsbStorageState usbStorageState = UsbStorageState::FirmwareOwned;
static volatile bool filesystemMounted = true;
static SemaphoreHandle_t storageAccessMutex = nullptr;
USBMSC usb_msc;
Adafruit_FlashTransport_ESP32 flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem fatfs;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

bool Utils::begin(uint32_t watchdogTimeout, const char *labelName, bool forceFormat) {
  bool status = true;

  storageAccessMutex = xSemaphoreCreateMutex();
  if (storageAccessMutex == nullptr) {
    console.warning.println("[UTILS] Could not create storage mutex");
    status = false;
  }

  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  if (watchdogTimeout > 0) {
    startWatchdog(watchdogTimeout);
  }

  if (!flash.begin()) {
    console.warning.println("[UTILS] Could not initialize SPI Flash");
    status = false;
  }

  if (!forceFormat) {
    static FATFS volumeLabelFatfs;
    FRESULT result = f_mount(&volumeLabelFatfs, "0:", 1);
    if (result == FR_OK) {
      char currentLabel[12] = {};
      result = f_getlabel("0:", currentLabel, nullptr);
      if (result != FR_OK) {
        console.warning.printf("[UTILS] Could not read volume label, error code: %d\n", result);
      } else if (std::strcmp(currentLabel, labelName) != 0) {
        result = f_setlabel(labelName);
        if (result != FR_OK) {
          console.warning.printf("[UTILS] Could not update volume label, error code: %d\n", result);
        }
      }
    }
    f_unmount("0:");
  }

  const bool filesystemReady = fatfs.begin(&flash);
  if (!filesystemReady || forceFormat)  // Check if disk must be formatted
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

  usb_msc.vendorID(USB_MANUFACTURER);
  usb_msc.productID(USB_PRODUCT);
  usb_msc.productRevision(FIRMWARE_VERSION);
  usb_msc.onRead(coreMscRead);
  usb_msc.onWrite(coreMscWrite);
  usb_msc.onStartStop(coreMscStartStop);
  if (!usb_msc.begin(flash.size() / MSC_BLOCK_SIZE, MSC_BLOCK_SIZE)) {
    console.warning.println("[UTILS] Could not initialize USB mass storage");
    status = false;
  }
  usb_msc.mediaPresent(false);
  if (!USB.begin()) {
    console.warning.println("[UTILS] Could not initialize USB");
    status = false;
  }

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
  const esp_task_wdt_config_t watchdogConfig = {
      .timeout_ms = seconds * 1000U,
      .idle_core_mask = 0,
      .trigger_panic = true,
  };

  esp_err_t status = esp_task_wdt_init(&watchdogConfig);
  if (status == ESP_ERR_INVALID_STATE) {
    status = esp_task_wdt_reconfigure(&watchdogConfig);
  }
  if (status != ESP_OK) {
    console.warning.printf("[UTILS] Could not configure watchdog: %s\n", esp_err_to_name(status));
    return;
  }

  status = esp_task_wdt_add(nullptr);
  if (status != ESP_OK) {
    console.warning.printf("[UTILS] Could not subscribe task to watchdog: %s\n", esp_err_to_name(status));
    return;
  }

  status = esp_task_wdt_reset();
  if (status != ESP_OK) {
    console.warning.printf("[UTILS] Could not feed watchdog: %s\n", esp_err_to_name(status));
  }
}

void Utils::feedWatchdog() {
  const esp_err_t status = esp_task_wdt_reset();
  if (status != ESP_OK) {
    console.warning.printf("[UTILS] Could not feed watchdog: %s\n", esp_err_to_name(status));
  }
}

// USB connection and storage-ownership transitions intentionally share one task loop.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void Utils::update(void *pvParameter) {
  auto *ref = static_cast<Utils *>(pvParameter);

  TickType_t consoleReadyAt = 0;
  UsbConnectionState previousState = UsbConnectionState::Disconnected;
  while (true) {
    TickType_t task_last_tick = xTaskGetTickCount();

    const UsbConnectionState currentState = usbConnectionState;
    if (currentState != previousState) {
      if (currentState == UsbConnectionState::Disconnected) {
        consoleReadyAt = 0;
        console.enable(false);
        if (usbStorageState == UsbStorageState::HostOwned || usbStorageState == UsbStorageState::Preparing) {
          usbStorageState = UsbStorageState::Reclaiming;
        }
      } else if (currentState == UsbConnectionState::Suspended) {
        // Avoid blocking CDC writes while the host has suspended the bus.  A
        // resume is not a disconnect and must not change storage ownership.
        console.enable(false);
      } else if (currentState == UsbConnectionState::Active) {
        if (previousState == UsbConnectionState::Disconnected) {
          // Give the host time to finish enumeration before enabling CDC output.
          consoleReadyAt = task_last_tick + pdMS_TO_TICKS(MSC_STARTUP_DELAY);
        } else if (previousState == UsbConnectionState::Suspended) {
          console.enable(true);
        }
      }
      previousState = currentState;
    }

    if (currentState == UsbConnectionState::Active && consoleReadyAt != 0 &&
        deadlineReached(task_last_tick, consoleReadyAt)) {
      console.enable(true);
      consoleReadyAt = 0;
    }

    if (usbStorageState == UsbStorageState::Preparing) {
      if (currentState != UsbConnectionState::Active || storageAccessMutex == nullptr) {
        usbStorageState = UsbStorageState::Reclaiming;
      } else if (xSemaphoreTake(storageAccessMutex, portMAX_DELAY) == pdTRUE) {
        // FatFS::end() flushes its own sector cache into the flash driver's
        // cache.  Sync the flash only after that flush, before exposing raw
        // sectors to the host.
        const bool unmounted = fatfs.end() != nullptr;
        filesystemMounted = false;
        const bool synced = unmounted && flash.syncBlocks();
        if (synced) {
          usbStorageState = UsbStorageState::HostOwned;
          ref->mscReady = true;
          usb_msc.mediaPresent(true);
        } else {
          // Reclaiming attempts a clean remount so a failed handoff does not
          // leave firmware storage permanently unavailable.
          usbStorageState = UsbStorageState::Reclaiming;
        }
        xSemaphoreGive(storageAccessMutex);
      }
    }

    if (usbStorageState == UsbStorageState::Reclaiming) {
      if (ref->mscReady) {
        ref->mscReady = false;
        usb_msc.mediaPresent(false);
      }
      if (storageAccessMutex == nullptr) {
        usbStorageState = UsbStorageState::Fault;
      } else if (xSemaphoreTake(storageAccessMutex, portMAX_DELAY) == pdTRUE) {
        const bool synced = flash.syncBlocks();
        const bool mounted = filesystemMounted || fatfs.begin(&flash);
        filesystemMounted = mounted;
        usbStorageState = synced && mounted ? UsbStorageState::FirmwareOwned : UsbStorageState::Fault;
        if (synced && mounted) {
          updated = true;
        }
        xSemaphoreGive(storageAccessMutex);
      }
    }

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

bool Utils::isConnected() { return usbConnectionState != UsbConnectionState::Disconnected; }

bool Utils::requestMassStorage() {
  if (storageAccessMutex == nullptr || xSemaphoreTake(storageAccessMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return false;
  }
  const bool available = usbConnectionState == UsbConnectionState::Active &&
                         usbStorageState == UsbStorageState::FirmwareOwned && filesystemMounted;
  if (available) {
    usbStorageState = UsbStorageState::Preparing;
  }
  xSemaphoreGive(storageAccessMutex);
  return available;
}

void Utils::requestFirmwareStorage() {
  if (usbStorageState == UsbStorageState::Preparing || usbStorageState == UsbStorageState::HostOwned) {
    usbStorageState = UsbStorageState::Reclaiming;
  }
}

bool Utils::claimFirmwareStorage(uint32_t timeoutMs) {
  requestFirmwareStorage();
  const uint32_t startedAt = millis();
  while (!isFilesystemAvailable()) {
    if (usbStorageState == UsbStorageState::Fault || millis() - startedAt >= timeoutMs) {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return true;
}

UsbStorageState Utils::getMassStorageState() { return usbStorageState; }

bool Utils::isFilesystemAvailable() { return usbStorageState == UsbStorageState::FirmwareOwned && filesystemMounted; }

bool Utils::format(const char *labelName) {
  static FATFS elmchanFatfs;
  static uint8_t workbuf[4096];  // Working buffer for f_fdisk function.

  static const LBA_t plist[] = {100, 0, 0, 0};  // 1 primary partition with 100% of space.
  static const MKFS_PARM formatOptions = {
      .fmt = static_cast<BYTE>(static_cast<unsigned>(FM_FAT) | static_cast<unsigned>(FM_SFD)),
      .n_fat = 0,
      .align = 0,
      .n_root = 0,
      .au_size = 0,
  };
  static uint8_t buf[512] = {0};  // Working buffer for f_fdisk function.
  static FRESULT r = f_fdisk(0, plist,
                             buf);  // Partition the flash with 1 partition that takes the entire space.
  if (r != FR_OK) {
    console.warning.printf("[UTILS] Error, f_fdisk failed with error code: %d\n", r);
    return false;
  }
  r = f_mkfs("", &formatOptions, workbuf, sizeof(workbuf));  // Make filesystem.
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
  if (storageAccessMutex == nullptr || xSemaphoreTake(storageAccessMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return -1;
  }
  if (!isFilesystemAvailable()) {
    xSemaphoreGive(storageAccessMutex);
    return -1;
  }
  const uint32_t num_clusters = fatfs.clusterCount() - 2;
  const uint32_t available_clusters = fatfs.freeClusterCount();

  const auto percentage = (static_cast<double>(available_clusters) / static_cast<double>(num_clusters)) * 100.0;
  const auto usage = static_cast<int32_t>(std::ceil(percentage));
  xSemaphoreGive(storageAccessMutex);
  return usage;
}

void Utils::streamUsb(Telemetry *link, uint8_t link_idx) {
  char print_char[150];

  // Console streaming must not consume the packet update intended for HMI.
  const packedRXMessage data = link->data.snapshot();
  const float latitude = static_cast<float>(data.lat) / 10000.0F;
  const float longitude = static_cast<float>(data.lon) / 10000.0F;
  const float voltage = static_cast<float>(data.voltage) / 10.0F;
  const int32_t altitude = data.altitude;
  int16_t velocity = data.velocity;
  if ((velocity < -100) && data.state < 5U) {
    velocity += 1024;
  }

  const int32_t time_int = data.timestamp / 10;
  const int32_t time_prec = data.timestamp - time_int * 10;

  const auto lat_int = static_cast<int32_t>(latitude);
  const auto lat_prec = static_cast<int32_t>((latitude - static_cast<float>(lat_int)) * 100000.0F);

  const auto lon_int = static_cast<int32_t>(longitude);
  const auto lon_prec = static_cast<int32_t>((longitude - static_cast<float>(lon_int)) * 100000.0F);

  const auto voltage_int = static_cast<int32_t>(voltage);
  const auto voltage_prec = static_cast<int32_t>((voltage - static_cast<float>(voltage_int)) * 10.0F);

  std::snprintf(print_char, 150,
                "Link %d: Ts: %ld.%ld, State: %d, Lat: %ld.%05ld, Lon: %ld.%05ld, Alt: %ld, Vel: %d, V: %ld.%ld",
                link_idx, time_int, time_prec, data.state, lat_int, lat_prec, lon_int, lon_prec, altitude, velocity,
                voltage_int, voltage_prec);
  console.log.println(print_char);
}

void usbEventCallback(void *arg [[maybe_unused]], esp_event_base_t event_base, int32_t event_id,
                      void *event_data [[maybe_unused]]) {
  if (event_base == ARDUINO_USB_EVENTS) {
    // arduino_usb_event_data_t *data = (arduino_usb_event_data_t *)event_data;
    if (event_id == ARDUINO_USB_STARTED_EVENT || event_id == ARDUINO_USB_RESUME_EVENT) {
      usbConnectionState = UsbConnectionState::Active;
    }
    if (event_id == ARDUINO_USB_SUSPEND_EVENT) {
      usbConnectionState = UsbConnectionState::Suspended;
    }
    if (event_id == ARDUINO_USB_STOPPED_EVENT) {
      usbConnectionState = UsbConnectionState::Disconnected;
    }
  }
}

static bool msc_read_range(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t size) {
  uint8_t block[MSC_BLOCK_SIZE];
  uint32_t address = lba * MSC_BLOCK_SIZE + offset;

  while (size > 0) {
    const uint32_t blockIndex = address / MSC_BLOCK_SIZE;
    const uint32_t blockOffset = address % MSC_BLOCK_SIZE;
    const uint32_t available = MSC_BLOCK_SIZE - blockOffset;
    const uint32_t chunk = size < available ? size : available;

    if (blockOffset == 0 && chunk == MSC_BLOCK_SIZE) {
      if (!flash.readBlocks(blockIndex, buffer, 1)) {
        return false;
      }
    } else {
      if (!flash.readBlocks(blockIndex, block, 1)) {
        return false;
      }
      memcpy(buffer, block + blockOffset, chunk);
    }

    address += chunk;
    buffer += chunk;
    size -= chunk;
  }
  return true;
}

static bool msc_write_range(uint32_t lba, uint32_t offset, const uint8_t *buffer, uint32_t size) {
  uint8_t block[MSC_BLOCK_SIZE];
  uint32_t address = lba * MSC_BLOCK_SIZE + offset;

  while (size > 0) {
    const uint32_t blockIndex = address / MSC_BLOCK_SIZE;
    const uint32_t blockOffset = address % MSC_BLOCK_SIZE;
    const uint32_t available = MSC_BLOCK_SIZE - blockOffset;
    const uint32_t chunk = size < available ? size : available;

    if (blockOffset == 0 && chunk == MSC_BLOCK_SIZE) {
      if (!flash.writeBlocks(blockIndex, buffer, 1)) {
        return false;
      }
    } else {
      if (!flash.readBlocks(blockIndex, block, 1)) {
        return false;
      }
      memcpy(block + blockOffset, buffer, chunk);
      if (!flash.writeBlocks(blockIndex, block, 1)) {
        return false;
      }
    }

    address += chunk;
    buffer += chunk;
    size -= chunk;
  }
  return true;
}

// Copy a possibly partial READ10 range into the host buffer.
static int32_t coreMscRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
  if (usbStorageState != UsbStorageState::HostOwned || storageAccessMutex == nullptr ||
      xSemaphoreTake(storageAccessMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return -1;
  }
  const uint64_t endAddress = static_cast<uint64_t>(lba) * MSC_BLOCK_SIZE + offset + bufsize;
  const bool read = endAddress <= flash.size() && msc_read_range(lba, offset, static_cast<uint8_t *>(buffer), bufsize);
  xSemaphoreGive(storageAccessMutex);
  return read ? static_cast<int32_t>(bufsize) : -1;
}

// Process a possibly partial WRITE10 range from the host buffer.
static int32_t coreMscWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
  if (usbStorageState != UsbStorageState::HostOwned || storageAccessMutex == nullptr ||
      xSemaphoreTake(storageAccessMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return -1;
  }
  const uint64_t endAddress = static_cast<uint64_t>(lba) * MSC_BLOCK_SIZE + offset + bufsize;
  const bool written = endAddress <= flash.size() && msc_write_range(lba, offset, buffer, bufsize);
  xSemaphoreGive(storageAccessMutex);
  return written ? static_cast<int32_t>(bufsize) : -1;
}

// Callback invoked when WRITE10 command is completed (status received and
// accepted by host). Used to flush any pending cache.
static void msc_flush_cb() {
  if (usbStorageState != UsbStorageState::HostOwned || storageAccessMutex == nullptr ||
      xSemaphoreTake(storageAccessMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;
  }
  flash.syncBlocks();
  xSemaphoreGive(storageAccessMutex);
  updated = true;
}

static bool coreMscStartStop(uint8_t power_condition [[maybe_unused]], bool start, bool load_eject) {
  if (load_eject && !start && usbStorageState == UsbStorageState::HostOwned) {
    usbStorageState = UsbStorageState::Reclaiming;
  }
  return true;
}

extern "C" void tud_msc_write10_complete_cb(uint8_t lun [[maybe_unused]]) { msc_flush_cb(); }

//--------------------------------------------------------------------+
// fatfs diskio
//--------------------------------------------------------------------+
extern "C" {
#if FF_MULTI_PARTITION
// Preserve the existing super-floppy layout. The old build resolved this table
// from ESP-IDF's libfatfs as {0, 0}; owning it here avoids that hidden linkage.
// FatFs declares this callback table as mutable external state.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
PARTITION VolToPart[FF_VOLUMES] = {{0, 0}};
#endif

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
                  LBA_t sector,  // Start sector in LBA
                  UINT count     // Number of sectors to read
) {
  (void)pdrv;
  return flash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv,         // Physical drive nmuber to identify the drive
                   const BYTE *buff,  // Data to be written
                   LBA_t sector,      // Start sector in LBA
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
      *((LBA_t *)buff) = flash.size() / 512;
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
