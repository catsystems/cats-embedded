/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Additional notice:
/// This file was adapted from Florian Baumgartner's ESP32 IoT Framework
/// (https://github.com/FlorianBaumgartner/ESP32_IoT_Framework), released under MIT License.

/// Wrapper for Serial Debugging (threadsafe, buffered, colorized, etc.)

#include "console.hpp"

bool Console::begin() {
  if (type == USBCDC_t) {
    // NOLINTBEGIN(cppcoreguidelines-pro-type-static-cast-downcast) dynamic cast is not allowed with -fno-rtti
    static_cast<USBCDC*>(&stream)->enableReboot(true);  // Enables entering bootloader when changing to baudrate of 1200
                                                        // bit/s (normaly not used, due to dedicated DFU USB-Endpoint)
    static_cast<USBCDC*>(&stream)->onEvent(usbEventCallback);
    static_cast<USBCDC*>(&stream)->begin();
    // NOLINTEND(cppcoreguidelines-pro-type-static-cast-downcast) dynamic cast is not allowed with -fno-rtti
  } else {
    return false;
  }
  return initialize();
}

bool Console::begin(uint32_t baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert, uint32_t timeout_ms,
                    uint8_t rxfifo_full_thrhd) {
  if (type == HardwareSerial_t) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast) dynamic cast is not allowed with -fno-rtti
    static_cast<HardwareSerial*>(&stream)->begin(baud, config, rxPin, txPin, invert, timeout_ms, rxfifo_full_thrhd);
  } else {
    return false;
  }
  return initialize();
}

bool Console::initialize() {
  initialized = true;
  bufferAccessSemaphore = xSemaphoreCreateMutex();
  xTaskCreate(writeTask, "task_consoleWrite", 4096, this, 1, &writeTaskHandle);
  xTaskCreate(interfaceTask, "task_consoleIface", 4096, this, 1, nullptr);  // TODO: Stack size must be that large?!
  return true;
}

void Console::end() { initialized = false; }

void Console::writeTask(void* pvParameter) {
  auto* ref = static_cast<Console*>(pvParameter);

  while (ref->initialized) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait on notification for data in buffer or console opened
    if (ref->streamActive) {
      if (xSemaphoreTake(ref->bufferAccessSemaphore, portMAX_DELAY)) {
        if (ref->readIdx < ref->writeIdx)  // Regular case, no wrap around needed
        {
          // NOLINTNEXTLINE(google-readability-casting)
          ref->stream.write((const uint8_t*)ref->ringBuffer + ref->readIdx, ref->writeIdx - ref->readIdx);
        } else if (ref->readIdx > ref->writeIdx)  // Need to send buffer in two parts (ReadIdx to End | 0 to WriteIdx)
        {
          // NOLINTNEXTLINE(google-readability-casting)
          ref->stream.write((const uint8_t*)ref->ringBuffer + ref->readIdx, QUEUE_BUFFER_LENGTH - ref->readIdx);
          // NOLINTNEXTLINE(google-readability-casting)
          ref->stream.write((const uint8_t*)ref->ringBuffer, ref->writeIdx);
        }
        ref->readIdx = ref->writeIdx;
        xSemaphoreGive(ref->bufferAccessSemaphore);
      }
    }
  }
  vTaskDelete(nullptr);
}

void Console::interfaceTask(void* pvParameter) {
  auto* ref = static_cast<Console*>(pvParameter);

  TickType_t interfaceTimer{0};
  TickType_t enabledTimer{0};
  bool enabledOld = false;
  bool enabledDelayed = false;
  bool interfaceOld = false;
  bool interfaceDelayed = false;
  bool streamActiveOld = false;
  while (ref->initialized) {
    TickType_t task_last_tick = xTaskGetTickCount();

    if (ref->enabled && !enabledOld) {
      enabledTimer = xTaskGetTickCount() + CONSOLE_ACTIVE_DELAY;
    }
    enabledOld = ref->enabled;
    enabledDelayed = (xTaskGetTickCount() > enabledTimer) && ref->enabled;

    if (ref->getInterfaceState() && !interfaceOld) {
      interfaceTimer = xTaskGetTickCount() + INTERFACE_ACTIVE_DELAY;
    }
    interfaceOld = ref->getInterfaceState();
    interfaceDelayed = (xTaskGetTickCount() > interfaceTimer) && ref->getInterfaceState();

    ref->streamActive = enabledDelayed && interfaceDelayed;
    if (ref->streamActive && !streamActiveOld) {
      ref->printStartupMessage();
      vTaskDelay(static_cast<TickType_t>(10));  // Make sure that startup message is printed befor everything else
      xTaskNotifyGive(ref->writeTaskHandle);    // Send signal to update task (for sending out data in queue buffer)
    }
    if (!ref->streamActive && streamActiveOld)  // Detect if console has been closed
    {
      ref->stream.flush();
      ref->stream.clearWriteError();
    }
    streamActiveOld = ref->streamActive;

    vTaskDelayUntil(&task_last_tick, static_cast<TickType_t>(1000) / INTERFACE_UPDATE_RATE);
  }
  vTaskDelete(nullptr);
}

size_t Console::write(const uint8_t* buffer, size_t size) {
  if (size == 0) {
    return 0;
  }
  if (xSemaphoreTake(bufferAccessSemaphore, portMAX_DELAY)) {
    uint32_t free{0};
    size = min(size, static_cast<size_t>(QUEUE_BUFFER_LENGTH) - 1);
    if (writeIdx + size <= QUEUE_BUFFER_LENGTH) {
      // NOLINTNEXTLINE(google-readability-casting)
      memcpy((uint8_t*)ringBuffer + writeIdx, buffer, size);
      free = QUEUE_BUFFER_LENGTH - (writeIdx - readIdx);
    } else {
      const uint32_t firstPartSize = QUEUE_BUFFER_LENGTH - writeIdx;
      // NOLINTNEXTLINE(google-readability-casting)
      memcpy((uint8_t*)ringBuffer + writeIdx, buffer, firstPartSize);
      // NOLINTNEXTLINE(google-readability-casting)
      memcpy((uint8_t*)ringBuffer, buffer + firstPartSize, size - firstPartSize);
      free = readIdx - writeIdx;
    }
    writeIdx = (writeIdx + size) & (QUEUE_BUFFER_LENGTH - 1);
    if (size > free) {
      readIdx = (readIdx + (size - free)) & (QUEUE_BUFFER_LENGTH - 1);
    }

    xSemaphoreGive(bufferAccessSemaphore);
    xTaskNotifyGive(writeTaskHandle);  // Send signal to update task (for sending out data)
    return size;
  }
  return 0;
}

void Console::printTimestamp() {
  const uint32_t h = std::min(millis() / 3600000, 99UL);
  const auto m = static_cast<uint32_t>((millis() / 60000) % 60);
  const auto s = static_cast<uint32_t>((millis() / 1000) % 60);
  const auto ms = static_cast<uint32_t>(millis() % 1000);
  printf("[%02lu:%02lu:%02lu.%03lu] ", h, m, s, ms);
}

void Console::printStartupMessage() {
  stream.print(CONSOLE_CLEAR);
  stream.print(CONSOLE_COLOR_BOLD_CYAN CONSOLE_BACKGROUND_DEFAULT);
  stream.println("****************************************************");
  stream.println("*                CATS Groundstation                *");
  stream.println("****************************************************");
  stream.println(CONSOLE_LOG);
}

void Console::usbEventCallback(void* arg [[maybe_unused]], esp_event_base_t event_base, int32_t event_id,
                               void* event_data [[maybe_unused]]) {
  if (event_base == ARDUINO_USB_CDC_EVENTS) {
    switch (event_id) {
      case ARDUINO_USB_CDC_CONNECTED_EVENT:
      case ARDUINO_USB_CDC_DISCONNECTED_EVENT:
      case ARDUINO_USB_CDC_LINE_STATE_EVENT:
      case ARDUINO_USB_CDC_LINE_CODING_EVENT:
      default:
        break;
    }
  }
}

#ifndef USE_CUSTOM_CONSOLE
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
USBCDC USBSerial;
Console console(USBSerial);
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)
#endif
