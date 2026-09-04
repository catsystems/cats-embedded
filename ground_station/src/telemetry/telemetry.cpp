/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "telemetry/telemetry.hpp"
#include "config.hpp"
#include "console.hpp"
#include "crc.hpp"

constexpr uint8_t TASK_TELE_FREQ = 100;

void Telemetry::begin() {
  uartMutex = xSemaphoreCreateMutex();
  if (uartMutex == nullptr) {
    return;
  }
  serial.begin(115200, SERIAL_8N1, rxPin, txPin);
  parser.init(&data, &info, &location, &time);
  controlQueue = xQueueCreate(1, sizeof(SelfTestControl));
  initialized = true;

  xTaskCreate(update, "task_telemetry", 2048, this, 1, nullptr);
}

bool Telemetry::setSelfTestOverride(const char* phrase, bool enabled) {
  SelfTestControl control{};
  control.active = phrase != nullptr;
  control.enabled = enabled;
  if (phrase != nullptr) strncpy(control.phrase, phrase, kMaxPhraseLen);
  control.generation = ++controlRequested;
  return controlQueue != nullptr && xQueueOverwrite(controlQueue, &control) == pdPASS;
}

void Telemetry::setLinkPhrase(const char* phrase, uint32_t length) {
  memset(linkPhrase, 0, kMaxPhraseLen + 1);
  memcpy(linkPhrase, phrase, length);
  newSetting = true;
}

void Telemetry::setLinkPhrase(const String& phrase) { setLinkPhrase(phrase.c_str(), phrase.length()); }

void Telemetry::setTestingPhrase(const char* phrase, uint32_t length) {
  memset(testingPhrase, 0, kMaxPhraseLen + 1);
  memcpy(testingPhrase, phrase, length);
  newSetting = true;
}

void Telemetry::setTestingPhrase(const String& phrase) { setTestingPhrase(phrase.c_str(), phrase.length()); }

void Telemetry::setDirection(transmission_direction_e dir) {
  if (dir != transmissionDirection) {
    transmissionDirection = dir;
    newSetting = true;
  }
}

void Telemetry::setMode(transmission_mode_e mode) {
  if (mode != transmissionMode) {
    transmissionMode = mode;
    newSetting = true;
  }
}

void Telemetry::initLink() {
  const auto direction = selfTestControl.active ? RX_DIR : transmissionDirection;
  const auto mode = selfTestControl.active ? UNIDIRECTIONAL : transmissionMode;
  const char* phrase = selfTestControl.active ? selfTestControl.phrase : reinterpret_cast<const char*>(linkPhrase);
  // The receiver MCU can remain active across an ESP reset, so its state is
  // unknown even during the first initialization.
  sendDisable();
  linkInitialized = false;

  vTaskDelay(100);
  sendSetting(CMD_DIRECTION, direction);
  vTaskDelay(100);
  sendSetting(CMD_MODE, mode);
  vTaskDelay(100);
  sendSetting(CMD_PA_GAIN, 0);
  vTaskDelay(100);

  if (phrase[0] != 0 && (!selfTestControl.active || selfTestControl.enabled)) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) uint8 to char is OK
    const uint32_t phraseCrc = crc32(reinterpret_cast<const uint8_t*>(phrase), strlen(phrase));
    sendLinkPhraseCrc(phraseCrc, 4);
    vTaskDelay(100);

    // Keep this next to ENABLE. If the receiver became ready partway through
    // startup, it must still see the intended direction before starting.
    sendSetting(CMD_DIRECTION, direction);
    sendEnable();
    linkInitialized = true;
  }

  if (testingPhrase[0] != 0) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) uint8 to char is OK
    testingCrc = crc32(testingPhrase, strlen(reinterpret_cast<const char*>(testingPhrase)));
  }
}

void Telemetry::exitTesting() {
  if (!lockNormalWriter()) {
    return;
  }
  testingMsg.header = 0x72;
  testingMsg.passcode = testingCrc;
  testingMsg.enable_pyros = 0;
  testingMsg.event = 0;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  sendTXPayload(reinterpret_cast<uint8_t*>(&testingMsg), 15);
  vTaskDelay(50);
  setMode(BIDIRECTIONAL);
  requestExitTesting = true;
  xSemaphoreGive(uartMutex);
}

void Telemetry::enterTesting() {
  if (!lockNormalWriter()) {
    return;
  }
  testingActive = true;
  testingMsg.header = 0x72;
  testingMsg.passcode = testingCrc;
  testingMsg.enable_pyros = 1;
  testingMsg.event = 0;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  sendTXPayload(reinterpret_cast<uint8_t*>(&testingMsg), 15);
  vTaskDelay(50);
  setMode(BIDIRECTIONAL);
  xSemaphoreGive(uartMutex);
}

void Telemetry::triggerEvent(uint8_t event) {
  if (!lockNormalWriter()) {
    return;
  }
  testingMsg.header = 0x72;
  testingMsg.passcode = testingCrc;
  testingMsg.enable_pyros = 1;
  testingMsg.event = event;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  sendTXPayload(reinterpret_cast<uint8_t*>(&testingMsg), 15);
  triggerAction = true;
  triggerActionStart = xTaskGetTickCount();
  xSemaphoreGive(uartMutex);
}

void Telemetry::update(void* pvParameter) {
  auto* ref = static_cast<Telemetry*>(pvParameter);
  const uint32_t versionReadStarted = millis();
  uint32_t versionLastRequest = versionReadStarted - 1000U;

  while (ref->initialized) {
    TickType_t task_last_tick = xTaskGetTickCount();

    if (xSemaphoreTake(ref->uartMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
      continue;
    }
    if (ref->updateRequested || ref->quarantined) {
      if (ref->updateRequested && !ref->updateGranted) {
        for (size_t count = 0; count < 512 && ref->serial.available(); ++count) {
          ref->parser.process(ref->serial.read());
        }
        if (!ref->safeForUpdateLocked()) {
          xSemaphoreGive(ref->uartMutex);
          vTaskDelay(pdMS_TO_TICKS(10));
          continue;
        }
        ref->sendDisable();
        ref->serial.flush();
        ref->linkInitialized = false;
        ref->updateGranted = true;
      }
      xSemaphoreGive(ref->uartMutex);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (ref->controlQueue != nullptr && xQueueReceive(ref->controlQueue, &ref->selfTestControl, 0) == pdPASS) {
      ref->newSetting = true;
    }

    if (ref->newSetting) {
      ref->newSetting = false;
      ref->initLink();
      ref->controlApplied = ref->selfTestControl.generation;
    }

    // The telemetry MCU waits 4 seconds for GNSS at boot; allow 8 seconds for its reply.
    // Explicit requests remain available to self-test.
    if (!ref->versionReadDone.load()) {
      const uint32_t now = millis();
      if (ref->diagnostics().versionReplies != 0 || now - versionReadStarted >= 8000U) {
        ref->versionReadDone = true;
      } else if (now - versionLastRequest >= 1000U) {
        ref->versionRequested = true;
        versionLastRequest = now;
      }
    }

    if (ref->versionRequested.exchange(false)) {
      const uint8_t header[] = {CMD_VERSION_INFO, 0};
      uint8_t request[] = {CMD_VERSION_INFO, 0, crc8(header, sizeof(header))};
      ref->serial.write(request, sizeof(request));
    }

    if (ref->requestExitTesting) {
      ref->requestExitTesting = false;
      vTaskDelay(1000);
      ref->setMode(UNIDIRECTIONAL);
      ref->testingActive = false;
    }

    if (ref->triggerAction && (ref->triggerActionStart + 1000) < xTaskGetTickCount()) {
      ref->triggerAction = false;
      ref->testingMsg.header = 0x72;
      ref->testingMsg.passcode = ref->testingCrc;
      ref->testingMsg.enable_pyros = 1;
      ref->testingMsg.event = 0;
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      ref->sendTXPayload(reinterpret_cast<uint8_t*>(&ref->testingMsg), 15);
    }

    for (size_t count = 0; count < 512 && ref->serial.available(); ++count) {
      ref->parser.process(ref->serial.read());
    }

    xSemaphoreGive(ref->uartMutex);

    vTaskDelayUntil(&task_last_tick, static_cast<TickType_t>(1000) / TASK_TELE_FREQ);
  }
}

bool Telemetry::lockNormalWriter() {
  if (uartMutex == nullptr || xSemaphoreTake(uartMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
    return false;
  }
  if (updateRequested || quarantined) {
    xSemaphoreGive(uartMutex);
    return false;
  }
  return true;
}

void Telemetry::disable() {
  if (!lockNormalWriter()) {
    return;
  }
  if (linkInitialized) {
    sendDisable();
    linkInitialized = false;
  }
  xSemaphoreGive(uartMutex);
}

void Telemetry::enable() {
  if (!lockNormalWriter()) {
    return;
  }
  if (!linkInitialized && linkPhrase[0] != 0) {
    sendEnable();
    linkInitialized = true;
  }
  xSemaphoreGive(uartMutex);
}

bool Telemetry::safeForUpdateLocked() const {
  const auto packet = data.snapshot();
  const bool recent = (xTaskGetTickCount() - data.getLastUpdateTime()) <= pdMS_TO_TICKS(2000);
  const bool airborne = packet.state > 2 && packet.state < 7;
  return initialized && !quarantined && !testingActive && !requestExitTesting && !triggerAction &&
         !(recent && (airborne || packet.testing_mode));
}

bool Telemetry::safeForUpdate() {
  if (uartMutex == nullptr || xSemaphoreTake(uartMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
    return false;
  }
  const bool safe = safeForUpdateLocked();
  xSemaphoreGive(uartMutex);
  return safe;
}

bool Telemetry::beginUpdate() {
  if (uartMutex == nullptr || xSemaphoreTake(uartMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
    return false;
  }
  const bool safe = !updateRequested && safeForUpdateLocked();
  if (safe) {
    updateGranted = false;
    updateRequested = true;
  }
  xSemaphoreGive(uartMutex);
  if (!safe) {
    return false;
  }
  const uint32_t started = millis();
  while (!updateGranted && millis() - started < 2000) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if (!updateGranted) {
    // Cancellation is serialized with the grant; no delayed grant can escape.
    if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
      updateRequested = false;
      updateGranted = false;
      newSetting = true;
      xSemaphoreGive(uartMutex);
    }
    return false;
  }
  return true;
}

void Telemetry::configureUpdateUart(bool rom) {
  /* HardwareSerial::begin() updates data bits/parity/stop bits in place when
   * the UART is already running. Keeping the pins attached prevents a low
   * pulse on TX from poisoning the STM32 ROM's autobaud detection. */
  serial.begin(115200, rom ? SERIAL_8E1 : SERIAL_8N1, rxPin, txPin);
}

void Telemetry::finishUpdate(bool healthy) {
  if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
    quarantined = true;
    updateRequested = false;
    return;
  }
  // The normal task has acknowledged and no longer touches the UART/parser.
  configureUpdateUart(false);
  parser.reset();
  linkInitialized = false;
  quarantined = !healthy;
  newSetting = healthy;
  updateGranted = false;
  updateRequested = false;
  xSemaphoreGive(uartMutex);
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static) uses serial
void Telemetry::sendLinkPhraseCrc(uint32_t crc, uint32_t length) {
  uint8_t out[7];  // 1 OP + 1 LEN + 4 DATA + 1 CRC
  out[0] = CMD_LINK_PHRASE;
  out[1] = static_cast<uint8_t>(length);
  memcpy(&out[2], &crc, length);
  out[length + 2] = crc8(out, length + 2);

  serial.write(out, length + 3);
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static) uses serial
void Telemetry::sendSetting(uint8_t command, uint8_t value) {
  uint8_t out[4];  // 1 OP + 1 LEN + 1 DATA + 1 CRC
  out[0] = command;
  out[1] = 1;
  out[2] = value;
  out[3] = crc8(out, 3);

  serial.write(out, 4);
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static) uses serial
void Telemetry::sendEnable() {
  uint8_t out[3];  // 1 OP + 1 LEN + 1 DATA + 1 CRC
  out[0] = CMD_ENABLE;
  out[1] = 0;
  out[2] = crc8(out, 2);

  serial.write(out, 3);
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static) uses serial
void Telemetry::sendDisable() {
  uint8_t out[3];  // 1 OP + 1 LEN + 1 DATA + 1 CRC
  out[0] = CMD_DISBALE;
  out[1] = 0;
  out[2] = crc8(out, 2);

  serial.write(out, 3);
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static) uses serial
void Telemetry::sendTXPayload(uint8_t* payload, uint32_t length) {
  uint8_t out[19];  // 1 OP + 1 LEN + 16 DATA + 1 CRC
  out[0] = CMD_TX;
  out[1] = static_cast<uint8_t>(length);
  memcpy(&out[2], payload, length);
  out[length + 2] = crc8(out, length + 2);

  serial.write(out, length + 3);
}
