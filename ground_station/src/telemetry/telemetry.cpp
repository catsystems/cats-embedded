/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "telemetry/telemetry.hpp"
#include "config.hpp"
#include "console.hpp"
#include "crc.hpp"

constexpr uint8_t TASK_TELE_FREQ = 100;

void Telemetry::begin() {
  serial.begin(115200, SERIAL_8N1, rxPin, txPin);
  parser.init(&data, &info, &location, &time);
  initialized = true;

  xTaskCreate(update, "task_telemetry", 2048, this, 1, nullptr);
}

void Telemetry::setLinkPhrase(const char* phrase, uint32_t length) {
  memset(linkPhrase, 0, kMaxPhraseLen + 1);
  memcpy(linkPhrase, phrase, length);
  newSetting = true;
}

void Telemetry::setLinkPhrase(const String& phrase) { setLinkPhrase(phrase.c_str(), phrase.length()); }

const char* Telemetry::getLinkPhrase() { return reinterpret_cast<const char*>(linkPhrase); }

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
  if (linkInitialized) {
    sendDisable();
  }

  linkInitialized = true;

  vTaskDelay(100);
  sendSetting(CMD_DIRECTION, transmissionDirection);
  vTaskDelay(100);
  sendSetting(CMD_MODE, transmissionMode);
  vTaskDelay(100);
  sendSetting(CMD_PA_GAIN, 0);
  vTaskDelay(100);

  if (linkPhrase[0] != 0) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) uint8 to char is OK
    const uint32_t phraseCrc = crc32(linkPhrase, strlen(reinterpret_cast<const char*>(linkPhrase)));
    console.error.printf("[TELEMETRY] Sending link phrase: %s (CRC: %lu)\n", linkPhrase, phraseCrc);
    sendLinkPhraseCrc(phraseCrc, 4);
    vTaskDelay(100);
    sendEnable();
    console.warning.println("[TELEMETRY] Link Enabled");
  }

  if (testingPhrase[0] != 0) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) uint8 to char is OK
    testingCrc = crc32(testingPhrase, strlen(reinterpret_cast<const char*>(testingPhrase)));
  }
}

void Telemetry::exitTesting() {
  testingMsg.header = 0x72;
  testingMsg.passcode = testingCrc;
  testingMsg.enable_pyros = 0;
  testingMsg.event = 0;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  sendTXPayload(reinterpret_cast<uint8_t*>(&testingMsg), 15);
  vTaskDelay(50);
  setMode(BIDIRECTIONAL);
  requestExitTesting = true;
}

void Telemetry::enterTesting() {
  testingMsg.header = 0x72;
  testingMsg.passcode = testingCrc;
  testingMsg.enable_pyros = 1;
  testingMsg.event = 0;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  sendTXPayload(reinterpret_cast<uint8_t*>(&testingMsg), 15);
  vTaskDelay(50);
  setMode(BIDIRECTIONAL);
}

void Telemetry::triggerEvent(uint8_t event) {
  testingMsg.header = 0x72;
  testingMsg.passcode = testingCrc;
  testingMsg.enable_pyros = 1;
  testingMsg.event = event;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  sendTXPayload(reinterpret_cast<uint8_t*>(&testingMsg), 15);
  triggerAction = true;
  triggerActionStart = xTaskGetTickCount();
}

void Telemetry::update(void* pvParameter) {
  auto* ref = static_cast<Telemetry*>(pvParameter);

  while (ref->initialized) {
    TickType_t task_last_tick = xTaskGetTickCount();

    if (ref->newSetting) {
      ref->newSetting = false;
      ref->initLink();
    }

    if (ref->requestExitTesting) {
      ref->requestExitTesting = false;
      vTaskDelay(1000);
      ref->setMode(UNIDIRECTIONAL);
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

    while (ref->serial.available()) {
      ref->parser.process(ref->serial.read());
    }

    vTaskDelayUntil(&task_last_tick, static_cast<TickType_t>(1000) / TASK_TELE_FREQ);
  }
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static) uses serial
void Telemetry::sendLinkPhraseCrc(uint32_t crc, uint32_t length) {
  console.log.println(crc);
  console.log.println(length);
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
