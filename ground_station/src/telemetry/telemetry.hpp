/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <atomic>

#include "config.hpp"
#include "parser.hpp"
#include "telemetryData.hpp"
#include "telemetry_reg.hpp"

class Telemetry {
 public:
  Telemetry(HardwareSerial& serial, int rxPin, int txPin) : serial(serial), rxPin(rxPin), txPin(txPin) {}
  void begin();

  void setPacketSink(ITelemetryPacketSink* sink, uint8_t source) { parser.setPacketSink(sink, source); }

  void setLinkPhrase(const char* phrase, uint32_t length);
  void setLinkPhrase(const String& phrase);

  void setTestingPhrase(const char* phrase, uint32_t length);
  void setTestingPhrase(const String& phrase);

  void setDirection(transmission_direction_e dir);
  void setMode(transmission_mode_e mode);

  void exitTesting();
  void enterTesting();
  void triggerEvent(uint8_t event);
  bool setSelfTestOverride(const char* phrase, bool enabled = true);
  bool selfTestReady() const { return controlApplied.load() == controlRequested.load(); }
  void requestVersion() { versionRequested = true; }
  bool versionReadComplete() const { return versionReadDone.load(); }
  SelfTestLinkObservation diagnostics() const { return parser.diagnostics(); }
  void resetReceptionStats(uint32_t now) { parser.resetReceptionStats(now); }

  void disable();
  void enable();

  // Acknowledged handoff: only the update worker may use this UART until
  // finishUpdate(). No task suspension while a parser/setting write is active.
  bool safeForUpdate();
  bool beginUpdate();
  void finishUpdate(bool healthy, const char* verifiedVersion = nullptr);
  HardwareSerial& updateSerial() { return serial; }
  void configureUpdateUart(bool rom);
  bool isQuarantined() const { return quarantined; }

  // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes)
  TelemetryData data{};
  TelemetryInfo info{};
  TelemetryLocation location{};
  TelemetryTime time{};
  // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes)

 private:
  HardwareSerial serial;
  struct SelfTestControl {
    char phrase[kMaxPhraseLen + 1]{};
    bool active{false};
    bool enabled{true};
    uint32_t generation{0};
  };
  SelfTestControl selfTestControl{};
  QueueHandle_t controlQueue{nullptr};
  std::atomic<uint32_t> controlRequested{0};
  std::atomic<uint32_t> controlApplied{0};
  std::atomic<bool> versionRequested{false};
  std::atomic<bool> versionReadDone{false};
  SemaphoreHandle_t uartMutex{nullptr};
  std::atomic<bool> updateRequested{false};
  std::atomic<bool> updateGranted{false};
  std::atomic<bool> quarantined{false};
  bool testingActive{false};
  bool safeForUpdateLocked() const;
  bool lockNormalWriter();

  void initLink();

  static void update(void* pvParameter);

  void sendLinkPhraseCrc(uint32_t crc, uint32_t length);
  void sendSetting(uint8_t command, uint8_t value);
  void sendEnable();
  void sendDisable();
  void sendTXPayload(uint8_t* payload, uint32_t length);

  volatile bool initialized = false;
  volatile bool linkInitialized = false;

  Parser parser;
  int rxPin;
  int txPin;

  uint8_t linkPhrase[kMaxPhraseLen + 1] = {};
  uint8_t testingPhrase[kMaxPhraseLen + 1] = {};
  uint32_t testingCrc = 0;

  bool requestExitTesting = false;
  bool triggerAction = false;
  uint32_t triggerActionStart = 0;
  bool newSetting = false;
  transmission_direction_e transmissionDirection = RX_DIR;
  transmission_mode_e transmissionMode = UNIDIRECTIONAL;

  struct [[gnu::packed]] {
    uint8_t header;
    uint32_t passcode;
    uint8_t event;
    uint8_t enable_pyros;
    uint32_t dummy1;
    uint32_t dummy2;
  } testingMsg{};

  static_assert(sizeof(testingMsg) == 15);
};
