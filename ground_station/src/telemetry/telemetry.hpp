#pragma once

#include "config.hpp"
#include "parser.hpp"
#include "telemetryData.hpp"
#include "telemetry_reg.hpp"

class Telemetry {
 public:
  Telemetry(HardwareSerial& serial, int rxPin, int txPin) : serial(serial), rxPin(rxPin), txPin(txPin) {}
  void begin();

  void setLinkPhrase(const char* phrase, uint32_t length);
  void setLinkPhrase(const String& phrase);

  void setTestingPhrase(const char* phrase, uint32_t length);
  void setTestingPhrase(const String& phrase);

  void setDirection(transmission_direction_e dir);
  void setMode(transmission_mode_e mode);

  void exitTesting();
  void enterTesting();
  void triggerEvent(uint8_t event);

  void disable() {
    if (linkInitialized) {
      sendDisable();
      linkInitialized = false;
    }
  }

  void enable() {
    if (!linkInitialized) {
      sendEnable();
      linkInitialized = true;
    }
  }

  // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes)
  TelemetryData data{};
  TelemetryInfo info{};
  TelemetryLocation location{};
  TelemetryTime time{};
  // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes)

 private:
  HardwareSerial serial;

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
