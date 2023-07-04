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
  void setLinkPhrase(String phrase);

  void setTestingPhrase(const char* phrase, uint32_t length);
  void setTestingPhrase(String phrase);

  void setDirection(transmission_direction_e dir);
  void setMode(transmission_mode_e mode);

  void sendTXPayload(uint8_t* payload, uint32_t length);

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

  TelemetryData data;
  TelemetryInfo info;
  TelemetryLocation location;
  TelemetryTime time;

 private:
  void initLink();

  void sendLinkPhraseCrc(uint32_t crc, uint32_t length);
  void sendSetting(uint8_t command, uint8_t value);
  void sendEnable();
  void sendDisable();

  static void update(void* pvParameter);

  volatile bool initialized = false;
  volatile bool linkInitialized = false;

  HardwareSerial serial;

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

  struct packed_testing_msg_t {
    uint8_t header;
    uint32_t passcode;
    uint8_t event;
    uint8_t enable_pyros;
    uint32_t dummy1;
    uint32_t dummy2;
  } __attribute__((packed)) testingMsg;
};
