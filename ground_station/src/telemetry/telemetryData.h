#pragma once

#include <Arduino.h>
#include "console.h"

typedef struct {
  uint8_t state : 3;
  uint16_t timestamp : 15;
  uint8_t errors : 6;
  int32_t lat : 22;
  int32_t lon : 22;
  int32_t altitude : 17;
  int16_t velocity : 10;
  uint16_t voltage : 8;
  uint8_t pyro_continuity : 2;
  bool testing_mode : 1;
  // fill up to 16 bytes
  uint8_t : 0;  // sent
  uint8_t d1;   // dummy
} __attribute__((packed)) packedRXMessage;

static_assert(sizeof(packedRXMessage) == 15);

class TelemetryData {
 public:
  void commit(uint8_t *data, uint32_t length) {
    memcpy(&rxData, data, length);
    lastCommitTime = xTaskGetTickCount();
    updated = true;
  }

  /// Clears the updated flag
  void clear() { updated = false; }

  /// Returns true if the data has been updated since the last call to clear()
  bool isUpdated() const { return updated; }

  int16_t velocity() {
    updated = false;
    return rxData.velocity;
  }

  int32_t altitude() {
    updated = false;
    return rxData.altitude;
  }

  uint16_t ts() {
    updated = false;
    return rxData.timestamp;
  }

  float lat() {
    updated = false;
    return (float)rxData.lat / 10000.0f;
  }

  float lon() {
    updated = false;
    return (float)rxData.lon / 10000.0f;
  }

  int8_t d1() { return rxData.d1; }

  uint16_t state() {
    updated = false;
    return rxData.state;
  }

  uint8_t errors() {
    updated = false;
    return rxData.errors;
  }

  float voltage() {
    updated = false;
    return static_cast<float>(rxData.voltage / 10.0F);
  }

  uint8_t pyroContinuity() {
    updated = false;
    return rxData.pyro_continuity;
  }

  bool testingMode() {
    updated = false;
    return rxData.testing_mode;
  }

  uint32_t getLastUpdateTime() const { return lastCommitTime; }

  packedRXMessage rxData;

 private:
  bool updated;
  uint32_t lastCommitTime;
};

typedef struct {
  uint8_t lq;
  int8_t rssi;
  int8_t snr;
} __attribute__((packed)) TelemetryInfoData;

class TelemetryInfo {
 public:
  void commit(uint8_t *data, uint32_t length) {
    memcpy(&infoData, data, sizeof(infoData));
    lastCommitTime = millis();
    updated = true;
  }

  void clear() { updated = false; }

  bool isUpdated() const { return updated; }

  int16_t snr() {
    updated = false;
    return (int16_t)infoData.snr;
  }

  int16_t rssi() {
    updated = false;
    return (int16_t)infoData.rssi;
  }

  uint16_t lq() {
    updated = false;
    return (uint16_t)infoData.lq;
  }

 private:
  TelemetryInfoData infoData;
  uint32_t lastCommitTime;
  bool updated;
};

typedef struct {
  uint8_t second;
  uint8_t minute;
  uint8_t hour;
} __attribute__((packed)) TelemetryTimeData;

class TelemetryTime {
 public:
  void commit(uint8_t *data, uint32_t length) {
    memcpy(&timeData, data, sizeof(TelemetryTimeData));
    lastCommitTime = millis();
    updated = true;
    wasUpdated = true;
  }

  bool isUpdated() const { return updated; }

  uint8_t second() {
    updated = false;
    return timeData.second;
  }

  uint8_t minute() {
    updated = false;
    return timeData.minute;
  }

  uint8_t hour() {
    updated = false;
    return timeData.hour;
  }

 private:
  TelemetryTimeData timeData;
  uint32_t lastCommitTime;
  bool updated;
  bool wasUpdated = false;
};

typedef struct {
  float lat;
  float lon;
  int32_t alt;
} __attribute__((packed)) TelemetryLocationData;

class TelemetryLocation {
 public:
  void commit(uint8_t *data, uint32_t length) {
    memcpy(&locationData, data, sizeof(TelemetryLocationData));
    lastCommitTime = millis();
    updated = true;
    wasUpdated = true;
  }

  bool isUpdated() const { return updated; }

  bool isValid() const {
    if (locationData.lat && locationData.lon && wasUpdated) {
      return true;
    }
    return false;
  }

  float lat() {
    updated = false;
    return locationData.lat;
  }

  float lon() {
    updated = false;
    return locationData.lon;
  }

  int16_t alt() {
    updated = false;
    return (uint16_t)locationData.alt;
  }

 private:
  TelemetryLocationData locationData;
  uint32_t lastCommitTime;
  bool updated;
  bool wasUpdated = false;
};
