#pragma once

#include <cstdint>

class TelemetryData {
 public:
  void set(uint8_t state, uint8_t errors, int32_t altitude, int16_t velocity, float latitude, float longitude,
           float voltage, uint8_t continuity, bool testingMode) {
    state_ = state;
    errors_ = errors;
    altitude_ = altitude;
    velocity_ = velocity;
    latitude_ = latitude;
    longitude_ = longitude;
    voltage_ = voltage;
    continuity_ = continuity;
    testingMode_ = testingMode;
    updated_ = true;
  }
  void clear() { updated_ = false; }
  [[nodiscard]] bool isUpdated() const { return updated_; }
  int16_t velocity() { updated_ = false; return velocity_; }
  int32_t altitude() { updated_ = false; return altitude_; }
  float lat() { updated_ = false; return latitude_; }
  float lon() { updated_ = false; return longitude_; }
  uint16_t state() { updated_ = false; return state_; }
  uint8_t errors() { updated_ = false; return errors_; }
  float voltage() { updated_ = false; return voltage_; }
  uint8_t pyroContinuity() { updated_ = false; return continuity_; }
  bool testingMode() { updated_ = false; return testingMode_; }

 private:
  uint8_t state_ = 2;
  uint8_t errors_ = 0;
  int32_t altitude_ = 0;
  int16_t velocity_ = 0;
  float latitude_ = 0.0F;
  float longitude_ = 0.0F;
  float voltage_ = 0.0F;
  uint8_t continuity_ = 0;
  bool testingMode_ = false;
  bool updated_ = false;
};

class TelemetryInfo {
 public:
  void set(uint8_t linkQuality, int8_t rssi, int8_t snr) {
    linkQuality_ = linkQuality;
    rssi_ = rssi;
    snr_ = snr;
    updated_ = true;
  }
  void clear() { updated_ = false; }
  [[nodiscard]] bool isUpdated() const { return updated_; }
  int16_t snr() { updated_ = false; return snr_; }
  int16_t rssi() { updated_ = false; return rssi_; }
  uint16_t lq() { updated_ = false; return linkQuality_; }

 private:
  uint8_t linkQuality_ = 0;
  int8_t rssi_ = 0;
  int8_t snr_ = 0;
  bool updated_ = false;
};
