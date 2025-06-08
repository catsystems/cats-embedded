/// Copyright (C) 2020, 2025 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "Arduino.h"

#include <cstdint>
#include <memory>

enum class CompassModel { kQMC5883L, kQMC5883P, kInvalid };

class QMC5883Compass {
 public:
  virtual void init();

  virtual void setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max);
  virtual void read();

  virtual void readRaw(float* destination);
  virtual void readCalibrated(float* destination);

  virtual ~QMC5883Compass() = default;

 protected:
  virtual void _writeReg(byte reg, byte val);
  virtual void setADDR(byte b);
  byte _ADDR = 0x0D;  // Default address for QMC5883L
  bool calibrationUse = false;

  float raw[3] = {0, 0, 0};
  int calibration[3][2];
  float calibrated[3];
  void applyCalibration();

  float xOffset, yOffset, zOffset;
  float xScale, yScale, zScale;
};

// Sensor factory declared here
class CreateCompass final {
 public:
  // Prevent instantiation
  CreateCompass() = delete;
  CreateCompass(const CreateCompass&) = delete;
  CreateCompass(CreateCompass&&) = delete;
  CreateCompass& operator=(const CreateCompass&) = delete;
  CreateCompass& operator=(CreateCompass&&) = delete;

  [[nodiscard]] static std::unique_ptr<QMC5883Compass> createSensor();
};
