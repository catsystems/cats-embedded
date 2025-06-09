/// Copyright (C) 2020, 2025 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "QMC5883Compass.hpp"
#include "QMC5883LCompass.hpp"
#include "QMC5883PCompass.hpp"

#include <memory>

/* Check the whoAmI register to select the proper magnetometer. */
static CompassModel SelectCompass() {
  Wire.begin(14, 15, 400000UL);
  Wire.beginTransmission(0x0D);
  Wire.write(0x0D);  // WhoAmI reg
  int err = Wire.endTransmission();
  uint8_t whoAmI = 0;
  if (!err) {
    Wire.requestFrom(0x0D, (byte)1);
    whoAmI = Wire.read();
  }

  if (whoAmI == 0xFF) {
    return CompassModel::kQMC5883L;
  }

  /* Check WhoamI of QMC5883P */
  Wire.beginTransmission(0x2C);
  Wire.write(0x00);  // WhoAmI reg
  err = Wire.endTransmission();
  if (!err) {
    Wire.requestFrom(0x2C, (byte)1);
    whoAmI = Wire.read();
  }

  if (whoAmI == 0x80) {
    return CompassModel::kQMC5883P;
  }

  /* Handle Error */
  return CompassModel::kInvalid;
}

/* Set the calibration of the magnetometer, computed from the fibonacci sphere in the navigation task. */
void QMC5883Compass::setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max) {
  calibrationUse = true;

  calibration[0][0] = x_min;
  calibration[0][1] = x_max;
  calibration[1][0] = y_min;
  calibration[1][1] = y_max;
  calibration[2][0] = z_min;
  calibration[2][1] = z_max;

  applyCalibration();
}

/* Apply the calibration to the mag data. */
void QMC5883Compass::applyCalibration() {
  xOffset = (float)(calibration[0][0] + calibration[0][1]) / 2.0f;
  yOffset = (float)(calibration[1][0] + calibration[1][1]) / 2.0f;
  zOffset = (float)(calibration[2][0] + calibration[2][1]) / 2.0f;
  float xAvgDelta = (float)(calibration[0][1] - calibration[0][0]) / 2.0f;
  float yAvgDelta = (float)(calibration[1][1] - calibration[1][0]) / 2.0f;
  float zAvgDelta = (float)(calibration[2][1] - calibration[2][0]) / 2.0f;

  float avg_delta = (xAvgDelta + yAvgDelta + zAvgDelta) / 3.0f;

  xScale = (float)avg_delta / xAvgDelta;
  yScale = (float)avg_delta / yAvgDelta;
  zScale = (float)avg_delta / zAvgDelta;
}

/* Read raw mag data. */
void QMC5883Compass::readRaw(float* destination) { memcpy(destination, raw, sizeof(raw)); }

/* Read calibrated mag data. */
void QMC5883Compass::readCalibrated(float* destination) { memcpy(destination, calibrated, sizeof(calibrated)); }

/* Set the address of the mag. */
void QMC5883Compass::setADDR(byte b) { _ADDR = b; }

/* Write a single register on the mag. */
void QMC5883Compass::_writeReg(byte r, byte v) {
  Wire.beginTransmission(_ADDR);
  Wire.write(r);
  Wire.write(v);
  Wire.endTransmission();
}

/* Create the compass object based on the whoamI register. */

std::unique_ptr<QMC5883Compass> CreateCompass::createSensor() {
  CompassModel selection = SelectCompass();
  switch (selection) {
    case CompassModel::kQMC5883L:
      return std::make_unique<QMC5883LCompass>();
    case CompassModel::kQMC5883P:
      return std::make_unique<QMC5883PCompass>();
    case CompassModel::kInvalid:
    default:
      return std::make_unique<QMC5883LCompass>();
  }
}
