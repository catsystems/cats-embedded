/// Copyright (C) 2020, 2025 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

/*

FROM QST QMC5883L Datasheet [https://nettigo.pl/attachments/440]
-----------------------------------------------
 MODE CONTROL (MODE)
        Standby			0x00
        Continuous		0x01

OUTPUT DATA RATE (ODR)
        10Hz        	0x00
        50Hz        	0x04
        100Hz       	0x08
        200Hz       	0x0C

FULL SCALE (RNG)
        2G          	0x00
        8G          	0x10

OVER SAMPLE RATIO (OSR)
        512         	0x00
        256         	0x40
        128         	0x80
        64          	0xC0

*/

#include "QMC5883PCompass.hpp"
#include <Wire.h>
#include "Arduino.h"

QMC5883PCompass::QMC5883PCompass() {}

void QMC5883PCompass::init() {
  setADDR(0x2C);
  _writeReg(0x0B, 0x01);
  setMode(0x01, 0x08, 0x00, 0x00);

  // setReset(0x10);
}

void QMC5883PCompass::setMode(byte mode, byte odr, byte osr1, byte osr2) {
  _writeReg(_CTRL_REG_1, mode | odr | osr1 | osr2);
}

void QMC5883PCompass::setReset(byte rng) { _writeReg(_CTRL_REG_2, 0x80 | rng); }

void QMC5883PCompass::read() {
  Wire.beginTransmission(_ADDR);
  Wire.write(_READ_START_ADDR);
  int err = Wire.endTransmission();
  if (!err) {
    Wire.requestFrom(_ADDR, (byte)6);
    raw[0] = (float)(int16_t)(Wire.read() | Wire.read() << 8);
    raw[1] = (float)(int16_t)(Wire.read() | Wire.read() << 8);
    raw[2] = (float)(int16_t)(Wire.read() | Wire.read() << 8);
  }
  if (calibrationUse) {
    calibrated[0] = (raw[0] - xOffset) * xScale;
    calibrated[1] = (raw[1] - yOffset) * yScale;
    calibrated[2] = (raw[2] - zOffset) * zScale;
  }
}
