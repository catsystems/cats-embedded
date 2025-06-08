/// Copyright (C) 2020, 2025 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "Arduino.h"
#include "QMC5883Compass.hpp"
#include "Wire.h"

class QMC5883PCompass : public QMC5883Compass {
 public:
  QMC5883PCompass();

  void init() override;
  void read() override;

 private:
  void setReset(byte rng);
  void setMode(byte mode, byte odr, byte rng, byte osr);

  byte _READ_START_ADDR = 0x01;
  byte _CTRL_REG_1 = 0x0A;
  byte _CTRL_REG_2 = 0x0B;
};
