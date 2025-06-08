
#pragma once

#include "Arduino.h"
#include "QMC5883Compass.hpp"
#include "Wire.h"

class QMC5883LCompass : public QMC5883Compass {
 public:
  QMC5883LCompass();

  void init() override;
  void read() override;

 private:
  void setReset();
  void setMode(byte mode, byte odr, byte rng, byte osr);
};
