/*
===============================================================================================================
QMC5883LCompass.h
Library for using QMC5583L series chip boards as a compass.
Learn more at [https://github.com/mprograms/QMC5883LCompass]

Supports:

- Getting values of XYZ axis.
- Calculating Azimuth.
- Getting 16 point Azimuth bearing direction (0 - 15).
- Getting 16 point Azimuth bearing Names (N, NNE, NE, ENE, E, ESE, SE, SSE, S, SSW, SW, WSW, W, WNW, NW, NNW)
- Smoothing of XYZ readings via rolling averaging and min / max removal.
- Optional chipset modes (see below)

===============================================================================================================

v1.0 - June 13, 2019
Written by MRPrograms
Github: [https://github.com/mprograms/]

Release under the GNU General Public License v3
[https://www.gnu.org/licenses/gpl-3.0.en.html]

===============================================================================================================



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

#include "QMC5883LCompass.hpp"
#include <Wire.h>
#include "Arduino.h"

QMC5883LCompass::QMC5883LCompass() {}

void QMC5883LCompass::init() {
  setADDR(0x0D);
  _writeReg(0x0B, 0x01);
  setMode(0x01, 0x08, 0x10, 0X00);
}

void QMC5883LCompass::setMode(byte mode, byte odr, byte rng, byte osr) { _writeReg(0x09, mode | odr | rng | osr); }

void QMC5883LCompass::setReset() { _writeReg(0x0A, 0x80); }

void QMC5883LCompass::read() {
  Wire.beginTransmission(_ADDR);
  Wire.write(0x00);
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
