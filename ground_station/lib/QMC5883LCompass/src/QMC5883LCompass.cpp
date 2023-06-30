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



#include "Arduino.h"
#include "QMC5883LCompass.h"
#include <Wire.h>

QMC5883LCompass::QMC5883LCompass() {
}


void QMC5883LCompass::init(){
	Wire.begin(14,15,400000UL);
	_writeReg(0x0B,0x01);
	setMode(0x01,0x08,0x10,0X00);
}


void QMC5883LCompass::setADDR(byte b){
	_ADDR = b;
}




void QMC5883LCompass::_writeReg(byte r, byte v){
	Wire.beginTransmission(_ADDR);
	Wire.write(r);
	Wire.write(v);
	Wire.endTransmission();
}


void QMC5883LCompass::setMode(byte mode, byte odr, byte rng, byte osr){
	_writeReg(0x09,mode|odr|rng|osr);
}


void QMC5883LCompass::setReset(){
	_writeReg(0x0A,0x80);
}


void QMC5883LCompass::setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max){
	calibrationUse = true;

	calibration[0][0] = x_min;
	calibration[0][1] = x_max;
	calibration[1][0] = y_min;
	calibration[1][1] = y_max;
	calibration[2][0] = z_min;
	calibration[2][1] = z_max;

	applyCalibration();
}


void QMC5883LCompass::read(){
	Wire.beginTransmission(_ADDR);
	Wire.write(0x00);
	int err = Wire.endTransmission();
	if (!err) {
		Wire.requestFrom(_ADDR, (byte)6);
		raw[0] = (float)(int16_t)(Wire.read() | Wire.read() << 8);
		raw[1] = (float)(int16_t)(Wire.read() | Wire.read() << 8);
		raw[2] = (float)(int16_t)(Wire.read() | Wire.read() << 8);
	}
	if(calibrationUse){
		calibrated[0] = (raw[0] - xOffset) * xScale;
		calibrated[1] = (raw[1] - yOffset) * yScale;
		calibrated[2] = (raw[2] - zOffset) * zScale;
	}
}


void QMC5883LCompass::applyCalibration(){
	xOffset = (float)(calibration[0][0] + calibration[0][1])/2.0f;
	yOffset = (float)(calibration[1][0] + calibration[1][1])/2.0f;
	zOffset = (float)(calibration[2][0] + calibration[2][1])/2.0f;
	float xAvgDelta = (float)(calibration[0][1] - calibration[0][0])/2.0f;
	float yAvgDelta = (float)(calibration[1][1] - calibration[1][0])/2.0f;
	float zAvgDelta = (float)(calibration[2][1] - calibration[2][0])/2.0f;

	float avg_delta = (xAvgDelta + yAvgDelta + zAvgDelta) / 3.0f;

	xScale = (float)avg_delta / xAvgDelta;
	yScale = (float)avg_delta / yAvgDelta;
	zScale = (float)avg_delta / zAvgDelta;
}

void QMC5883LCompass::readRaw(float* destination){
	memcpy(destination, raw, sizeof(raw));
}

void QMC5883LCompass::readCalibrated(float* destination){
	memcpy(destination, calibrated, sizeof(calibrated));
}

