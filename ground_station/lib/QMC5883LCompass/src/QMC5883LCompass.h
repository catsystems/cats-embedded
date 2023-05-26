#ifndef QMC5883L_Compass
#define QMC5883L_Compass

#include "Arduino.h"
#include "Wire.h"


class QMC5883LCompass{
	
  public:
    QMC5883LCompass();
	void init();
    void setADDR(byte b);
    void setMode(byte mode, byte odr, byte rng, byte osr);

	void setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max);
    void setReset();
    void read();



	void readRaw(float* destination);
	void readCalibrated(float* destination);
	
  private:
    void _writeReg(byte reg,byte val);
    
	byte _ADDR = 0x0D;

	bool calibrationUse = false;

	float raw[3] = {0,0,0};
	int calibration[3][2];
	float calibrated[3];
	void applyCalibration();

	float xOffset, yOffset, zOffset;
	float xScale, yScale, zScale;

};

#endif
