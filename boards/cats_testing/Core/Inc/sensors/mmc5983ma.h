/*
 * mmc5983ma.h
 *
 *  Created on: Jun 16, 2021
 *      Author: Luca
 */

#pragma once

#include "drivers/spi.h"

#define MMC5983MA_XOUT_0     0x00
#define MMC5983MA_XOUT_1     0x01
#define MMC5983MA_YOUT_0     0x02
#define MMC5983MA_YOUT_1     0x03
#define MMC5983MA_ZOUT_0     0x04
#define MMC5983MA_ZOUT_1     0x05
#define MMC5983MA_XYZOUT_2   0x06
#define MMC5983MA_TOUT       0x07
#define MMC5983MA_STATUS     0x08
#define MMC5983MA_CONTROL_0  0x09
#define MMC5983MA_CONTROL_1  0x0A
#define MMC5983MA_CONTROL_2  0x0B
#define MMC5983MA_CONTROL_3  0x0C
#define MMC5983MA_PRODUCT_ID 0x2F  // Answer should be 0x30

enum mmc5983ma_sample_rate {
  MMC5983MA_ODR_ONESHOT = 0x00,
  MMC5983MA_ODR_1Hz = 0x01,
  MMC5983MA_ODR_10Hz = 0x02,
  MMC5983MA_ODR_20Hz = 0x03,
  MMC5983MA_ODR_50Hz = 0x04,
  MMC5983MA_ODR_100Hz = 0x05,
  MMC5983MA_ODR_200Hz = 0x06,   // BW = 0x01 only
  MMC5983MA_ODR_1000Hz = 0x07,  // BW = 0x11 only
};

enum mmc5983ma_bandwith {
  MMC5983MA_BW_100Hz = 0x00,  // 8 ms measurement time
  MMC5983MA_BW_200Hz = 0x01,  // 4 ms
  MMC5983MA_BW_400Hz = 0x02,  // 2 ms
  MMC5983MA_BW_800Hz = 0x03,  // 0.5 ms
};

enum mmc5983ma_setreset {
  MMC5983MA_SET_1 = 0x00,   // Set/Reset each data measurement
  MMC5983MA_SET_25 = 0x01,  // each 25 data measurements
  MMC5983MA_SET_75 = 0x02,
  MMC5983MA_SET_100 = 0x03,
  MMC5983MA_SET_250 = 0x04,
  MMC5983MA_SET_500 = 0x05,
  MMC5983MA_SET_1000 = 0x06,
  MMC5983MA_SET_2000 = 0x07,
};

#define MMC5983MA_OFFSET 131072.0f
#define MMC5983MA_RES    (1.0f / 16384.0f)

typedef struct mmc5983ma_dev {
  // Hardware Configuration
  SPI_BUS *spi;
  // Sensor Configuration
  enum mmc5983ma_sample_rate sample_rate;
  enum mmc5983ma_bandwith bandwidth;
  enum mmc5983ma_setreset setreset;
  // Calibraion Data
  float mag_bias[3];   // Hard iron offset
  float mag_scale[3];  // Soft iron
} MMC5983MA;

void mmc5983ma_init(const MMC5983MA *dev);
bool mmc5983ma_selftest(const MMC5983MA *dev);
void mmc5983_calibration(MMC5983MA *dev);
void mmc5983ma_read_raw(const MMC5983MA *dev, uint32_t *destination);
void mmc5983ma_read_real(const MMC5983MA *dev, float *destination);
void mmc5983ma_read_calibrated(const MMC5983MA *dev, float *destination);
