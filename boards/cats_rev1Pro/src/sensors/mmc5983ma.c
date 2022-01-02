/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "sensors/mmc5983ma.h"
#include <stdbool.h>

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

#define MMC5983MA_OFFSET 131072.0f
#define MMC5983MA_RES    (1.0f / 16384.0f)

static void write_register(SPI_BUS *spi, uint8_t reg, uint8_t data);
static void read_data(SPI_BUS *spi, uint8_t reg, uint8_t *data, uint32_t length);

void mmc5983ma_init(const MMC5983MA *dev) {
  // enable data ready interrupt (bit2 == 1), enable auto set/reset (bit 5 == 1)
  // this set/reset is a low current sensor offset measurement for normal use
  write_register(dev->spi, MMC5983MA_CONTROL_0, 0x20 | 0x04);

  // set magnetometer bandwidth
  write_register(dev->spi, MMC5983MA_CONTROL_1, dev->bandwidth);

  // enable continuous measurement mode (bit 3 == 1), set sample rate
  // enable automatic Set/Reset (bit 7 == 1), set set/reset rate
  // this set/reset is a high-current "deGaussing" that should be used only to recover from
  // high magnetic field detuning of the magnetoresistive film
  write_register(dev->spi, MMC5983MA_CONTROL_2, 0x80 | (dev->setreset << 4) | 0x08 | dev->sample_rate);
}

void mmc5983ma_read_raw(const MMC5983MA *dev, uint32_t *destination) {
  uint8_t rawData[7];                                     // x/y/z mag register data stored here
  read_data(dev->spi, MMC5983MA_XOUT_0, &rawData[0], 7);  // Read the 7 raw data registers into data array
  destination[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 |
                              (rawData[6] & 0xC0) >> 6);  // Turn the 18 bits into an unsigned 32-bit value
  destination[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 |
                              (rawData[6] & 0x30) >> 4);  // Turn the 18 bits into an unsigned 32-bit value
  destination[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 |
                              (rawData[6] & 0x0C) >> 2);  // Turn the 18 bits into an unsigned 32-bit value
}

void mmc5983ma_read_real(const MMC5983MA *dev, float *destination) {
  uint32_t tmp[3];
  mmc5983ma_read_raw(dev, tmp);
  for (int i = 0; i < 3; i++) {
    destination[i] = ((float)tmp[i] - MMC5983MA_OFFSET) * MMC5983MA_RES;
  }
}

void mmc5983ma_read_calibrated(const MMC5983MA *dev, float *destination) {
  float tmp[3];
  mmc5983ma_read_real(dev, tmp);
  for (int i = 0; i < 3; i++) {
    destination[i] = (tmp[i] - dev->mag_bias[i]) * dev->mag_scale[i];
  }
}

bool mmc5983ma_selftest(const MMC5983MA *dev) {
  uint32_t data_set[3] = {0}, data_reset[3] = {0};

  // Reset registers
  write_register(dev->spi, MMC5983MA_CONTROL_0, 0x00);
  write_register(dev->spi, MMC5983MA_CONTROL_1, 0x00);
  write_register(dev->spi, MMC5983MA_CONTROL_2, 0x00);

  // SET current
  write_register(dev->spi, MMC5983MA_CONTROL_0, 0x08);
  HAL_Delay(1);
  // One time read
  write_register(dev->spi, MMC5983MA_CONTROL_0, 0x01);
  HAL_Delay(10);
  mmc5983ma_read_raw(dev, data_set);

  // RESET current
  write_register(dev->spi, MMC5983MA_CONTROL_0, 0x10);
  HAL_Delay(1);
  // One time read
  write_register(dev->spi, MMC5983MA_CONTROL_0, 0x01);
  HAL_Delay(10);
  mmc5983ma_read_raw(dev, data_reset);

  for (int i = 0; i < 3; i++) {
    uint32_t delta;
    if (data_set[i] > data_reset[i])
      delta = data_set[i] - data_reset[i];
    else
      delta = data_reset[i] - data_set[i];

    if (delta < 100) return false;
  }
  return true;
}

void mmc5983_calibration(MMC5983MA *dev) {
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int32_t mag_max[3] = {-262143, -262143, -262143}, mag_min[3] = {262143, 262143, 262143};
  uint32_t mag_temp[3] = {0, 0, 0}, mag_offset = 131072;

  for (int ii = 0; ii < 4000; ii++) {
    mmc5983ma_read_raw(dev, mag_temp);
    for (int jj = 0; jj < 3; jj++) {
      if ((int32_t)(mag_temp[jj] - mag_offset) > mag_max[jj]) mag_max[jj] = (int32_t)(mag_temp[jj] - mag_offset);
      if ((int32_t)(mag_temp[jj] - mag_offset) < mag_min[jj]) mag_min[jj] = (int32_t)(mag_temp[jj] - mag_offset);
    }
    HAL_Delay(12);
  }

  // Get hard iron correction
  mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
  mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
  mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

  dev->mag_bias[0] = (float)(mag_bias[0]) * MMC5983MA_RES;  // save mag biases in G for main program
  dev->mag_bias[1] = (float)(mag_bias[1]) * MMC5983MA_RES;
  dev->mag_bias[2] = (float)(mag_bias[2]) * MMC5983MA_RES;

  // Get soft iron correction estimate
  mag_scale[0] = (mag_max[0] - mag_min[0]) / 2;  // get average x axis max chord length in counts
  mag_scale[1] = (mag_max[1] - mag_min[1]) / 2;  // get average y axis max chord length in counts
  mag_scale[2] = (mag_max[2] - mag_min[2]) / 2;  // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0f;

  dev->mag_scale[0] = avg_rad / ((float)mag_scale[0]);
  dev->mag_scale[1] = avg_rad / ((float)mag_scale[1]);
  dev->mag_scale[2] = avg_rad / ((float)mag_scale[2]);
  HAL_Delay(1);
}

static void write_register(SPI_BUS *spi, uint8_t reg, uint8_t data) {
  uint8_t tmp[2] = {reg, data};
  spi_transmit(spi, tmp, 2);
}

static void read_data(SPI_BUS *spi, uint8_t reg, uint8_t *data, uint32_t length) {
  reg = reg | 0x80;
  spi_transmit_receive(spi, &reg, 1, data, length);
}
