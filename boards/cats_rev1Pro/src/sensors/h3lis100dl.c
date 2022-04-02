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

/* H3LIS100DL Device Library */

#include "sensors/h3lis100dl.h"

//	REGISTERS
#define H3LIS100DL_WHO_AM_I  0x0F  // Answer should be 0x32
#define H3LIS100DL_CTRL_REG1 0x20
#define H3LIS100DL_CTRL_REG2 0x21
#define H3LIS100DL_CTRL_REG3 0x22
#define H3LIS100DL_CTRL_REG4 0x23
#define H3LIS100DL_CTRL_REG5 0x24

#define H3LIS100DL_FILTER_RESET 0x25
#define H3LIS100DL_REFERENCE    0x26
#define H3LIS100DL_STATUS_REG   0x27

#define H3LIS100DL_OUT_X 0x29
#define H3LIS100DL_OUT_Y 0x2B
#define H3LIS100DL_OUT_Z 0x2D

#define H3LIS100DL_INT_CFG1 0x30
#define H3LIS100DL_INT_SRC1 0x31
#define H3LIS100DL_INT_THS1 0x32
#define H3LIS100DL_INT_DUR1 0x33

#define H3LIS100DL_INT_CFG2 0x34
#define H3LIS100DL_INT_SRC2 0x35
#define H3LIS100DL_INT_THS2 0x36
#define H3LIS100DL_INT_DUR2 0x37

#define H3LIS100DL_WHO_AM_I_CONST 0x32

static void write_register(SPI_BUS *spi, uint8_t reg, uint8_t data);

static void read_data(SPI_BUS *spi, uint8_t reg, uint8_t *data, uint32_t length);

bool h3lis100dl_init(const H3LIS100DL *dev) {
  // This needs to be volatile otherwise it will break with optimizations
  volatile uint8_t tmp;

  // General Procedure:
  //  1. Check connection
  //  2. Set control register 1
  //  3. Set control register 2

  // verify we are able to read from the chip
  uint8_t buffer = 0;

  read_data(dev->spi, H3LIS100DL_WHO_AM_I, &buffer, 1);
  if (buffer != H3LIS100DL_WHO_AM_I_CONST) return false;

  // CTRL_REG_1
  // Select power mode, sample rate and enable all axis
  tmp = (uint8_t)dev->power_mode | (uint8_t)dev->sample_rate | 0x07;
  write_register(dev->spi, H3LIS100DL_CTRL_REG1, tmp);

  // CTRL_REG_2
  // Put in normal mode, enable the high pass filter
  if (dev->filter != H3LIS100DL_NO_FILTER) {
    tmp = dev->filter | 0x10;
    write_register(dev->spi, H3LIS100DL_CTRL_REG2, tmp);
  }

  return true;
}

void h3lis100dl_read_raw(const H3LIS100DL *dev, int8_t *data) {
  read_data(dev->spi, H3LIS100DL_OUT_X, (uint8_t *)&data[0], 1);
  read_data(dev->spi, H3LIS100DL_OUT_Y, (uint8_t *)&data[1], 1);
  read_data(dev->spi, H3LIS100DL_OUT_Z, (uint8_t *)&data[2], 1);
}

void h3lis100dl_read(const H3LIS100DL *dev, float *data) {
  int8_t tmp[3];
  // Readout the raw sensor values
  h3lis100dl_read_raw(dev, tmp);
  // Scale to m/s^2 -> LSB is 0.78g / 7.6518m/s^2
  for (int i = 0; i < 3; i++) {
    data[i] = (float)((int8_t)tmp[i]) * 7.6518f;
  }
}

// Private function to write a register
static void write_register(SPI_BUS *spi, uint8_t reg, uint8_t data) {
  uint8_t tmp[2] = {reg, data};
  spi_transmit(spi, tmp, 2);
}

// Private function to read data
static void read_data(SPI_BUS *spi, uint8_t reg, uint8_t *data, uint32_t length) {
  // Select read mode
  reg = reg | 0x80;
  spi_transmit_receive(spi, &reg, 1, data, length);
}
