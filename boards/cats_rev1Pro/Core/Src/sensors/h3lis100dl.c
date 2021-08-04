// H3LIS100DL Device Library
// Author: Luca Jost
// 04.08.2021

#include "sensors/h3lis100dl.h"

static void write_register(SPI_BUS *spi, uint8_t reg, uint8_t data);
static void read_data(SPI_BUS *spi, uint8_t reg, uint8_t *data, uint32_t length);

bool h3lis100dl_init(const H3LIS100DL *dev) {
  // These need to be volatile otherwise it will break with optimizations
  volatile uint8_t tmp;

  // General Procedure:
  //  1. Check connection
  //  2. Set control register 1
  //  3. Set control register 2

  // verify we are able to read from the chip
  uint8_t buffer = 0;

  read_data(dev, H3LIS100DL_WHO_AM_I, &buffer, 1);
  if (buffer != H3LIS100DL_WHO_AM_I_CONST) return false;

  // CTRL_REG_1
  // Select power mode, sample rate and enable all axis
  tmp = dev->power_mode | dev->sample_rate | 0x03;
  write_register(dev->spi, H3LIS100DL_CTRL_REG1, tmp);

  // CTRL_REG_2
  // Put in normal mode, enable the high pass filter
  tmp = dev->filter | 0x10;
  write_register(dev->spi, H3LIS100DL_CTRL_REG2, tmp);

  return true;
}

void h3lis100dl_read_raw(const H3LIS100DL *dev, int8_t *data) {
  read_data(dev->spi, H3LIS100DL_OUT_X, (uint8_t)data[0], 1);
  read_data(dev->spi, H3LIS100DL_OUT_Y, (uint8_t)data[1], 1);
  read_data(dev->spi, H3LIS100DL_OUT_Z, (uint8_t)data[2], 1);
}

void h3lis100dl_read(const H3LIS100DL *dev, float *data) {
  uint8_t tmp[3];
  h3lis100dl_read_raw(dev, tmp);
  for (int i = 0; i < 3; i++) {
    data[i] = (float)((int8_t)tmp[i]) * 7.6518f;
  }
}

static void write_register(SPI_BUS *spi, uint8_t reg, uint8_t data) {
  uint8_t tmp[2] = {reg, data};
  spi_transmit(spi, tmp, 2);
}

static void read_data(SPI_BUS *spi, uint8_t reg, uint8_t *data, uint32_t length) {
  reg = reg | 0x80;
  spi_transmit_receive(spi, &reg, 1, data, length);
}