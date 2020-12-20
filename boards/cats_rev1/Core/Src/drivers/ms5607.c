// MS5607 Barometer Device Library
// Author: Luca Jost
// 11.06.2020

#include "drivers/ms5607.h"

// *** Macros *** //

#define UINT8_TO_UINT16(dst, src_high, src_low) \
  do {                                          \
    dst = (src_high);                           \
    dst <<= 8;                                  \
    dst |= (src_low);                           \
  } while (0);

// *** Local functions *** //

uint32_t _get_conversion_ticks(struct ms5607_dev *dev) {
  uint32_t time;
  time =
      (BARO_CONVERSION_TIME_OSR_BASE * (dev->osr + 1) * osKernelGetTickFreq()) /
      1000;
  if (time < 2) time = 2;
  return time;
}

// Read bytes
void _ms_read_bytes(struct ms5607_dev *dev, uint8_t command, uint8_t *pData,
                    uint16_t size) {
  HAL_I2C_Master_Transmit(dev->i2c_bus, dev->i2c_address, &command, 1,
                          BARO_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(dev->i2c_bus, dev->i2c_address, pData, size,
                         BARO_I2C_TIMEOUT);
}

// Write command
void _ms_write_command(struct ms5607_dev *dev, uint8_t command) {
  HAL_I2C_Master_Transmit(dev->i2c_bus, dev->i2c_address, &command, 1,
                          BARO_I2C_TIMEOUT);
}

void _read_calibration(struct ms5607_dev *dev) {
  for (int i = 0; i < 6; i++) {
    uint8_t rec[2] = {0};
    _ms_read_bytes(dev, COMMAND_PROM_READ_BASE + (2 * (i + 1)), rec, 2);
    UINT8_TO_UINT16(dev->coefficients[i], rec[0], rec[1]);
  }
}

// *** Global functions *** //

void ms5607_init(struct ms5607_dev *dev) {
  uint32_t reset_time;
  reset_time = 3 * osKernelGetTickFreq() / 1000;
  // General Procedure:
  //  1. reset chip
  //  2. Read out calibration

  // Reset chip
  _ms_write_command(dev, COMMAND_RESET);
  osDelay(reset_time);

  // Read calibration
  _read_calibration(dev);
}

void ms5607_read_raw_pres_temp(struct ms5607_dev *dev, int32_t *pressure_raw,
                               int32_t *temperature_raw) {
  uint32_t wait_time;
  uint8_t command;
  uint8_t rec[3] = {0};

  // figure out how many ticks a conversion needs
  wait_time = _get_conversion_ticks(dev);

  // initiate pressure conversion
  command = COMMAND_CONVERT_D1_BASE + (dev->osr * 2);
  _ms_write_command(dev, command);

  // wait till the conversion is done
  osDelay(wait_time);

  // read out raw pressure value
  _ms_read_bytes(dev, COMMAND_ADC_READ, rec, 3);
  *pressure_raw = (rec[0] << 16) | (rec[1] << 8) | rec[2];

  command = COMMAND_CONVERT_D2_BASE + (dev->osr * 2);
  _ms_write_command(dev, command);

  // wait till the conversion is done
  osDelay(wait_time);

  // read out raw pressure value
  _ms_read_bytes(dev, COMMAND_ADC_READ, rec, 3);
  *temperature_raw = (rec[0] << 16) | (rec[1] << 8) | rec[2];
}

void ms5607_read_pres_temp(struct ms5607_dev *dev, int32_t *temperature,
                           int32_t *pressure) {
  int32_t pressure_raw;
  int32_t temperature_raw;

  ms5607_read_raw_pres_temp(dev, &pressure_raw, &temperature_raw);

  // Calculate real values with coefficients
  int64_t dT;
  int64_t OFF, SENS;

  dT = temperature_raw - ((int32_t)dev->coefficients[4] << 8);
  /* Temperature in 2000  = 20.00° C */
  *temperature = (int32_t)2000 + (dT * dev->coefficients[5] >> 23);

  OFF = ((int64_t)dev->coefficients[1] << 17) +
        ((dev->coefficients[3] * dT) >> 6);
  SENS = ((int64_t)dev->coefficients[0] << 16) +
         ((dev->coefficients[2] * dT) >> 7);
  /* Pressure in 110002 = 1100.02 mbar */
  *pressure = (int32_t)((((pressure_raw * SENS) >> 21) - OFF) >> 15);
}
