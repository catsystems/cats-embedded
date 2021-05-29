// MS5607 Barometer Device Library
// Author: Luca Jost
// 11.06.2020

#include "sensors/ms5607.h"
#include "util/types.h"
#include <stdbool.h>

volatile static bool i2c1_busy = false;
volatile static bool i2c2_busy = false;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/** Private Function Declarations **/

// Read bytes
static void ms_read_bytes(MS5607 *dev, uint8_t command, uint8_t *pData, uint16_t size);
static void ms_read_bytes_it(MS5607 *dev, uint8_t command, uint8_t *pData, uint16_t size);
// Write command
static void ms_write_command(MS5607 *dev, uint8_t command);
static void read_calibration(MS5607 *dev);
static void ms5607_read_raw(MS5607 *dev);

/** Exported Function Definitions **/

void ms5607_init(MS5607 *dev) {
  uint32_t reset_time;
  reset_time = 3 * osKernelGetTickFreq() / 1000;
  // General Procedure:
  //  1. reset chip
  //  2. Read out calibration

  // Reset chip
  ms_write_command(dev, COMMAND_RESET);
  osDelay(reset_time);

  // Read calibration
  read_calibration(dev);
}

void ms5607_prepare_temp(MS5607 *dev) {
  uint8_t command;
  dev->data = MS5607_TEMPERATURE;
  command = COMMAND_CONVERT_D2_BASE + (dev->osr * 2);
  ms_write_command(dev, command);
}

void ms5607_prepare_pres(MS5607 *dev) {
  uint8_t command;
  dev->data = MS5607_PRESSURE;
  command = COMMAND_CONVERT_D1_BASE + (dev->osr * 2);
  ms_write_command(dev, command);
}

/**
 *
 * @param dev
 * @param temperature
 * @param pressure
 * @return true if reading successful
 */
bool ms5607_get_temp_pres(MS5607 *dev, int32_t *temperature, int32_t *pressure) {
  if (i2c1_busy || i2c2_busy) return false;
  int64_t OFF, SENS;
  int64_t dT;
  int32_t temp, pres;

  temp = (dev->raw_temp[0] << 16) + (dev->raw_temp[1] << 8) + dev->raw_temp[2];
  dT = temp - ((int32_t)dev->coefficients[4] << 8);
  /* Temperature in 2000  = 20.00° C */
  *temperature = (int32_t)(2000 + (dT * dev->coefficients[5] >> 23));

  pres = (dev->raw_pres[0] << 16) + (dev->raw_pres[1] << 8) + dev->raw_pres[2];
  OFF = ((int64_t)dev->coefficients[1] << 17) + ((dev->coefficients[3] * dT) >> 6);
  SENS = ((int64_t)dev->coefficients[0] << 16) + ((dev->coefficients[2] * dT) >> 7);
  /* Pressure in 110002 = 1100.02 mbar */
  *pressure = (int32_t)((((pres * SENS) >> 21) - OFF) >> 15);
  return true;
}

/**
 *
 * @return true if i2c is busy
 */
bool ms5607_busy() { return i2c1_busy && i2c2_busy; }

/**
 *
 * @param dev
 * @return true if readout successful
 */
bool ms5607_try_readout(MS5607 *dev) {
  if (dev->i2c_bus == &hi2c1) {
    if (i2c1_busy)
      return false;
    else {
      ms5607_read_raw(dev);
      return true;
    }
  } else if (dev->i2c_bus == &hi2c2) {
    if (i2c2_busy)
      return false;
    else {
      ms5607_read_raw(dev);
      return true;
    }
  }
  return false;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &hi2c1) {
    i2c1_busy = false;
  } else if (hi2c == &hi2c2) {
    i2c2_busy = false;
  }
}

/** Private Function Definitions **/

// Read bytes
static void ms_read_bytes(MS5607 *dev, uint8_t command, uint8_t *pData, uint16_t size) {
  HAL_I2C_Master_Transmit(dev->i2c_bus, dev->i2c_address, &command, 1, BARO_I2C_TIMEOUT);
  HAL_I2C_Master_Receive(dev->i2c_bus, dev->i2c_address, pData, size, BARO_I2C_TIMEOUT);
}

static void ms_read_bytes_it(MS5607 *dev, uint8_t command, uint8_t *pData, uint16_t size) {
  HAL_I2C_Master_Transmit(dev->i2c_bus, dev->i2c_address, &command, 1, BARO_I2C_TIMEOUT);
  HAL_I2C_Master_Receive_IT(dev->i2c_bus, dev->i2c_address, pData, size);
}

// Write command
static void ms_write_command(MS5607 *dev, uint8_t command) {
  HAL_I2C_Master_Transmit(dev->i2c_bus, dev->i2c_address, &command, 1, BARO_I2C_TIMEOUT);
}

static void read_calibration(MS5607 *dev) {
  for (int i = 0; i < 6; i++) {
    uint8_t rec[2] = {0};
    ms_read_bytes(dev, COMMAND_PROM_READ_BASE + (2 * (i + 1)), rec, 2);
    dev->coefficients[i] = uint8_to_uint16(rec[0], rec[1]);
  }
}

static void ms5607_read_raw(MS5607 *dev) {
  if (dev->i2c_bus == &hi2c1)
    i2c1_busy = 1;
  else if (dev->i2c_bus == &hi2c2)
    i2c2_busy = 1;
  if (dev->data == MS5607_PRESSURE)
    ms_read_bytes_it(dev, COMMAND_ADC_READ, dev->raw_pres, 3);
  else if (dev->data == MS5607_TEMPERATURE)
    ms_read_bytes_it(dev, COMMAND_ADC_READ, dev->raw_temp, 3);
}
