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

/* MS5607 Barometer Device Library */

#include "sensors/ms5607.h"
#include "util/types.h"
#include <stdbool.h>
#include "drivers/spi.h"

// Commands
#define COMMAND_RESET           0x1E
#define COMMAND_CONVERT_D1_BASE 0x40
#define COMMAND_CONVERT_D2_BASE 0x50
#define COMMAND_ADC_READ        0x00
#define COMMAND_PROM_READ_BASE  0xA0

// Conversion time
#define BARO_CONVERSION_TIME_OSR_BASE 0.6f

/** Private Function Declarations **/

static uint32_t get_conversion_ticks(MS5607 *dev);
// Read bytes
static void ms_read_bytes(MS5607 *dev, uint8_t command, uint8_t *pData, uint16_t size);
// Write command
static void ms_write_command(MS5607 *dev, uint8_t command);
static void read_calibration(MS5607 *dev);

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
  osDelay(1);
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
  int64_t OFF, SENS;
  int64_t dT;
  int32_t temp, pres;

  temp = (dev->raw_temp[0] << 16) + (dev->raw_temp[1] << 8) + dev->raw_temp[2];
  dT = temp - ((int32_t)dev->coefficients[4] << 8);
  /* Temperature in 2000  = 20.00° C */
  *temperature = (int32_t)2000 + (dT * dev->coefficients[5] >> 23);

  pres = (dev->raw_pres[0] << 16) + (dev->raw_pres[1] << 8) + dev->raw_pres[2];
  OFF = ((int64_t)dev->coefficients[1] << 17) + ((dev->coefficients[3] * dT) >> 6);
  SENS = ((int64_t)dev->coefficients[0] << 16) + ((dev->coefficients[2] * dT) >> 7);
  /* Pressure in 110002 = 1100.02 mbar */
  *pressure = (int32_t)((((pres * SENS) >> 21) - OFF) >> 15);
  return true;
}

/** Private Function Definitions **/

static uint32_t get_conversion_ticks(MS5607 *dev) {
  uint32_t time;
  time = (BARO_CONVERSION_TIME_OSR_BASE * ((float)dev->osr + 1) * osKernelGetTickFreq()) / 1000;
  if (time < 1) time = 1;
  return time;
}

// Read bytes
static void ms_read_bytes(MS5607 *dev, uint8_t command, uint8_t *pData, uint16_t size) {
  spi_transmit_receive(dev->spi_bus, &command, 1, pData, size);
}

// Write command
static void ms_write_command(MS5607 *dev, uint8_t command) { spi_transmit(dev->spi_bus, &command, 1); }

static void read_calibration(MS5607 *dev) {
  for (int i = 0; i < 6; i++) {
    uint8_t rec[2] = {0};
    ms_read_bytes(dev, COMMAND_PROM_READ_BASE + (2 * (i + 1)), rec, 2);
    dev->coefficients[i] = uint8_to_uint16(rec[0], rec[1]);
  }
}

void ms5607_read_raw(MS5607 *dev) {
  if (dev->data == MS5607_PRESSURE)
    ms_read_bytes(dev, COMMAND_ADC_READ, dev->raw_pres, 3);
  else if (dev->data == MS5607_TEMPERATURE)
    ms_read_bytes(dev, COMMAND_ADC_READ, dev->raw_temp, 3);
}
