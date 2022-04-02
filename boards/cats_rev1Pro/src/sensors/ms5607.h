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

#pragma once

#include "cmsis_os.h"
#include "drivers/spi.h"

#include <stdbool.h>


/** Exported Types **/

enum ms5607_osr {
  MS5607_OSR_256 = 0,
  MS5607_OSR_512 = 1,
  MS5607_OSR_1024 = 2,
  MS5607_OSR_2048 = 3,
  MS5607_OSR_4096 = 4,
};

enum ms5607_data {
  MS5607_PRESSURE = 1,
  MS5607_TEMPERATURE = 2,
};

typedef struct ms5607_dev {
  // Hardware Configuration
  GPIO_TypeDef *const cs_port;
  uint16_t cs_pin;
  SPI_HandleTypeDef *const spi_handle;
  SPI_BUS *spi_bus;

  // Sensor Configuration
  enum ms5607_osr osr;
  enum ms5607_data data;
  // Calibration coefficients
  uint16_t coefficients[6];

  uint8_t raw_pres[3];
  uint8_t raw_temp[3];
} MS5607;

/** Exported Functions **/

void ms5607_init(MS5607 *dev);

void ms5607_prepare_temp(MS5607 *dev);
void ms5607_prepare_pres(MS5607 *dev);
void ms5607_read_raw(MS5607 *dev);
bool ms5607_get_temp_pres(MS5607 *dev, int32_t *temperature, int32_t *pressure);
