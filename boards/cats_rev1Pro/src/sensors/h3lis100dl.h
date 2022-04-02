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

#pragma once

#include "drivers/spi.h"
#include <stdbool.h>

// CTRL_REG_1
enum h3lis100dl_power_mode {
  H3LIS100DL_PM_PD = 0x00,      // Power Down
  H3LIS100DL_PM_NM_ODR = 0x20,  // Normal Mode ODR
  H3LIS100DL_PM_LP_05 = 0x40,   // Low power 0.5Hz
  H3LIS100DL_PM_LP_1 = 0x60,    // Low power 1Hz
  H3LIS100DL_PM_LP_2 = 0x80,    // Low power 2Hz
  H3LIS100DL_PM_LP_5 = 0xA0,    // Low power 5Hz
  H3LIS100DL_PM_LP_10 = 0xC0,   // Low power 10Hz
};

enum h3lis100dl_sample_rate {
  H3LIS100DL_ODR_50 = 0x00,
  H3LIS100DL_ODR_100 = 0x08,
  H3LIS100DL_ODR_400 = 0x10,
};

// CTRL_REG_2
// High Pass fiter cutoff:
// ft = fs/(6*HPC)
enum h3lis100dl_filter {
  H3LIS100DL_HPC_8 = 0x00,
  H3LIS100DL_HPC_16 = 0x01,
  H3LIS100DL_HPC_32 = 0x02,
  H3LIS100DL_HPC_64 = 0x03,
  H3LIS100DL_NO_FILTER = 0xFF,
};

typedef struct h3lis100dl_dev {
  // Hardware configuration
  SPI_BUS *spi;
  // Sensor configuration
  enum h3lis100dl_power_mode power_mode;
  enum h3lis100dl_sample_rate sample_rate;
  enum h3lis100dl_filter filter;
} H3LIS100DL;

/**
 * Initializes the high g accelerometer
 *
 * @param dev - sensor definition struct
 * @return  true if successful, false otherwise
 */
bool h3lis100dl_init(const H3LIS100DL *dev);

/**
 * Read out the raw sensor data, 3 x 8 bit
 * LSB is 0.78g -> 7.6518m/s^2
 *
 * @param dev - sensor definition struct
 * @param data - pointer where the data will be stored in
 */
void h3lis100dl_read_raw(const H3LIS100DL *dev, int8_t *data);

/**
 * Read out the sensor data scaled to m/s^2
 *
 * @param dev - sensor definition struct
 * @param data - pointer where the data will be stored in
 */
void h3lis100dl_read(const H3LIS100DL *dev, float *data);
