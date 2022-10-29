/*
 * Reefing System Bachelor Thesis Software
 * Copyright (C) 2022 Institute for Microelectronics and Embedded Systems OST
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

#pragma once

#include "cmsis_os.h"
#include "lsm6dsr_reg.h"
#include "target.h"

typedef struct lsm6dsr_dev {
  // Hardware Configuration
  SPI_HandleTypeDef *const spi_handle;

  // Sensor Configuration
  stmdev_ctx_t *dev_ctx;

  lsm6dsr_fs_xl_t accel_range;
  lsm6dsr_odr_xl_t accel_odr;

  lsm6dsr_fs_g_t gyro_range;
  lsm6dsr_odr_g_t gyro_odr;
} LSM6DSR;

bool lsm6dsr_init(LSM6DSR *dev);
void lsm6dsr_shutdown(LSM6DSR *dev);
void lsm6dsr_enable(LSM6DSR *dev);

// Threshold in m/s^2
void lsm6dsr_wakeup_enable(LSM6DSR *dev, uint32_t threshold_ms2);
void lsm6dsr_wakeup_disable(LSM6DSR *dev);

void lsm6dsr_get_accel(LSM6DSR *dev, float *acceleration);
void lsm6dsr_read_gyro_raw(LSM6DSR *dev, int16_t *gyro);
void lsm6dsr_read_accel_raw(LSM6DSR *dev, int16_t *acceleration);
