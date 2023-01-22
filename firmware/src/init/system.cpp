/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
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

#include "system.h"
#include "lfs.h"

#include "drivers/w25q.h"
#include "flash/lfs_custom.h"

#include "sensors/icm20601.h"
#include "sensors/ms5607.h"

#if NUM_MAGNETO > 0
#include "sensors/mmc5983ma.h"
#endif

#include "config/globals.h"

static void init_lfs();

void init_storage() {
  /* FLASH */
  w25q_init();
  HAL_Delay(100);
  init_lfs();
}

void init_lfs() {
  {
    /* mount the filesystem */
    int err = lfs_mount(&lfs, get_lfs_cfg());
    if (err != 0) {
      /* reformat if we can't mount the filesystem, this should only happen on the first boot */
      log_error("LFS mounting failed with error %d!", err);
      log_error("Trying LFS format");
      lfs_format(&lfs, get_lfs_cfg());
      int err2 = lfs_mount(&lfs, get_lfs_cfg());
      if (err2 != 0) {
        log_error("LFS mounting failed again with error %d!", err2);
      }
    }

    err = lfs_file_open(&lfs, &fc_file, "flight_counter", LFS_O_RDWR | LFS_O_CREAT);
    if (err != 0) {
      log_error("LFS initialization failed: could not open 'flight_counter' file, error %d", err);
      return;
    }

    /* read how many flights we have */
    if (lfs_file_read(&lfs, &fc_file, &flight_counter, sizeof(flight_counter)) > 0) {
      log_info("Flights found: %lu", flight_counter);
    } else {
      log_info("Flights found: %lu", flight_counter);
      lfs_file_rewind(&lfs, &fc_file);
      lfs_file_write(&lfs, &fc_file, &flight_counter, sizeof(flight_counter));
    }
    lfs_file_close(&lfs, &fc_file);

    /* TODO: create a single function for this, it's used in multiple places */
    /* create the flights directory */
    lfs_mkdir(&lfs, "flights");
    lfs_mkdir(&lfs, "stats");

    strncpy(cwd, "/", sizeof(cwd));

    log_info("LFS mounted successfully!");
  }
}

static void init_imu() {
#if IMU_TYPE == ICM20601_TYPE
  auto imu_init_fun = icm20601_init;
#elif IMU_TYPE == LSM6DSR_TYPE
  auto imu_init_fun = lsm6dsr_init;
#endif

  for (int i = 0; i < NUM_IMU; i++) {
    int32_t timeout_counter = 0;
    while (!imu_init_fun(&IMU_DEV[i])) {
      HAL_Delay(10);
      if (++timeout_counter > 20) {
        log_error("IMU %d initialization failed", i);
        break;
      }
    }
    if (timeout_counter < 20) {
      imu_initialized[i] = true;
    }
  }
  for (int i = 0; i < NUM_ACCELEROMETER; i++) {
    while (!h3lis100dl_init(&ACCEL)) {
      HAL_Delay(10);
      log_error("ACCEL initialization failed");
    }
  }
}

static void init_baro() {
  for (int i = 0; i < NUM_BARO; i++) {
    ms5607_init(&BARO_DEV[i]);
    HAL_Delay(10);
  }
}

static void init_magneto() {
#if NUM_MAGNETO > 0
  spi_init(MAG.spi);
  mmc5983ma_init(&MAG);
  // mmc5983_calibration(&MAG);
#endif
}

static void init_buzzer() {
  buzzer_set_freq(&BUZZER, 3200);
  if (HAL_GPIO_ReadPin(USB_DET_GPIO_Port, USB_DET_Pin)) {
    buzzer_set_volume(&BUZZER, 0);
  } else {
    buzzer_set_volume(&BUZZER, 30);
  }
}

void init_devices() {
  /* IMU */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_SET);
  HAL_Delay(10);
  init_imu();
  HAL_Delay(10);
  /* BARO */
  init_baro();
  HAL_Delay(10);
  /* MAGNETO */
  init_magneto();
  HAL_Delay(10);
  /* BUZZER */
  init_buzzer();
}
