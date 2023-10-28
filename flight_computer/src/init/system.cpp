/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "system.hpp"
#include "lfs.h"

#include "drivers/w25q.hpp"
#include "flash/lfs_custom.hpp"

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
      const int err2 = lfs_mount(&lfs, get_lfs_cfg());
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
    lfs_mkdir(&lfs, "configs");

    strncpy(cwd, "/", sizeof(cwd));

    log_info("LFS mounted successfully!");
  }
}
