/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
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

#include <optional>

#include "cli/cli.hpp"
#include "drivers/w25q.hpp"
#include "lfs.h"

static int w25q_lfs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
static int w25q_lfs_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer,
                         lfs_size_t size);
static int w25q_lfs_erase(const struct lfs_config *c, lfs_block_t block);
static int w25q_lfs_sync(const struct lfs_config *c);

#define LFS_CACHE_SIZE     512
#define LFS_LOOKAHEAD_SIZE 512

/* LFS Static Buffers */
static uint8_t read_buffer[LFS_CACHE_SIZE] = {};
static uint8_t prog_buffer[LFS_CACHE_SIZE] = {};
static uint8_t lookahead_buffer[LFS_LOOKAHEAD_SIZE] = {};

/* File System Handle  -- NOT THREAD-SAFE!!! */
lfs_t lfs;

std::optional<lfs_config> lfs_cfg;
void init_lfs_cfg(const w25q_t *w25q_ptr) {
  /* Flash must be initialized before initializing LFS */
  assert(w25q_ptr->initialized);
  /* Blocks in LFS correspond to Sectors on W25Q chips. */
  lfs_cfg.emplace(lfs_config({// block device operations
                              .read = w25q_lfs_read,
                              .prog = w25q_lfs_prog,
                              .erase = w25q_lfs_erase,
                              .sync = w25q_lfs_sync,
                              // block device configuration
                              .read_size = w25q_ptr->page_size,
                              .prog_size = w25q_ptr->page_size,
                              .block_size = w25q_ptr->sector_size,
                              .block_count = w25q_ptr->sector_count,
                              .block_cycles = 500,
                              .cache_size = LFS_CACHE_SIZE,
                              .lookahead_size = LFS_LOOKAHEAD_SIZE,
                              .read_buffer = read_buffer,
                              .prog_buffer = prog_buffer,
                              .lookahead_buffer = lookahead_buffer}));
}

const lfs_config *get_lfs_cfg() noexcept {
  if (!lfs_cfg.has_value()) {
    init_lfs_cfg(&w25q);
  }
  assert(lfs_cfg.has_value());
  return &(lfs_cfg.value());
};

char cwd[LFS_NAME_MAX] = {};

uint32_t flight_counter = 0;

// static uint8_t fc_file_cfg_buffer[LFS_CACHE_SIZE] = {};
// struct lfs_file_config fc_file_cfg = {.buffer = fc_file_cfg_buffer, .attr_count = 1};
lfs_file_t fc_file /* = {.cfg = &fc_file_cfg} */;

int8_t lfs_obj_type(const char *path) {
  struct lfs_info info;
  int32_t stat_err = lfs_stat(&lfs, path, &info);
  if (stat_err < 0) {
    // cli_print_linef("lfs_stat failed with error: %ld", stat_err);
    return -1;
  }
  /* casting here is fine because info.type should be 0x1 (LFS_TYPE_REG) or 0x2 (LFS_TYPE_FILE) */
  return (int8_t)info.type;
}

int lfs_ls(const char *path) {
  lfs_dir_t dir;
  int err = lfs_dir_open(&lfs, &dir, path);
  if (err) {
    return err;
  }

  struct lfs_info info;
  while (true) {
    int res = lfs_dir_read(&lfs, &dir, &info);
    if (res < 0) {
      return res;
    }

    if (res == 0) {
      break;
    }

    switch (info.type) {
      case LFS_TYPE_REG:
        cli_print("file ");
        break;
      case LFS_TYPE_DIR:
        cli_print(" dir ");
        break;
      default:
        cli_print("   ? ");
        break;
    }

    static const char *prefixes[] = {"", "K", "M", "G"};
    if (info.type == LFS_TYPE_REG) {
      for (int i = sizeof(prefixes) / sizeof(prefixes[0]) - 1; i >= 0; i--) {
        if (info.size >= (1U << 10 * i) - 1) {
          cli_printf("%*lu%sB ", 4 - (i != 0), info.size >> 10 * i, prefixes[i]);
          break;
        }
      }
    } else {
      cli_print("      ");
    }

    cli_printf("%s\n", info.name);
  }

  err = lfs_dir_close(&lfs, &dir);
  if (err) {
    return err;
  }

  return 0;
}

int32_t lfs_cnt(const char *path, enum lfs_type type) {
  if (type != LFS_TYPE_REG && type != LFS_TYPE_DIR) {
    return -1;
  }

  int32_t cnt = 0;

  lfs_dir_t dir;
  int err = lfs_dir_open(&lfs, &dir, path);
  if (err) {
    return err;
  }

  struct lfs_info info;
  /* Iterate over all the files in the current directory */
  while (true) {
    int32_t res = lfs_dir_read(&lfs, &dir, &info);
    if (res < 0) {
      return res;
    }

    if (res == 0) {
      break;
    }

    /* Increment the counter if the type matches */
    if (info.type == type) {
      ++cnt;
    }
  }

  err = lfs_dir_close(&lfs, &dir);
  if (err) {
    return err;
  }

  return cnt;
}

static int w25q_lfs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
  if (w25q_read_sector((uint8_t *)buffer, block, off, size) == W25Q_OK) {
    return 0;
  }
  return LFS_ERR_CORRUPT;
}
static int w25q_lfs_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer,
                         lfs_size_t size) {
  static uint32_t sync_counter = 0;
  static uint32_t sync_counter_err = 0;
  if (w25q_write_sector((uint8_t *)buffer, block, off, size) == W25Q_OK) {
    if (sync_counter % 32 == 0) {
      /* Flash the LED at certain intervals */
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
    ++sync_counter;
    return 0;
  }
  if (sync_counter_err % 32 == 0) {
    /* Flash the LED at certain intervals */
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
  }
  ++sync_counter_err;
  return LFS_ERR_CORRUPT;
}
static int w25q_lfs_erase(const struct lfs_config *c, lfs_block_t block) {
  if (w25q_sector_erase(block) == W25Q_OK) {
    return 0;
  }
  return LFS_ERR_CORRUPT;
}
static int w25q_lfs_sync(const struct lfs_config *c) { return 0; }
