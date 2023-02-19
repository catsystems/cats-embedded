/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Author: jflyper@github.com
 */
#include "emfat_file.h"

#include <stdio.h>
#include <string.h>
#include "emfat.h"
#include "lfs.h"

#include "flash/lfs_custom.hpp"
#include "util/log.h"

#define FILESYSTEM_SIZE_MB 32
#define HDR_BUF_SIZE       32

#define CMA_TIME EMFAT_ENCODE_CMA_TIME(1, 1, 2022, 13, 0, 0)
#define CMA \
  { CMA_TIME, CMA_TIME, CMA_TIME }

static void bblog_read_proc(uint8_t *dest, int size, uint32_t offset, emfat_entry_t *entry) {
  char filename[32] = {};
  static lfs_file_t curr_file;
  static int32_t number = -1;
  static bool file_open = false;

  if (number != entry->number) {
    number = entry->number;
    if (file_open) {
      file_open = false;
      lfs_file_close(&lfs, &curr_file);
    }
    snprintf(filename, 32, "/flights/flight_%05ld", entry->number);
    int err = lfs_file_open(&lfs, &curr_file, filename, LFS_O_RDONLY);
    if (err) {
      return;
    }
    file_open = true;
  }
  lfs_file_seek(&lfs, &curr_file, (int32_t)offset, LFS_SEEK_SET);
  lfs_file_read(&lfs, &curr_file, dest, size);
}

static void memory_read_proc(uint8_t *dest, int size, uint32_t offset, emfat_entry_t *entry) {
  int len = 0;
  if (offset > entry->curr_size) {
    return;
  }

  if (offset + size > entry->curr_size) {
    len = entry->curr_size - offset;
  } else {
    len = size;
  }

  memcpy(dest, &((char *)entry->user_data)[offset], len);
}

static const char readme_file[] =
    "Welcome to CATS!\r\nTo get started please visit our website: https://catsystems.io\r\n\r\nTo erase log files "
    "please use the CATS Configurator. You can find the latest version on our Github: "
    "https://github.com/catsystems/cats-configurator/releases";
#define README_SIZE (sizeof(readme_file) - 1)

static const emfat_entry_t entriesPredefined[] = {
    // name   dir   attr  lvl offset  size             max_size        user                time  read write
    {"", true, 0, 0, 0, 0, 0, 0, CMA, NULL, NULL, 0},
    {"readme.txt", false, 0, 1, 0, 0, README_SIZE, 1024 * 1024, (long)readme_file, CMA, memory_read_proc, NULL, 0},
    {"LOGS", false, 0, 1, 0, 0, 0, 0, CMA, bblog_read_proc, NULL, 0}};

#define PREDEFINED_ENTRY_COUNT (2)
#define APPENDED_ENTRY_COUNT   2

#define EMFAT_MAX_LOG_ENTRY 100
#define EMFAT_MAX_ENTRY     (PREDEFINED_ENTRY_COUNT + EMFAT_MAX_LOG_ENTRY + APPENDED_ENTRY_COUNT)

static emfat_entry_t entries[EMFAT_MAX_ENTRY];

emfat_t emfat;
static uint32_t cmaTime = CMA_TIME;

static void emfat_set_entry_cma(emfat_entry_t *entry) {
  // Set file creation/modification/access times to be the same, either the default date or that from the RTC
  // In practise this will be when the filesystem is mounted as the date is passed from the host over USB
  entry->cma_time[0] = cmaTime;
  entry->cma_time[1] = cmaTime;
  entry->cma_time[2] = cmaTime;
}

static void emfat_add_log(emfat_entry_t *entry, int number, uint32_t size, char *name) {
  static char logNames[EMFAT_MAX_LOG_ENTRY][8 + 1 + 3 + 1];

  snprintf(logNames[number], 12, "fl%03d.cfl", (uint8_t)number);
  entry->name = logNames[number];
  entry->level = 1;
  entry->number = number;
  entry->curr_size = size;
  entry->max_size = entry->curr_size;
  entry->readcb = bblog_read_proc;
  entry->writecb = NULL;
  // Set file modification/access times to be the same as the creation time
  entry->cma_time[1] = entry->cma_time[0];
  entry->cma_time[2] = entry->cma_time[0];
}

static int emfat_find_log(emfat_entry_t *entry, int maxCount) {

  int logCount = 0;

  logCount = lfs_cnt("/flights/", LFS_TYPE_REG);
  //  lfs_cnt_called = true;
  if (logCount < 1 || logCount > maxCount) return 0;

  struct lfs_info info;
  lfs_dir_t dir;

  int err = lfs_dir_open(&lfs, &dir, "/flights/");
  if (err) {
    return 0;
  }

  for (int i = 0; i < logCount + 2; i++) {
    lfs_dir_read(&lfs, &dir, &info);

    if (i > 1) {
      // Set the default timestamp
      entry->cma_time[0] = cmaTime;

      emfat_add_log(entry++, i - 1, info.size, info.name);
    }
  }

  lfs_dir_close(&lfs, &dir);

  return logCount;
}

void emfat_init_files(void) {
  static bool initialized = false;

  if (initialized) {
    return;
  }

  memset(entries, 0, sizeof(entries));

  // create the predefined entries
  for (size_t i = 0; i < PREDEFINED_ENTRY_COUNT; i++) {
    entries[i] = entriesPredefined[i];
    // These entries have timestamps corresponding to when the filesystem is mounted
    emfat_set_entry_cma(&entries[i]);
  }

  // Detect and create entries for each individual log
  emfat_find_log(&entries[PREDEFINED_ENTRY_COUNT], EMFAT_MAX_LOG_ENTRY);

  initialized = true;
  emfat_init(&emfat, "CATS", entries);
}
