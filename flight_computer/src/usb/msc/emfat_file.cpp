/*
 * This file was forked from Cleanflight and Betaflight.
 *
 * See the Cleanflight and Betaflight License below.
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

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "emfat.h"
#include "lfs.h"

#include "flash/lfs_custom.hpp"
#include "util/log.h"

#define CMA_TIME EMFAT_ENCODE_CMA_TIME(1, 1, 2023, 13, 0, 0)
#define CMA \
  { CMA_TIME, CMA_TIME, CMA_TIME }

static void lfs_read_file(uint8_t *dest, int size, uint32_t offset, emfat_entry_t *entry) {
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

    // Assume the files starting with 'f' are flight logs; all others are considered to be stats files.
    const bool flight_log = entry->name != NULL && entry->name[0] == 'f';
    snprintf(filename, 32, flight_log ? "/flights/flight_%05hu" : "/stats/stats_%05hu.txt", entry->lfs_flight_idx);
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
    "Welcome to CATS!\r\n\r\n"
    "To get started please visit our website: https://catsystems.io.\r\n\r\n"
    "To erase log files and to plot your flights, please use the CATS Configurator.\r\n\r\n"
    "You can find the latest version on our Github: https://github.com/catsystems/cats-configurator/releases\r\n\r\n"
    "The number of logs exposed via Mass Storage Controller is limited to 50 flight log files and 50 stats files.\r\n";
#define README_SIZE_BYTES (sizeof(readme_file) - 1)

#define PREDEFINED_ENTRY_COUNT 2
#define README_FILE_IDX        1

// We are limited to 50 flight logs & 50 stats files due to RAM memory limits
// TODO: It seems the number has to be 1 more than the actual limit, this should be investigated
constexpr uint32_t kMaxNumVisibleLogs = 100;
static_assert(kMaxNumVisibleLogs > 0 && kMaxNumVisibleLogs % 2 == 0,
              "Maximum number of visible logs has to be divisible by 2!");

#define EMFAT_MAX_ENTRY (PREDEFINED_ENTRY_COUNT + kMaxNumVisibleLogs)

static char logNames[EMFAT_MAX_ENTRY][8 + 1 + 3 + 1] = {"", "readme.txt"};

static const emfat_entry_t entriesPredefined[] = {{
                                                      logNames[0],  // name
                                                      true,         // dir
                                                      ATTR_DIR,     // attr
                                                      0,            // level
                                                      0,            // offset
                                                      0,            // number
                                                      0,            // curr_size
                                                      0,            // max_size
                                                      0,            // user_data
                                                      CMA,          // cma_time[3]
                                                      NULL,         // readcb
                                                      NULL          // writecb
                                                  },
                                                  {
                                                      logNames[1],        // name
                                                      false,              // dir
                                                      ATTR_READ,          // attr
                                                      1,                  // level
                                                      0,                  // offset
                                                      0,                  // number
                                                      README_SIZE_BYTES,  // curr_size
                                                      README_SIZE_BYTES,  // max_siize
                                                      (long)readme_file,  // user_data
                                                      CMA,                // cma_time[3]
                                                      memory_read_proc,   // readcb
                                                      NULL                // writecb
                                                  }};

static emfat_entry_t entries[EMFAT_MAX_ENTRY]{};

emfat_t emfat;
static uint32_t cmaTime = CMA_TIME;

static void emfat_set_entry_cma(emfat_entry_t *entry) {
  // Set file creation/modification/access times to be the same, either the default date or that from the RTC
  // In practise this will be when the filesystem is mounted as the date is passed from the host over USB
  entry->cma_time[0] = cmaTime;
  entry->cma_time[1] = cmaTime;
  entry->cma_time[2] = cmaTime;
}

typedef enum { FLIGHT_LOG, STATS_LOG } log_type_e;

static void emfat_add_log(emfat_entry_t *entry, uint32_t size, const char *name, log_type_e log_type) {
  const uint64_t entry_idx = entry - entries;

  uint16_t lfs_flight_idx = 0;
  int idx_start = log_type == FLIGHT_LOG ? 7 : 6;

  // flight_000xx, stats_000xx.txt
  if (sscanf(&name[idx_start], "%hu", &lfs_flight_idx) > 0) {
    log_error("Reading lfs_flight_idx failed: %hu", lfs_flight_idx);
  }

  snprintf(logNames[entry_idx], 12, "%s%03d.%s", log_type == FLIGHT_LOG ? "fl" : "st", (uint8_t)lfs_flight_idx,
           log_type == FLIGHT_LOG ? "cfl" : "txt");
  entry->name = logNames[entry_idx];
  entry->level = 1;
  entry->number = entry_idx;
  entry->lfs_flight_idx = lfs_flight_idx;
  entry->curr_size = size;
  entry->max_size = entry->curr_size;
  entry->readcb = lfs_read_file;
  entry->writecb = NULL;
  emfat_set_entry_cma(entry);
}

/**
 * @brief Add file from path, returns 0 on success.
 */
static void add_logs_from_path(emfat_entry_t **entry, const char *path, log_type_e log_type, uint32_t max_logs_to_add) {
  struct lfs_info info;
  lfs_dir_t dir;
  int err = lfs_dir_open(&lfs, &dir, path);
  if (err < 0) {
    return;
  }

  // read twice because '.' and '..' are read first
  lfs_dir_read(&lfs, &dir, &info);
  lfs_dir_read(&lfs, &dir, &info);

  uint32_t curr_log_idx = 0;
  while (++curr_log_idx <= max_logs_to_add) {
    if (lfs_dir_read(&lfs, &dir, &info) <= 0) {
      break;
    }
    emfat_add_log((*entry), info.size, info.name, log_type);
    // Move to next entry in the array
    ++(*entry);
  }

  lfs_dir_close(&lfs, &dir);

  return;
}

static void emfat_find_logs(emfat_entry_t *entry) {
  constexpr uint32_t max_logs_to_add_per_file_type = kMaxNumVisibleLogs / 2;

  add_logs_from_path(&entry, "/flights/", FLIGHT_LOG, max_logs_to_add_per_file_type);
  add_logs_from_path(&entry, "/stats/", STATS_LOG, max_logs_to_add_per_file_type);
}

enum class InitState { kNotInitialized, kInitFailed, kInitSucceeded };

/**
 * @return true on success, false on failure
 */
extern "C" bool emfat_init_files() {
  static bool initialized = false;
  static InitState init_state = InitState::kNotInitialized;

  if (initialized) {
    return init_state == InitState::kInitSucceeded;
  }

  memset(entries, 0, sizeof(entries));

  // create the predefined entries
  for (size_t i = 0; i < PREDEFINED_ENTRY_COUNT; i++) {
    entries[i] = entriesPredefined[i];
    // These entries have timestamps corresponding to when the filesystem is mounted
    emfat_set_entry_cma(&entries[i]);
  }

  // Detect and create entries for each individual log
  emfat_find_logs(&entries[PREDEFINED_ENTRY_COUNT]);

  const lfs_ssize_t curr_sz_blocks = lfs_fs_size(&lfs);
  const lfs_size_t block_size_kb = get_lfs_cfg()->block_size / 1024;
  const lfs_size_t curr_sz_kb = curr_sz_blocks * block_size_kb;
  const lfs_size_t total_sz_kb = block_size_kb * get_lfs_cfg()->block_count;

  /* We only care about the LittleFS size, since readme.txt is in memory. */
  entries[README_FILE_IDX].max_size = (total_sz_kb - curr_sz_kb) * 1024;

  initialized = true;
  init_state = emfat_init(&emfat, "CATS", entries) ? InitState::kInitSucceeded : InitState::kInitFailed;

  return init_state == InitState::kInitSucceeded;
}
