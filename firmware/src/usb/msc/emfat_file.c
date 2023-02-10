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

#include "flash/lfs_custom.h"

#define FILESYSTEM_SIZE_MB 32
#define HDR_BUF_SIZE       32

#define CMA_TIME EMFAT_ENCODE_CMA_TIME(1, 1, 2022, 13, 0, 0)
#define CMA \
  { CMA_TIME, CMA_TIME, CMA_TIME }

static void bblog_read_proc(uint8_t *dest, int size, uint32_t offset, emfat_entry_t *entry) {
  log_raw("hello from read: %p, %d, %lu, %ld, %s", dest, size, offset, entry->number, entry->name);
  // flashfsReadAbs(offset, dest, size);
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
  //  log_raw("num: %ld off: %ld", entry->number, offset);
}

static void bblog_write_proc(const uint8_t *data, int size, uint32_t offset, struct emfat_entry_s *entry) {
  log_raw("hello from write: %p, %d, %lu, %ld, %s", data, size, offset, entry->number, entry->name);
}

static const emfat_entry_t entriesPredefined[] = {
    // name   dir   attr  lvl offset  size             max_size        user                time  read write
    {"", true, 0, 0, 0, 0, 0, 0, CMA, NULL, NULL, 0},
    {"TST"
     "_ALL.BBL",
     0,
     0,
     1,
     0,
     0,
     0,
     0,
     CMA,
     bblog_read_proc,
     bblog_write_proc,
     {0}},
    {"PADDING.TXT", 0, ATTR_HIDDEN, 1, 0, 0, 0, 0, CMA, NULL, NULL, {0}},
};

#define PREDEFINED_ENTRY_COUNT (1)
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

//  log_raw("emfat_add_log: %d, %lu, %s", number, size, name);

  snprintf(logNames[number], 12, "fl%03d.cfl", number);
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
  //  static bool lfs_cnt_called = false;
  //  if (lfs_cnt_called) {
  //    return 0;
  //  }
//  log_raw("hello from find_log: %ld, %s", entry->number, entry->name);
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

    if (err) {
      return 0;
    }

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
  uint32_t flashfsUsedSpace = 0;
  int entryIndex = PREDEFINED_ENTRY_COUNT;
  emfat_entry_t *entry;
  memset(entries, 0, sizeof(entries));


  // create the predefined entries
  for (size_t i = 0; i < PREDEFINED_ENTRY_COUNT; i++) {
    entries[i] = entriesPredefined[i];
    // These entries have timestamps corresponding to when the filesystem is mounted
    emfat_set_entry_cma(&entries[i]);
  }

  uint32_t curr_sz_blocks = lfs_fs_size(&lfs);
  uint32_t block_size_kb = get_lfs_cfg()->block_size;
  flashfsUsedSpace = curr_sz_blocks * block_size_kb;

  // Detect and create entries for each individual log
  const int logCount = emfat_find_log(&entries[PREDEFINED_ENTRY_COUNT], EMFAT_MAX_LOG_ENTRY);

  entryIndex += logCount;

  // Padding file to fill out the filesystem size to FILESYSTEM_SIZE_MB
  if (flashfsUsedSpace * 2 < FILESYSTEM_SIZE_MB * 1024 * 1024) {
    entries[entryIndex] = entriesPredefined[PREDEFINED_ENTRY_COUNT + 1];
    entry = &entries[entryIndex];
    // used space is doubled because of the individual files plus the single complete file
    entry->curr_size = (flashfsUsedSpace * 2);
    entry->max_size = (FILESYSTEM_SIZE_MB * 1024 * 1024);
    // This entry has timestamps corresponding to when the filesystem is mounted
    emfat_set_entry_cma(entry);
  }


//    log_raw("hello from init_files: %ld, %s", entries[entryIndex].number, entries[entryIndex].name);

  emfat_init(&emfat, "CATS", entries);
}
