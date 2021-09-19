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

#include "util/reader.h"
#include "util/log.h"
#include "config/globals.h"
#include "config/cats_config.h"
#include "lfs/lfs_custom.h"

void dump_recording(uint16_t number) {
  if (global_recorder_status == REC_WRITE_TO_FLASH) {
    log_raw("The recorder is currently active, stop it first!");
    return;
  }

  char *string_buffer1 = calloc(400, sizeof(char));
  char *string_buffer2 = calloc(400, sizeof(char));
  uint8_t *read_buf = (uint8_t *)calloc(256, sizeof(uint8_t));

  char filename[32] = {};
  snprintf(filename, 32, "flights/flight_%05d", number);

  log_raw("Dumping file: %s", filename);

  lfs_file_t curr_file;
  if (lfs_file_open(&lfs, &curr_file, filename, LFS_O_RDONLY) == LFS_ERR_OK) {
    int file_size = lfs_file_size(&lfs, &curr_file);
    for (lfs_size_t i = 0; i < file_size; i += 256) {
      lfs_size_t chunk = lfs_min(256, file_size - i);

      lfs_file_read(&lfs, &curr_file, read_buf, chunk);

      int write_idx = 0;
      for (uint32_t j = 0; j < 128; ++j) {
        write_idx += sprintf(string_buffer1 + write_idx, "%02x ", read_buf[j]);
      }
      log_rawr("%s", string_buffer1);
      write_idx = 0;
      for (uint32_t j = 128; j < 256; ++j) {
        write_idx += sprintf(string_buffer2 + write_idx, "%02x ", read_buf[j]);
      }
      log_rawr("%s\n", string_buffer2);

      memset(string_buffer1, 0, 400);
      memset(string_buffer2, 0, 400);
    }
  } else {
    log_error("Flight %d not found!", number);
  }

  lfs_file_close(&lfs, &curr_file);

  free(string_buffer1);
  free(string_buffer2);
}

void parse_recording(uint16_t number) {
  if (global_recorder_status == REC_WRITE_TO_FLASH) {
    log_raw("The recorder is currently active, stop it first!");
    return;
  }

  char *string_buffer1 = calloc(400, sizeof(char));
  char *string_buffer2 = calloc(400, sizeof(char));
  uint8_t *read_buf = (uint8_t *)calloc(256, sizeof(uint8_t));

  char filename[32] = {};
  snprintf(filename, 32, "flights/flight_%05d", number);

  log_raw("Reading file: %s", filename);

  lfs_file_t curr_file;
  if (lfs_file_open(&lfs, &curr_file, filename, LFS_O_RDONLY) == LFS_ERR_OK) {
    int file_size = lfs_file_size(&lfs, &curr_file);
    lfs_size_t i = 0;
    //    while(i < file_size){
    //      break;
    //    }

    //    for (lfs_size_t i = 0; i < file_size; i += 256) {
    //      lfs_size_t chunk = lfs_min(256, file_size - i);
    //
    //      lfs_file_read(&lfs, &curr_file, read_buf, chunk);
    //
    //      int write_idx = 0;
    //      for (uint32_t j = 0; j < 128; ++j) {
    //        write_idx += sprintf(string_buffer1 + write_idx, "%02x ", read_buf[j]);
    //      }
    //      log_rawr("%s", string_buffer1);
    //      write_idx = 0;
    //      for (uint32_t j = 128; j < 256; ++j) {
    //        write_idx += sprintf(string_buffer2 + write_idx, "%02x ", read_buf[j]);
    //      }
    //      log_rawr("%s\n", string_buffer2);
    //
    //      memset(string_buffer1, 0, 400);
    //      memset(string_buffer2, 0, 400);
    //    }
  } else {
    log_error("Flight %d not found!", number);
  }

  lfs_file_close(&lfs, &curr_file);

  free(string_buffer1);
  free(string_buffer2);
}

void erase_recordings() { /* remove everything from /flights */
}
