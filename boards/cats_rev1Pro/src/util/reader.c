/*
 * reader.c
 *
 *  Created on: 2 Mar 2021
 *      Author: Luca
 */
#include "util/reader.h"
#include "util/log.h"
#include "config/globals.h"
#include "config/cats_config.h"
#include "lfs/lfs_custom.h"

void dump_recording(uint16_t number) {
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
