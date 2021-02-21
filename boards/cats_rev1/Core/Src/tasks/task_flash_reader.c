//
// Created by stoja on 31.01.21.
//

#include "util/log.h"
#include "config/cats_config.h"
#include "drivers/w25qxx.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

void task_flash_reader(void *argument) {
  char *string_buffer1 = calloc(400, sizeof(char));
  char *string_buffer2 = calloc(400, sizeof(char));
  uint8_t *read_buf = (uint8_t *)calloc(256, sizeof(uint8_t));
  uint16_t start_sector = 1;
  uint16_t end_sector = 500;
  log_trace("starting flash reader task from sector %hu to %hu", start_sector,
            end_sector);
  uint32_t start_page_id = w25qxx_sector_to_page(start_sector);
  uint32_t end_page_id = w25qxx_sector_to_page(end_sector + 1);
  uint32_t curr_page_id = start_page_id;
  while (curr_page_id < end_page_id) {
    // log_trace("reading page %lu", curr_page_id);
    w25qxx_read_page(read_buf, curr_page_id, 0, 256);
    int write_idx = 0;
    for (uint32_t i = 0; i < 128; ++i) {
      write_idx += sprintf(string_buffer1 + write_idx, "%02x ", read_buf[i]);
    }
    log_rawr("%s\n", string_buffer1);
    write_idx = 0;
    for (uint32_t i = 128; i < 256; ++i) {
      write_idx += sprintf(string_buffer2 + write_idx, "%02x ", read_buf[i]);
    }
    log_rawr("%s\n", string_buffer2);
    //    for (uint32_t i = 0; i < 256; ++i) {
    //      log_rawr("%02x ", read_buf[i]);
    //    }
    // log_rawr("\n");
    ++curr_page_id;
    memset(string_buffer1, 0, 400);
    memset(string_buffer2, 0, 400);
  }
  free(string_buffer1);
  free(string_buffer2);
}