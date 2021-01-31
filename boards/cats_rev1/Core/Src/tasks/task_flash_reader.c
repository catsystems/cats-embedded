//
// Created by stoja on 31.01.21.
//

#include "util/log.h"
#include "config/cats_config.h"
#include "drivers/w25qxx.h"

#include <stdint.h>
#include <stdlib.h>

void task_flash_reader(void *argument) {
  uint8_t *read_buf = (uint8_t *)calloc(256, sizeof(uint8_t));
  log_trace("starting flash reader task");
  uint16_t start_sector = 1;
  uint16_t end_sector = 5;
  uint32_t start_page_id = w25qxx_sector_to_page(start_sector);
  uint32_t end_page_id = w25qxx_sector_to_page(end_sector + 1);
  uint32_t curr_page_id = start_page_id;
  while (curr_page_id < end_page_id) {
    // log_trace("reading page %lu", curr_page_id);
    w25qxx_read_page(read_buf, curr_page_id, 0, 256);
    for (uint32_t i = 0; i < 256; ++i) {
      log_rawr("%02x ", read_buf[i]);
    }
    log_rawr("\n");
    ++curr_page_id;
  }
}