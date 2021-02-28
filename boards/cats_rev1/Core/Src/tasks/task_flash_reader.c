//
// Created by stoja on 31.01.21.
//

#include "util/log.h"
#include "config/cats_config.h"
#include "drivers/w25qxx.h"
#include "main.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

// static void print_sector(uint32_t sector_idx, char *strbuf1, char *strbuf2) {
//  uint32_t start_page_id = w25qxx_sector_to_page(sector_idx);
//  uint32_t end_page_id = w25qxx_sector_to_page(sector_idx + 1);
//  uint32_t curr_page_id = start_page_id;
//  while (curr_page_id < end_page_id) {
//    w25qxx_read_page(read_buf, curr_page_id, 0, 256);
//    int write_idx = 0;
//    for (uint32_t i = 0; i < 128; ++i) {
//      write_idx += sprintf(strbuf1 + write_idx, "%02x ", read_buf[i]);
//    }
//    log_rawr("%s\n", string_buffer1);
//    write_idx = 0;
//    for (uint32_t i = 128; i < 256; ++i) {
//      write_idx += sprintf(string_buffer2 + write_idx, "%02x ", read_buf[i]);
//    }
//    log_rawr("%s\n", string_buffer2);
//    ++curr_page_id;
//    memset(string_buffer1, 0, 400);
//    memset(string_buffer2, 0, 400);
//  }
//}

void task_flash_reader(void *argument) {
  char *string_buffer1 = calloc(400, sizeof(char));
  char *string_buffer2 = calloc(400, sizeof(char));
  uint8_t *read_buf = (uint8_t *)calloc(256, sizeof(uint8_t));
  uint16_t start_sector = CATS_STATUS_SECTOR + 1;
  uint16_t current_sector = start_sector;
  uint16_t num_flights = cs_get_num_recorded_flights();
  uint16_t last_recorded_sector = cs_get_last_recorded_sector();

  log_trace("starting flash reader task from sector %hu to %hu", start_sector,
            last_recorded_sector);

  log_trace("Number of recorded flights: %hu", num_flights);
  for (uint16_t i = 0; i < num_flights; i++) {
    log_trace("Last sectors of flight %hu: %hu", i,
              cs_get_last_sector_of_flight(i));
  }
  for (uint16_t i = 0; i < num_flights; i++) {
    uint16_t last_sector_of_flight = cs_get_last_sector_of_flight(i);
    if (last_sector_of_flight != 0) {
      log_raw("Recording of flight #%hu:", i);
      osDelay(2000);

      uint32_t start_page_id = w25qxx_sector_to_page(current_sector);
      uint32_t end_page_id = w25qxx_sector_to_page(last_sector_of_flight + 1);
      uint32_t curr_page_id = start_page_id;
      while (curr_page_id < end_page_id) {
        w25qxx_read_page(read_buf, curr_page_id, 0, 256);
        int write_idx = 0;
        for (uint32_t j = 0; j < 128; ++j) {
          write_idx +=
              sprintf(string_buffer1 + write_idx, "%02x ", read_buf[j]);
        }
        log_rawr("%s", string_buffer1);
        write_idx = 0;
        for (uint32_t j = 128; j < 256; ++j) {
          write_idx +=
              sprintf(string_buffer2 + write_idx, "%02x ", read_buf[j]);
        }
        log_rawr("%s\n", string_buffer2);

        ++curr_page_id;
        if (curr_page_id % 16 == 0) {
          HAL_GPIO_TogglePin(GPIOC, LED_FAULT_Pin);
        }
        memset(string_buffer1, 0, 400);
        memset(string_buffer2, 0, 400);
      }

      current_sector = last_sector_of_flight + 1;
    } else {
      log_error("Last recorded sector of flight is 0!");
    }
  }
  log_rawr("Flight reader done");
  //  uint32_t start_page_id = w25qxx_sector_to_page(start_sector);
  //  uint32_t end_page_id = w25qxx_sector_to_page(last_recorded_sector + 1);
  //  uint32_t curr_page_id = start_page_id;
  //  while (curr_page_id < end_page_id) {
  //    w25qxx_read_page(read_buf, curr_page_id, 0, 256);
  //    int write_idx = 0;
  //    for (uint32_t i = 0; i < 128; ++i) {
  //      write_idx += sprintf(string_buffer1 + write_idx, "%02x ",
  //      read_buf[i]);
  //    }
  //    log_rawr("%s\n", string_buffer1);
  //    write_idx = 0;
  //    for (uint32_t i = 128; i < 256; ++i) {
  //      write_idx += sprintf(string_buffer2 + write_idx, "%02x ",
  //      read_buf[i]);
  //    }
  //    log_rawr("%s\n", string_buffer2);
  //    ++curr_page_id;
  //    memset(string_buffer1, 0, 400);
  //    memset(string_buffer2, 0, 400);
  //  }

  free(string_buffer1);
  free(string_buffer2);
}