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
#include "drivers/w25q256.h"

#include "main.h"

void print_recording(uint16_t number) {
  char *string_buffer1 = calloc(400, sizeof(char));
  char *string_buffer2 = calloc(400, sizeof(char));
  uint8_t *read_buf = (uint8_t *)calloc(256, sizeof(uint8_t));
  uint16_t start_sector = CATS_STATUS_SECTOR + 1;
  if (number > 0) start_sector = cs_get_last_sector_of_flight(number - 1) + 1;
  uint16_t current_sector = start_sector;

  uint16_t last_sector_of_flight = cs_get_last_sector_of_flight(number);
  if (last_sector_of_flight != 0) {
    log_raw("Recording of flight #%hu:", number);
    osDelay(2000);

    uint32_t start_page_id = (current_sector * 4096) / 256;
    uint32_t end_page_id = ((last_sector_of_flight + 1) * 4096) / 256;
    uint32_t curr_page_id = start_page_id;
    while (curr_page_id < end_page_id) {
        QSPI_W25Qxx_ReadBuffer(read_buf, curr_page_id * 256, 256);
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

      ++curr_page_id;
      if (curr_page_id % 16 == 0) {
        //        HAL_GPIO_TogglePin(GPIOC, LED_FAULT_Pin);
      }
      memset(string_buffer1, 0, 400);
      memset(string_buffer2, 0, 400);
    }
  } else {
    log_error("Last recorded sector of flight is 0!");
  }
  free(string_buffer1);
  free(string_buffer2);
}

void erase_recordings() {
  for (uint32_t i = CATS_STATUS_SECTOR + 1; i < cs_get_last_recorded_sector() + 1; i++) {
      QSPI_W25Qxx_SectorErase(i);
    log_raw("Erased Sector %lu out of %lu", i, cs_get_last_recorded_sector());
  }
  cs_clear();
  cs_save();
}
