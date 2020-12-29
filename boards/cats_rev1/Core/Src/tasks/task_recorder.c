//
// Created by stoja on 26.12.20.
//

#include <stdlib.h>
#include "cmsis_os.h"
#include "tasks/task_recorder.h"
#include "util/log.h"
#include "util/types.h"
#include "drivers/w25qxx.h"
#include "util/recorder.h"
#include "config/cats_config.h"

const static uint32_t REC_BUFFER_LEN = 256;

static inline void write_value(const rec_elem_t *rec_elem, uint8_t *rec_buffer,
                               uint16_t *rec_buffer_idx);

void task_recorder(void *argument) {
  uint8_t *rec_buffer = (uint8_t *)calloc(REC_BUFFER_LEN, sizeof(uint8_t));

  uint16_t rec_buffer_idx = 0;

  log_debug("Recorder Task Started...\n");

#ifdef FLASH_READ
  uint8_t *flash_read_buffer =
      (uint8_t *)calloc(REC_BUFFER_LEN, sizeof(uint8_t));
  uint16_t flash_read_buffer_idx = 0;
#endif

  /* at this point, last recorded should be 100% accurate */
  uint16_t last_recorded_sector = cc_get_last_recorded_sector();
  uint32_t page_id = w25qxx_sector_to_page(last_recorded_sector + 1);

  while (1) {
    rec_elem_t curr_log_elem;

    while (rec_buffer_idx < (REC_BUFFER_LEN - sizeof(rec_elem_t))) {
      if (osMessageQueueGet(rec_queue, &curr_log_elem, NULL, osWaitForever) ==
          osOK) {
        write_value(&curr_log_elem, rec_buffer, &rec_buffer_idx);
      } else {
        log_error("Receiver queue full!");
      }
    }
    /* reset log buffer index */
    rec_buffer_idx = 0;
    log_debug("Rec buffer filled.");

    /* TODO: if the actual buffer size is < 256 the rest is filled with
     * leftover junk; should be optimized later */
    w25qxx_write_page(rec_buffer, page_id, 0, 256);

#ifdef FLASH_READ
    osDelay(500);
    W25qxx_ReadPage(flash_read_buffer, page_id, 0, FLASH_BUFFER_LEN);
    // osDelay(1000);

    if (compare_arrays(flash_buffer, flash_read_buffer, FLASH_BUFFER_LEN)) {
      osDelay(100);
      usb_print("[FLASH] Re-reading the data\n");
      osDelay(100);
      W25qxx_ReadPage(flash_read_buffer, page_id, 0, FLASH_BUFFER_LEN);
    }

    compare_arrays(flash_buffer, flash_read_buffer, FLASH_BUFFER_LEN);

    flash_read_buffer_idx = 0;
    while (flash_read_buffer_idx < (FLASH_BUFFER_LEN - sizeof(log_elem_t))) {
      read_value(flash_read_buffer, &flash_read_buffer_idx, &curr_log_elem,
                 page_id);
    }
#endif

    // HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    log_debug("Page %lu filled.", page_id);

    if (page_id == w25qxx.page_count) {
      /* throw an error */
      log_error("No more space left, all pages filled!");
      break; /* end the task */
    } else {
      uint32_t last_page_of_last_recorded_sector =
          w25qxx_sector_to_page(last_recorded_sector) + 15;
      if (page_id > last_page_of_last_recorded_sector) {
        /* we stepped into a new sector, need to update it */
        cc_set_last_recorded_sector(++last_recorded_sector);
        log_debug("Updating last recorded sector...");
        cc_save();
      } else if (page_id < w25qxx_sector_to_page(last_recorded_sector)) {
        log_fatal("Something went horribly wrong!");
      }
      page_id++;
    }
  }
  free(rec_buffer);
#ifdef FLASH_READ
  free(flash_read_buffer);
#endif
}

static inline void write_value(const rec_elem_t *const rec_elem,
                               uint8_t *rec_buffer, uint16_t *rec_buffer_idx) {
  uint_fast8_t rec_elem_size = sizeof(rec_elem->rec_type);
  switch (rec_elem->rec_type) {
    case IMU0:
    case IMU1:
    case IMU2:
      rec_elem_size += sizeof(rec_elem->u.imu);
      break;
    case BARO0:
    case BARO1:
    case BARO2:
      rec_elem_size += sizeof(rec_elem->u.baro);
      break;

    case FLIGHT_INFO:
      rec_elem_size += sizeof(rec_elem->u.flight_info);
      break;

    case FLIGHT_STATE:
      rec_elem_size += sizeof(rec_elem->u.flight_state);
      break;
    default:
      log_fatal("Impossible recorder entry type!");
      break;
  }
  memcpy(&(rec_buffer[*rec_buffer_idx]), rec_elem, rec_elem_size);
  *rec_buffer_idx += rec_elem_size;
}