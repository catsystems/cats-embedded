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

static inline uint_fast8_t get_rec_elem_size(const rec_elem_t *rec_elem);
static inline void write_value(const rec_elem_t *rec_elem, uint8_t *rec_buffer,
                               uint16_t *rec_buffer_idx,
                               uint_fast8_t *rec_elem_size);

//#define PAGE_BREAK_TEST
#ifdef PAGE_BREAK_TEST
static inline void write_value_test(const rec_elem_t *rec_elem,
                                    uint8_t *rec_buffer,
                                    uint16_t *rec_buffer_idx,
                                    uint_fast8_t *rec_elem_size);
#endif

void task_recorder(void *argument) {
  uint8_t *rec_buffer = (uint8_t *)calloc(REC_BUFFER_LEN, sizeof(uint8_t));

#ifdef PAGE_BREAK_TEST
  uint16_t test_mem_idx = 0;
  uint8_t *test_mem_buffer = (uint8_t *)calloc(4096, sizeof(uint8_t));
  uint8_t *test_flash_buffer = (uint8_t *)calloc(4096, sizeof(uint8_t));
  uint_fast8_t curr_test_log_elem_size = 0;
#endif

  uint16_t rec_buffer_idx = 0;
  uint_fast8_t curr_log_elem_size = 0;

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

    /* TODO: check if this should be < or <= */
    while (rec_buffer_idx < REC_BUFFER_LEN) {
      if (osMessageQueueGet(rec_queue, &curr_log_elem, NULL, osWaitForever) ==
          osOK) {
        write_value(&curr_log_elem, rec_buffer, &rec_buffer_idx,
                    &curr_log_elem_size);
#ifdef PAGE_BREAK_TEST
        /* TODO: You need to set the offset here when reading every subsequent
         * sector because the struct was also broken on the sector boundary */
        if (test_mem_idx < 4096 - sizeof(rec_elem_t)) {
          write_value_test(&curr_log_elem, test_mem_buffer, &test_mem_idx,
                           &curr_test_log_elem_size);
        }
#endif
      } else {
        log_error("Receiver queue full!");
      }
    }

    // log_debug("Rec buffer filled.");

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

    /* reset log buffer index */
    if (rec_buffer_idx > REC_BUFFER_LEN) {
      uint16_t bytes_remaining = rec_buffer_idx - REC_BUFFER_LEN;
      //      log_warn(
      //          "rec_buffer_idx = %u, bytes_remaining = %u, curr_log_elem_size
      //          = "
      //          "%u",
      //          rec_buffer_idx, bytes_remaining, curr_log_elem_size);
      memcpy(rec_buffer,
             (uint8_t *)(&curr_log_elem) + curr_log_elem_size - bytes_remaining,
             bytes_remaining);
      rec_buffer_idx = bytes_remaining;
    } else {
      rec_buffer_idx = 0;
    }

    // HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    // log_debug("Page %lu filled.", page_id);

    if (page_id == w25qxx.page_count) {
      /* throw an error */
      log_error("No more space left, all pages filled!");
      /* TODO: this task should actually be killed somehow */
      break; /* end the task */
    } else {
      uint32_t last_page_of_last_recorded_sector =
          w25qxx_sector_to_page(last_recorded_sector) + 15;
#ifdef PAGE_BREAK_TEST
      if (page_id == last_page_of_last_recorded_sector) {
        /* test-read the current sector */
        w25qxx_read_sector(test_flash_buffer, last_recorded_sector, 0,
                           test_mem_idx);
        int i;
        for (i = 0; i < test_mem_idx; i++) {
          if (test_mem_buffer[i] != test_flash_buffer[i]) {
            log_error("arrays not same %d!", i);
            break;
          }
        }
        if (i == test_mem_idx) {
          log_info("ARRAYS SAME!!!");
        }
        test_mem_idx = 0;
      }
#endif
      if (page_id > last_page_of_last_recorded_sector) {
        /* we stepped into a new sector, need to update it */
        cc_set_last_recorded_sector(++last_recorded_sector);
        log_debug("Updating last recorded sector to %d", last_recorded_sector);
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

static uint_fast8_t get_rec_elem_size(const rec_elem_t *const rec_elem) {
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
  return rec_elem_size;
}

static inline void write_value(const rec_elem_t *const rec_elem,
                               uint8_t *const rec_buffer,
                               uint16_t *rec_buffer_idx,
                               uint_fast8_t *const rec_elem_size) {
  *rec_elem_size = get_rec_elem_size(rec_elem);
  if (*rec_buffer_idx + *rec_elem_size > REC_BUFFER_LEN) {
    memcpy(&(rec_buffer[*rec_buffer_idx]), rec_elem,
           (REC_BUFFER_LEN - *rec_buffer_idx));
  } else {
    memcpy(&(rec_buffer[*rec_buffer_idx]), rec_elem, *rec_elem_size);
  }
  *rec_buffer_idx += *rec_elem_size;
}

static inline void write_value_test(const rec_elem_t *const rec_elem,
                                    uint8_t *const rec_buffer,
                                    uint16_t *rec_buffer_idx,
                                    uint_fast8_t *const rec_elem_size) {
  *rec_elem_size = get_rec_elem_size(rec_elem);
  memcpy(&(rec_buffer[*rec_buffer_idx]), rec_elem, *rec_elem_size);
  *rec_buffer_idx += *rec_elem_size;
}