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

#include "cmsis_os.h"
#include "tasks/task_recorder.h"
#include "util/log.h"
#include "util/types.h"
#include "lfs/lfs_custom.h"
#include "util/recorder.h"
#include "config/cats_config.h"

#include <stdlib.h>

/** Private Constants **/

#define REC_BUFFER_LEN 256

/** Private Function Declarations **/

static inline uint_fast8_t get_rec_elem_size(const rec_elem_t *rec_elem);
static inline void write_value(const rec_elem_t *rec_elem, uint8_t *rec_buffer, uint16_t *rec_buffer_idx,
                               uint_fast8_t *rec_elem_size);

static void create_stats_file();

/** Exported Function Definitions **/

_Noreturn void task_recorder(__attribute__((unused)) void *argument) {
  uint8_t rec_buffer[REC_BUFFER_LEN] = {};

  uint16_t rec_buffer_idx = 0;
  uint_fast8_t curr_log_elem_size = 0;

  log_debug("Recorder Task Started...\n");

  uint16_t bytes_remaining = 0;
  uint32_t max_elem_count = 0;

  lfs_file_t current_flight_file;
  char current_flight_filename[MAX_FILENAME_SIZE] = {};

  while (1) {
    rec_cmd_type_e curr_rec_cmd = REC_CMD_INVALID;

    if (osMessageQueueGet(rec_cmd_queue, &curr_rec_cmd, NULL, osWaitForever) != osOK) {
      log_error("Something wrong with the command recorder queue");
      continue;
    }
    // trace_printf(flash_channel, "received command %d", curr_rec_cmd);
    switch (curr_rec_cmd) {
      case REC_CMD_INVALID:
        log_error("Invalid command value!");
        break;
      case REC_CMD_FILL_Q: {
        rec_elem_t dummy_log_elem;
        uint32_t cmd_check_counter = 0;
        log_info("Started filling pre recording queue");
        while (1) {
          uint32_t curr_elem_count = osMessageQueueGetCount(rec_queue);
          if (curr_elem_count > REC_QUEUE_PRE_THRUSTING_LIMIT) {
            /* If the number of elements goes over REC_QUEUE_PRE_THRUSTING_LIMIT we start to empty it. When thrusting is
             * detected we will have around REC_QUEUE_PRE_THRUSTING_LIMIT elements in the queue and in the next loop
             * iteration we will start to write the elements to the flash. */
            osMessageQueueGet(rec_queue, &dummy_log_elem, 0, 10);
            // osMessageQueueGet(rec_queue, &dummy_log_elem, 0, 10);
            // osMessageQueueGet(rec_queue, &dummy_log_elem, 0, 10);
          } else {
            /* Check for a new command */
            if (osMessageQueueGetCount(rec_cmd_queue) > 0) {
              /* breaks out of the inner while loop */
              break;
            }
            osDelay(1);
          }

          ++cmd_check_counter;
          /* Check for a new command */
          if (((cmd_check_counter % 16) == 0) && (osMessageQueueGetCount(rec_cmd_queue) > 0)) {
            /* breaks out of the inner while loop */
            break;
          }
        }
      } break;
      case REC_CMD_FILL_Q_STOP:
        osMessageQueueReset(rec_queue);
        break;
      case REC_CMD_WRITE: {
        /* increment number of flights */
        ++flight_counter;
        lfs_file_open(&lfs, &fc_file, "flight_counter", LFS_O_RDWR | LFS_O_CREAT);
        lfs_file_rewind(&lfs, &fc_file);
        lfs_file_write(&lfs, &fc_file, &flight_counter, sizeof(flight_counter));
        lfs_file_close(&lfs, &fc_file);

        /* reset flight stats */
        reset_global_flight_stats();

        /* open a new file */
        snprintf(current_flight_filename, MAX_FILENAME_SIZE, "flights/flight_%05lu", flight_counter);
        lfs_file_open(&lfs, &current_flight_file, current_flight_filename, LFS_O_WRONLY | LFS_O_CREAT);
        rec_elem_t curr_log_elem;
        uint32_t sync_counter = 0;
        log_info("Started writing to flash");
        while (1) {
          /* TODO: check if this should be < or <= */
          while (rec_buffer_idx < REC_BUFFER_LEN) {
            // uint32_t curr_elem_count = osMessageQueueGetCount(rec_queue);
            // if (max_elem_count < curr_elem_count) {
            //  max_elem_count = curr_elem_count;
            //  log_warn("max_queued_elems: %lu", max_elem_count);
            //}

            if (osMessageQueueGet(rec_queue, &curr_log_elem, NULL, osWaitForever) == osOK) {
              // trace_print(flash_channel, "write_value start");
              write_value(&curr_log_elem, rec_buffer, &rec_buffer_idx, &curr_log_elem_size);
              // trace_print(flash_channel, "write_value end");
            } else {
              log_error("Something wrong with the recording queue!");
            }
          }
          // log_info("lfw start");
          // trace_print(flash_channel, "lfw start");
          int32_t sz = lfs_file_write(&lfs, &current_flight_file, rec_buffer, (lfs_size_t)REC_BUFFER_LEN);
          // trace_printf(flash_channel, "lfw end, written %ld", sz);
          ++sync_counter;
          /* Check for a new command */
          if ((sync_counter % 32) == 0) {
            lfs_file_sync(&lfs, &current_flight_file);
          }
          // log_info("lfw synced");
          // log_info("written to file: %ld", sz);

          /* reset log buffer index */
          if (rec_buffer_idx > REC_BUFFER_LEN) {
            bytes_remaining = rec_buffer_idx - REC_BUFFER_LEN;
            rec_buffer_idx = bytes_remaining;
          } else {
            rec_buffer_idx = 0;
          }

          if (rec_buffer_idx > 0) {
            memcpy(rec_buffer, (uint8_t *)(&curr_log_elem) + curr_log_elem_size - bytes_remaining, bytes_remaining);
          }
          /* Check for a new command */
          if (osMessageQueueGetCount(rec_cmd_queue) > 0) {
            /* breaks out of the inner while loop */
            break;
          }
        }
      } break;
      case REC_CMD_WRITE_STOP: {
        log_info("Stopped writing to flash");
        /* close the current file */
        lfs_file_close(&lfs, &current_flight_file);

        /* reset recording buffer index and queue */
        rec_buffer_idx = 0;
        osMessageQueueReset(rec_queue);

        /* create flight stats file */
        create_stats_file();
      } break;
      default:
        log_error("Unknown command value: %u", curr_rec_cmd);
        break;
    }
  }
  /* TODO: check if there is enough space left on the flash */
}

/** Private Function Definitions **/

static uint_fast8_t get_rec_elem_size(const rec_elem_t *const rec_elem) {
  uint_fast8_t rec_elem_size = sizeof(rec_elem->rec_type);
  switch (get_record_type_without_id(rec_elem->rec_type)) {
    case IMU:
      rec_elem_size += sizeof(rec_elem->u.imu);
      break;
    case BARO:
      rec_elem_size += sizeof(rec_elem->u.baro);
      break;
    case MAGNETO:
      rec_elem_size += sizeof(rec_elem->u.magneto_info);
      break;
    case ACCELEROMETER:
      rec_elem_size += sizeof(rec_elem->u.accel_data);
      break;
    case FLIGHT_INFO:
      rec_elem_size += sizeof(rec_elem->u.flight_info);
      break;
    case ORIENTATION_INFO:
      rec_elem_size += sizeof(rec_elem->u.orientation_info);
      break;
    case FILTERED_DATA_INFO:
      rec_elem_size += sizeof(rec_elem->u.filtered_data_info);
      break;
    case FLIGHT_STATE:
      rec_elem_size += sizeof(rec_elem->u.flight_state);
      break;
    case SENSOR_INFO:
      rec_elem_size += sizeof(rec_elem->u.sensor_info);
      break;
    case EVENT_INFO:
      rec_elem_size += sizeof(rec_elem->u.event_info);
      break;
    case ERROR_INFO:
      rec_elem_size += sizeof(rec_elem->u.error_info);
      break;
    default:
      log_fatal("Impossible recorder entry type!");
      break;
  }
  return rec_elem_size;
}

static inline void write_value(const rec_elem_t *const rec_elem, uint8_t *const rec_buffer, uint16_t *rec_buffer_idx,
                               uint_fast8_t *const rec_elem_size) {
  *rec_elem_size = get_rec_elem_size(rec_elem);
  if (*rec_buffer_idx + *rec_elem_size > REC_BUFFER_LEN) {
    memcpy(&(rec_buffer[*rec_buffer_idx]), rec_elem, (REC_BUFFER_LEN - *rec_buffer_idx));
  } else {
    memcpy(&(rec_buffer[*rec_buffer_idx]), rec_elem, *rec_elem_size);
  }
  *rec_buffer_idx += *rec_elem_size;
}

static void create_stats_file() {
  lfs_file_t current_stats_file;
  char current_stats_filename[MAX_FILENAME_SIZE] = {};

  snprintf(current_stats_filename, MAX_FILENAME_SIZE, "stats/stats_%05lu", flight_counter);
  lfs_file_open(&lfs, &current_stats_file, current_stats_filename, LFS_O_WRONLY | LFS_O_CREAT);

  /* This will as long as there are no pointers in the global_flight_stats struct */
  lfs_file_write(&lfs, &current_stats_file, &global_flight_stats, sizeof(global_flight_stats));

  lfs_file_close(&lfs, &current_stats_file);
}
