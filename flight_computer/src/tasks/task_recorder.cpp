/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstdarg>
#include <cstdio>

#include "cmsis_os.h"
#include "config/globals.hpp"
#include "flash/lfs_custom.hpp"
#include "flash/recorder.hpp"
#include "tasks/task_recorder.hpp"
#include "util/log.h"

/** Private Constants **/

constexpr uint16_t REC_BUFFER_LEN = 256;

/** Private Function Declarations **/

namespace {

constexpr uint_fast8_t get_rec_elem_size(const rec_elem_t *rec_elem);
inline void write_value(const rec_elem_t *rec_elem, uint8_t *rec_buffer, uint16_t *rec_buffer_idx,
                        uint_fast8_t *rec_elem_size);

void create_stats_and_cfg_log();

}  // namespace

/** Exported Function Definitions **/

namespace task {

// NOLINTNEXTLINE(readability-convert-member-functions-to-static,readability-function-cognitive-complexity)
[[noreturn]] void Recorder::Run() noexcept {
  uint8_t rec_buffer[REC_BUFFER_LEN] = {};

  uint16_t rec_buffer_idx = 0;
  uint_fast8_t curr_log_elem_size = 0;

  log_debug("Recorder Task Started...\n");

  uint16_t bytes_remaining = 0;

  lfs_file_t current_flight_file;
  char current_flight_filename[MAX_FILENAME_SIZE] = {};

  while (true) {
    rec_cmd_type_e curr_rec_cmd = REC_CMD_INVALID;
    if (osMessageQueueGet(rec_cmd_queue, &curr_rec_cmd, nullptr, osWaitForever) != osOK) {
      log_error("Something wrong with the command recorder queue");
      continue;
    }
    log_info("recorder command %u received", curr_rec_cmd);
    switch (curr_rec_cmd) {
      case REC_CMD_INVALID:
        log_error("Invalid command value!");
        break;
      case REC_CMD_FILL_Q: {
        rec_elem_t dummy_log_elem{};
        uint32_t cmd_check_counter = 0;
        log_info("Started filling pre recording queue");
        while (true) {
          const uint32_t curr_elem_count = osMessageQueueGetCount(rec_queue);
          if (curr_elem_count > REC_QUEUE_PRE_THRUSTING_LIMIT) {
            /* If the number of elements goes over REC_QUEUE_PRE_THRUSTING_LIMIT we start to empty it. When thrusting is
             * detected we will have around REC_QUEUE_PRE_THRUSTING_LIMIT elements in the queue and in the next loop
             * iteration we will start to write the elements to the flash. */
            osMessageQueueGet(rec_queue, &dummy_log_elem, nullptr, 10);
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
        init_global_flight_stats();

        /* open a new file */
        snprintf(current_flight_filename, MAX_FILENAME_SIZE, "flights/flight_%05lu", flight_counter);
        log_info("Creating log file %lu...", flight_counter);
        lfs_file_open(&lfs, &current_flight_file, current_flight_filename, LFS_O_WRONLY | LFS_O_CREAT);
        lfs_file_write(&lfs, &current_flight_file, code_version, strlen(code_version) + 1);  // including '\0'
        rec_elem_t curr_log_elem{};
        uint32_t sync_counter = 0;
        log_info("Started writing to flash");
        while (true) {
          /* TODO: check if this should be < or <= */
          while (rec_buffer_idx < REC_BUFFER_LEN) {
            // uint32_t curr_elem_count = osMessageQueueGetCount(rec_queue);
            // if (max_elem_count < curr_elem_count) {
            //  max_elem_count = curr_elem_count;
            //  log_warn("max_queued_elems: %lu", max_elem_count);
            //}

            /* Wait 100 ticks to receive a recording element */
            if (osMessageQueueGet(rec_queue, &curr_log_elem, nullptr, 100U) == osOK) {
              write_value(&curr_log_elem, rec_buffer, &rec_buffer_idx, &curr_log_elem_size);
            } else {
              if (global_recorder_status < REC_FILL_QUEUE) {
                log_warn("global_recorder_status < REC_FILL_QUEUE, breaking out of the queue loop.");
              } else {
                log_error("Something wrong with the recording queue!");
              }
              break;
            }
          }

          const int32_t sz =
              lfs_file_write(&lfs, &current_flight_file, rec_buffer, static_cast<lfs_size_t>(REC_BUFFER_LEN));

          /* Writing less than REC_BUFFER_LEN bytes indicates that there is not enough space left on the flash chip. */
          if ((sz > 0) && (static_cast<uint32_t>(sz) < static_cast<lfs_size_t>(REC_BUFFER_LEN))) {
            add_error(CATS_ERR_LOG_FULL);
          }

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
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
            memcpy(rec_buffer, reinterpret_cast<uint8_t *>(&curr_log_elem) + curr_log_elem_size - bytes_remaining,
                   bytes_remaining);
          }
          /* Check for a new command */
          if (osMessageQueueGetCount(rec_cmd_queue) > 0) {
            lfs_file_sync(&lfs, &current_flight_file);
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

        /* TODO: stats file is not always created. Try adding a delay before creating it. */
        // osDelay(200);
        /* create flight stats file */
        create_stats_and_cfg_log();
      } break;
      default:
        log_error("Unknown command value: %u", curr_rec_cmd);
        break;
    }
  }
  /* TODO: check if there is enough space left on the flash */
}

}  // namespace task

/** Private Function Definitions **/

namespace {

constexpr uint_fast8_t get_rec_elem_size(const rec_elem_t *const rec_elem) {
  uint_fast8_t rec_elem_size = sizeof(rec_elem->rec_type) + sizeof(rec_elem->ts);
  switch (get_record_type_without_id(rec_elem->rec_type)) {
    case IMU:
      rec_elem_size += sizeof(rec_elem->u.imu);
      break;
    case BARO:
      rec_elem_size += sizeof(rec_elem->u.baro);
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
    case EVENT_INFO:
      rec_elem_size += sizeof(rec_elem->u.event_info);
      break;
    case ERROR_INFO:
      rec_elem_size += sizeof(rec_elem->u.error_info);
      break;
    case GNSS_INFO:
      rec_elem_size += sizeof(rec_elem->u.gnss_info);
      break;
    case VOLTAGE_INFO:
      rec_elem_size += sizeof(rec_elem->u.voltage_info);
      break;
    default:
      log_raw("Impossible recorder entry type!");
      break;
  }
  return rec_elem_size;
}

inline void write_value(const rec_elem_t *const rec_elem, uint8_t *const rec_buffer, uint16_t *rec_buffer_idx,
                        uint_fast8_t *const rec_elem_size) {
  *rec_elem_size = get_rec_elem_size(rec_elem);
  if (*rec_buffer_idx + *rec_elem_size > REC_BUFFER_LEN) {
    memcpy(&(rec_buffer[*rec_buffer_idx]), rec_elem, (REC_BUFFER_LEN - *rec_buffer_idx));
  } else {
    memcpy(&(rec_buffer[*rec_buffer_idx]), rec_elem, *rec_elem_size);
  }
  *rec_buffer_idx += *rec_elem_size;
}

void create_cfg_file() {
  lfs_file_t current_stats_file;
  char current_stats_filename[MAX_FILENAME_SIZE] = {};

  snprintf(current_stats_filename, MAX_FILENAME_SIZE, "configs/flight_%05lu.cfg", flight_counter);

  log_info("Creating flights_%05lu.cfg...", flight_counter);
  int32_t err = lfs_file_open(&lfs, &current_stats_file, current_stats_filename, LFS_O_WRONLY | LFS_O_CREAT);

  if (err != LFS_ERR_OK) {
    log_error("Creating config file %lu failed with %ld", flight_counter, err);
  } else {
    log_info("Created config file %lu.", flight_counter);
  }
  /* This will work as long as there are no pointers in the config struct */
  err = lfs_file_write(&lfs, &current_stats_file, &global_flight_stats.config, sizeof(global_flight_stats.config));

  if (err < 0) {
    log_error("Writing to config file failed with %ld", err);
  } else if (auto err_u = static_cast<uint32_t>(err); err_u < sizeof(global_flight_stats.config)) {
    log_error("Written less bytes to config file than expected: %lu vs %u", err_u, sizeof(global_flight_stats.config));
  }

  err = lfs_file_close(&lfs, &current_stats_file);

  if (err != LFS_ERR_OK) {
    log_error("Closing config file failed with %ld", err);
  }
}

void create_stats_file() {
  lfs_file_t current_stats_file;
  char current_stats_filename[MAX_FILENAME_SIZE] = {};

  snprintf(current_stats_filename, MAX_FILENAME_SIZE, "stats/stats_%05lu.txt", flight_counter);

  constexpr int kStringBufSz = 300;
  auto *string_buf = static_cast<char *>(pvPortMalloc(kStringBufSz * sizeof(char)));

  log_info("Creating stats file %lu...", flight_counter);
  int32_t err = lfs_file_open(&lfs, &current_stats_file, current_stats_filename, LFS_O_WRONLY | LFS_O_CREAT);

  if (err != LFS_ERR_OK) {
    log_error("Creating stats file %lu failed with %ld", flight_counter, err);
    vPortFree(string_buf);
    return;
  }

  log_info("Created stats file %lu.", flight_counter);

  auto write_line = [&](const char *fmt, ...) __attribute__((format(printf, 2, 3))) {
    va_list va;
    va_start(va, fmt);
    vsnprintf(string_buf, kStringBufSz, fmt, va);
    va_end(va);
    // NOLINTNEXTLINE(hicpp-signed-bitwise)
    err |= lfs_file_write(&lfs, &current_stats_file, string_buf, strlen(string_buf));
  };

  write_line("Flight #%lu Stats\r\n", flight_counter);
  write_line("========================\r\n");
  write_line("  Height\r\n");
  write_line("    Time Since Bootup: %lu\r\n", global_flight_stats.max_height.ts);
  write_line("    Max. Height [m]: %.10f\r\n", static_cast<double>(global_flight_stats.max_height.val));
  write_line("========================\r\n");
  write_line("  Velocity\r\n");
  write_line("    Time Since Bootup: %lu\r\n", global_flight_stats.max_velocity.ts);
  write_line("    Max. Velocity [m/s]: %.10f\r\n", static_cast<double>(global_flight_stats.max_velocity.val));
  write_line("========================\r\n");
  write_line("  Acceleration\r\n");
  write_line("    Time Since Bootup: %lu\r\n", global_flight_stats.max_acceleration.ts);
  write_line("    Max. Acceleration [m/s^2]: %.10f\r\n", static_cast<double>(global_flight_stats.max_acceleration.val));
  write_line("========================\r\n");
  write_line("  Calibration Values\r\n");
  write_line("    Height_0 [m ASL]: %.10f\r\n", static_cast<double>(global_flight_stats.height_0));
  write_line("    IMU:\r\n");
  write_line("      Angle: %.10f\r\n", static_cast<double>(global_flight_stats.calibration_data.angle));
  write_line("      Axis: %hu\r\n", global_flight_stats.calibration_data.axis);
  write_line("    Gyro:\r\n");
  write_line("      x [dps]: %.10f\r\n", static_cast<double>(global_flight_stats.calibration_data.gyro_calib.x));
  write_line("      y [dps]: %.10f\r\n", static_cast<double>(global_flight_stats.calibration_data.gyro_calib.y));
  write_line("      z [dps]: %.10f\r\n", static_cast<double>(global_flight_stats.calibration_data.gyro_calib.z));
  write_line("========================\r\n");
  write_line("  Liftoff Time: %02hu:%02hu:%02hu UTC\r\n", global_flight_stats.liftoff_time.hour,
             global_flight_stats.liftoff_time.min, global_flight_stats.liftoff_time.sec);
  write_line("========================\r\n");

  if (err < 0) {
    log_error("Writing to stats file failed with %ld", err);
  }

  err = lfs_file_close(&lfs, &current_stats_file);

  if (err != LFS_ERR_OK) {
    log_error("Closing stats file failed with %ld", err);
  }

  vPortFree(string_buf);
}

void create_stats_and_cfg_log() {
  create_stats_file();
  create_cfg_file();
}

}  // namespace
