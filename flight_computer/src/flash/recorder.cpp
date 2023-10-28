/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "recorder.hpp"
#include "config/cats_config.hpp"
#include "config/globals.hpp"
#include "util/gnss.hpp"
#include "util/log.h"

#include <cmath>

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
flight_stats_t global_flight_stats = {
    .max_height = {.val = -INFINITY}, .max_velocity = {.val = -INFINITY}, .max_acceleration = {.val = -INFINITY}};

/**
 * Checks whether the given rec_type should be recorded.
 *
 * @param rec_type - recorder entry type
 * @return true if the given rec_type should be recorded
 */
static inline bool should_record(rec_entry_type_e rec_type) { return (global_cats_config.rec_mask & rec_type) > 0; }

/* TODO: See whether this is optimized in assembler. Here we copy the entire struct but the alternative is to pass a
 * pointer and this will cause too many indirect accesses. */
static inline void collect_flight_info_stats(timestamp_t ts, flight_info_t flight_info) {
  if (global_recorder_status == REC_WRITE_TO_FLASH) {
    /* TODO: We need to offset the timestamps in the flight stats struct by subtracting the timestamp at EV_LIFTOFF. */
    if ((flight_info.height > global_flight_stats.max_height.val)) {
      global_flight_stats.max_height.ts = ts;
      global_flight_stats.max_height.val = flight_info.height;
    }
    if (flight_info.velocity > global_flight_stats.max_velocity.val) {
      global_flight_stats.max_velocity.ts = ts;
      global_flight_stats.max_velocity.val = flight_info.velocity;
    }
    if (flight_info.acceleration > global_flight_stats.max_acceleration.val) {
      global_flight_stats.max_acceleration.ts = ts;
      global_flight_stats.max_acceleration.val = flight_info.acceleration;
    }
  }
}

/* Counter used for determining whether individual types should be recorded. */
struct skip_counter_t {
  uint8_t imu;
  uint8_t baro;
  uint8_t flight_info;
  uint8_t orientation;
  uint8_t filtered_data;
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static skip_counter_t skip_counter = {};

/**
 * Determines whether the processed entry should be recorded or not, based on the recorder speed settings.
 *
 * Example:
 * User wants to log the data at half the task frequency. In this example we have 3 IMUs.
 *
 * Task frequency = 100Hz (== 10ms)
 * Recording frequency = 50Hz (== 20ms)
 *   => global_cats_config.rec_speed_idx = 1 (["100Hz", "50Hz", "33.33Hz"...])
 *   => inv_rec_rate = 2 (== we are keeping every 2nd entry that comes in)
 * Number of elements of the same type per task iteration (num_reps_per_iter) = 3
 *   - We want to record all elements of the same type in a single task iteration, that's why we need to know how many
 * occur in one iteration.
 *
 * ts [ms] | entry | cnt | skip |
 * ==============================
 *   1000  | IMU 0 |  0  |  0   |
 *   1000  | IMU 1 |  1  |  0   |
 *   1000  | IMU 2 |  2  |  0   |
 * ------------------------------
 *   1010  | IMU 0 |  3  |  1   |
 *   1010  | IMU 1 |  4  |  1   |
 *   1010  | IMU 2 |  5  |  1   |
 * ------------------------------
 *   1020  | IMU 0 |  0  |  0   |
 *   1020  | IMU 1 |  1  |  0   |
 *   1020  | IMU 2 |  2  |  0   |
 * ------------------------------
 *   1030  | IMU 0 |  3  |  1   |
 *   1030  | IMU 1 |  4  |  1   |
 *   1030  | IMU 2 |  5  |  1   |
 *
 * @param cnt - counter for the given entry type; used to determine from which task iteration the entries should be
 * recorded
 * @param num_reps_per_iter - number of repetitions of the same entry type in one task iteration
 * @return true if the entry should not be recorded, false otherwise
 */
inline static bool should_skip(uint8_t *cnt, uint8_t num_reps_per_iter) {
  /* Return right away if everything should be recorded */
  if (global_cats_config.rec_speed_idx == 0) {
    return false;
  }
  const uint8_t inv_rec_rate = global_cats_config.rec_speed_idx + 1;
  /* Skip the entry if we already recorded all entries from the iteration that should be recorded */
  const bool skip = (*cnt % (inv_rec_rate * num_reps_per_iter)) >= num_reps_per_iter;
  /* Increment the counter and reset it to 0 if the max value is reached */
  *cnt = (*cnt + 1) % (inv_rec_rate * num_reps_per_iter);
  return skip;
}

void record(timestamp_t ts, rec_entry_type_e rec_type_with_id, const void *const rec_value) {
  const rec_entry_type_e pure_rec_type = get_record_type_without_id(rec_type_with_id);

  if (global_recorder_status >= REC_FILL_QUEUE && should_record(pure_rec_type)) {
    rec_elem_t e = {.ts = ts, .rec_type = rec_type_with_id};
    switch (pure_rec_type) {
      case IMU:
        if (should_skip(&skip_counter.imu, NUM_IMU)) {
          return;
        }
        e.u.imu = *(static_cast<const imu_data_t *>(rec_value));
        break;
      case BARO:
        if (should_skip(&skip_counter.baro, NUM_BARO)) {
          return;
        }
        e.u.baro = *(static_cast<const baro_data_t *>(rec_value));
        break;
      case FLIGHT_INFO:
        e.u.flight_info = *(static_cast<const flight_info_t *>(rec_value));
        /* Record the flight info stats before deciding whether to record this entry or not. */
        collect_flight_info_stats(ts, e.u.flight_info);
        if (should_skip(&skip_counter.flight_info, 1)) {
          return;
        }
        break;
      case ORIENTATION_INFO:
        if (should_skip(&skip_counter.orientation, 1)) {
          return;
        }
        e.u.orientation_info = *(static_cast<const orientation_info_t *>(rec_value));
        break;
      case FILTERED_DATA_INFO:
        if (should_skip(&skip_counter.filtered_data, 1)) {
          return;
        }
        e.u.filtered_data_info = *(static_cast<const filtered_data_info_t *>(rec_value));
        break;
      case FLIGHT_STATE:
        e.u.flight_state = *(static_cast<const flight_fsm_e *>(rec_value));
        break;
      case EVENT_INFO:
        e.u.event_info = *(static_cast<const event_info_t *>(rec_value));
        break;
      case ERROR_INFO:
        e.u.error_info = *(static_cast<const error_info_t *>(rec_value));
        break;
      case GNSS_INFO:
        e.u.gnss_info = *(static_cast<const gnss_position_t *>(rec_value));
        break;
      case VOLTAGE_INFO:
        e.u.voltage_info = *(static_cast<const voltage_info_t *>(rec_value));
        break;
      default:
        log_fatal("Impossible recorder entry type %lu!", pure_rec_type);
        break;
    }

    const osStatus_t ret = osMessageQueuePut(rec_queue, &e, 0U, 0U);
    if (ret != osOK) {
      log_error("Inserting an element to the recorder queue failed! Error: %d", ret);
    }
  }
}
