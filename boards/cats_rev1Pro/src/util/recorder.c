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

#include "util/recorder.h"
#include "util/log.h"
#include "config/globals.h"
#include "config/cats_config.h"

#include <math.h>

flight_stats_t global_flight_stats = {
    .max_height.val = -INFINITY, .max_velocity.val = -INFINITY, .max_acceleration.val = -INFINITY};

/**
 * Checks whether the given rec_type should be recorded.
 *
 * @param rec_type - recorder entry type
 * @return true if the given rec_type should be recorded
 */
static inline bool should_record(rec_entry_type_e rec_type) {
  return (global_cats_config.config.recorder_mask & rec_type) > 0;
}

/* TODO: See whether this is optimized in assembler. Here we copy the entire struct but the alternative is to pass a
 * pointer and this will cause too many indirect accesses. */
static inline void collect_flight_info_stats(flight_info_t flight_info) {
  if (global_recorder_status == REC_WRITE_TO_FLASH) {
    /* TODO: We need to offset the timestamps in the flight stats struct by subtracting the timestamp at EV_LIFTOFF. */
    if ((flight_info.height > global_flight_stats.max_height.val)) {
      global_flight_stats.max_height.ts = flight_info.ts;
      global_flight_stats.max_height.val = flight_info.height;
    }
    if (flight_info.velocity > global_flight_stats.max_velocity.val) {
      global_flight_stats.max_velocity.ts = flight_info.ts;
      global_flight_stats.max_velocity.val = flight_info.velocity;
    }
    if (flight_info.acceleration > global_flight_stats.max_acceleration.val) {
      global_flight_stats.max_acceleration.ts = flight_info.ts;
      global_flight_stats.max_acceleration.val = flight_info.acceleration;
    }
  }
}

void record(rec_entry_type_e rec_type_with_id, const void *rec_value) {
  rec_entry_type_e pure_rec_type = get_record_type_without_id(rec_type_with_id);
  if (global_recorder_status >= REC_FILL_QUEUE && should_record(pure_rec_type)) {
    rec_elem_t e = {.rec_type = rec_type_with_id};
    switch (pure_rec_type) {
      case IMU:
        e.u.imu = *((imu_data_t *)rec_value);
        break;
      case BARO:
        e.u.baro = *((baro_data_t *)rec_value);
        break;
      case MAGNETO:
        e.u.magneto_info = *((magneto_data_t *)rec_value);
        break;
      case ACCELEROMETER:
        e.u.accel_data = *((accel_data_t *)rec_value);
        break;
      case FLIGHT_INFO:
        e.u.flight_info = *((flight_info_t *)rec_value);
        collect_flight_info_stats(e.u.flight_info);
        break;
      case ORIENTATION_INFO:
        e.u.orientation_info = *((orientation_info_t *)rec_value);
        break;
      case FILTERED_DATA_INFO:
        e.u.filtered_data_info = *((filtered_data_info_t *)rec_value);
        break;
      case FLIGHT_STATE:
        e.u.flight_state = *((flight_state_t *)rec_value);
        break;
      case SENSOR_INFO:
        e.u.sensor_info = *((sensor_info_t *)rec_value);
        break;
      case EVENT_INFO:
        e.u.event_info = *((event_info_t *)rec_value);
        break;
      case ERROR_INFO:
        e.u.error_info = *((error_info_t *)rec_value);
        break;
      default:
        log_fatal("Impossible recorder entry type %d!", pure_rec_type);
        break;
    }

    osStatus_t ret = osMessageQueuePut(rec_queue, &e, 0U, 0U);
    if (ret != osOK) {
      log_error("Inserting an element to the recorder queue failed! Error: %d", ret);
    }
  }
}
