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

#include <stddef.h>
#include <stdint.h>

#include "util/reader.h"
#include "util/recorder.h"
#include "util/log.h"
#include "config/globals.h"
#include "lfs/lfs_custom.h"
#include "control/data_processing.h"

void dump_recording(uint16_t number) {
  if (global_recorder_status == REC_WRITE_TO_FLASH) {
    log_raw("The recorder is currently active, stop it first!");
    return;
  }

  char *string_buffer1 = calloc(400, sizeof(char));
  char *string_buffer2 = calloc(400, sizeof(char));
  uint8_t *read_buf = (uint8_t *)calloc(256, sizeof(uint8_t));

  char filename[MAX_FILENAME_SIZE] = {};
  snprintf(filename, MAX_FILENAME_SIZE, "flights/flight_%05d", number);

  log_raw("Dumping file: %s", filename);

  lfs_file_t curr_file;
  if (lfs_file_open(&lfs, &curr_file, filename, LFS_O_RDONLY) == LFS_ERR_OK) {
    int file_size = lfs_file_size(&lfs, &curr_file);
    for (lfs_size_t i = 0; i < file_size; i += 256) {
      lfs_size_t chunk = lfs_min(256, file_size - i);

      lfs_file_read(&lfs, &curr_file, read_buf, chunk);

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

      memset(string_buffer1, 0, 400);
      memset(string_buffer2, 0, 400);
    }
  } else {
    log_error("Flight %d not found!", number);
  }

  lfs_file_close(&lfs, &curr_file);

  free(string_buffer1);
  free(string_buffer2);
}

void parse_recording(uint16_t number) {
  if (global_recorder_status == REC_WRITE_TO_FLASH) {
    log_raw("The recorder is currently active, stop it first!");
    return;
  }

  char filename[MAX_FILENAME_SIZE] = {};
  snprintf(filename, MAX_FILENAME_SIZE, "flights/flight_%05d", number);

  log_raw("Reading file: %s", filename);

  lfs_file_t curr_file;
  if (lfs_file_open(&lfs, &curr_file, filename, LFS_O_RDONLY) == LFS_ERR_OK) {
    rec_elem_t rec_elem;
    lfs_ssize_t file_size = lfs_file_size(&lfs, &curr_file);
    if (file_size < 0) {
      log_raw("Invalid file size %ld!", file_size);
      return;
    }
    rec_entry_type_e rec_type;
    while (lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_type, 4) > 0) {
      switch (get_record_type_without_id(rec_type)) {
        case IMU: {
          size_t elem_sz = sizeof(rec_elem.u.imu);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          log_raw("%lu|IMU%hu|%d|%d|%d|%d|%d|%d", rec_elem.u.imu.ts, get_id_from_record_type(rec_type),
                  rec_elem.u.imu.acc_x, rec_elem.u.imu.acc_y, rec_elem.u.imu.acc_z, rec_elem.u.imu.gyro_x,
                  rec_elem.u.imu.gyro_y, rec_elem.u.imu.gyro_z);
        } break;
        case BARO: {
          size_t elem_sz = sizeof(rec_elem.u.baro);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          log_raw("%lu|BARO%hu|%lu|%lu", rec_elem.u.baro.ts, get_id_from_record_type(rec_type),
                  rec_elem.u.baro.pressure, rec_elem.u.baro.temperature);
        } break;
        case MAGNETO: {
          size_t elem_sz = sizeof(rec_elem.u.magneto_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          log_raw("%lu|MAGNETO|%f|%f|%f", rec_elem.u.magneto_info.ts, (double)rec_elem.u.magneto_info.magneto_x,
                  (double)rec_elem.u.magneto_info.magneto_y, (double)rec_elem.u.magneto_info.magneto_z);
        } break;
        case ACCELEROMETER: {
          size_t elem_sz = sizeof(rec_elem.u.accel_data);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          log_raw("%lu|ACC|%d|%d|%d", rec_elem.u.accel_data.ts, rec_elem.u.accel_data.acc_x,
                  rec_elem.u.accel_data.acc_y, rec_elem.u.accel_data.acc_z);
        } break;
        case FLIGHT_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.flight_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          log_raw("%lu|FLIGHT_INFO|%f|%f|%f", rec_elem.u.flight_info.ts, (double)rec_elem.u.flight_info.acceleration,
                  (double)rec_elem.u.flight_info.height, (double)rec_elem.u.flight_info.velocity);
        } break;
        case ORIENTATION_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.orientation_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          log_raw("%lu|ORIENTATION_INFO|%d|%d|%d|%d", rec_elem.u.orientation_info.ts,
                  rec_elem.u.orientation_info.estimated_orientation[0],
                  rec_elem.u.orientation_info.estimated_orientation[1],
                  rec_elem.u.orientation_info.estimated_orientation[2],
                  rec_elem.u.orientation_info.estimated_orientation[3]);
        } break;
        case FILTERED_DATA_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.filtered_data_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          log_raw("%lu|FILTERED_DATA_INFO|%f|%f", rec_elem.u.filtered_data_info.ts,
                  (double)rec_elem.u.filtered_data_info.filtered_altitude_AGL,
                  (double)rec_elem.u.filtered_data_info.filtered_acceleration);
        } break;
        case FLIGHT_STATE: {
          size_t elem_sz = sizeof(rec_elem.u.flight_state);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          log_raw("%lu|FLIGHT_STATE|%u", rec_elem.u.flight_state.ts,
                  rec_elem.u.flight_state.flight_or_drop_state.flight_state);
        } break;
        case SENSOR_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.sensor_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          log_raw("%lu|SENSOR_INFO|%u|%u|%u|%u|%u|%u", rec_elem.u.sensor_info.ts, rec_elem.u.sensor_info.faulty_imu[0],
                  rec_elem.u.sensor_info.faulty_imu[1], rec_elem.u.sensor_info.faulty_imu[2],
                  rec_elem.u.sensor_info.faulty_baro[0], rec_elem.u.sensor_info.faulty_baro[1],
                  rec_elem.u.sensor_info.faulty_baro[2]);
        } break;
        case EVENT_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.event_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          log_raw("%lu|EVENT_INFO|%d|%u", rec_elem.u.event_info.ts, rec_elem.u.event_info.event,
                  rec_elem.u.event_info.action_idx);
        } break;
        case ERROR_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.error_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          log_raw("%lu|ERROR_INFO|%d", rec_elem.u.error_info.ts, rec_elem.u.error_info.error);
        } break;
        default:
          log_raw("Impossible recorder entry type!");
          break;
      }
    }
    lfs_file_close(&lfs, &curr_file);
  } else {
    log_raw("Flight %d not found!", number);
  }

  lfs_file_close(&lfs, &curr_file);
}

void parse_stats(uint16_t number) {
  if (global_recorder_status == REC_WRITE_TO_FLASH) {
    log_raw("The recorder is currently active, stop it first!");
    return;
  }

  char filename[MAX_FILENAME_SIZE] = {};
  snprintf(filename, MAX_FILENAME_SIZE, "stats/stats_%05d", number);

  log_raw("Reading file: %s", filename);

  lfs_file_t curr_file;

  if (lfs_file_open(&lfs, &curr_file, filename, LFS_O_RDONLY) == LFS_ERR_OK) {
    flight_stats_t local_flight_stats = {};
    if (lfs_file_read(&lfs, &curr_file, &local_flight_stats, sizeof(flight_stats_t)) > 0) {
      log_raw("Flight Stats %d", number);
      log_raw("========================");
      log_raw("  Height");
      log_raw("    Time Since Bootup: %lu", local_flight_stats.max_height.ts);
      log_raw("    Max. Height [m]: %f", (double)local_flight_stats.max_height.val);
      log_raw("========================");
      log_raw("  Velocity");
      log_raw("    Time Since Bootup: %lu", local_flight_stats.max_velocity.ts);
      log_raw("    Max. Velocity [m/s]: %f", (double)local_flight_stats.max_velocity.val);
      log_raw("========================");
      log_raw("  Acceleration");
      log_raw("    Time Since Bootup: %lu", local_flight_stats.max_acceleration.ts);
      log_raw("    Max. Acceleration [m/s^2]: %f", (double)local_flight_stats.max_acceleration.val);
    } else {
      log_raw("Error while reading Stats %d", number);
    }
  } else {
    log_raw("Stats %d not found!", number);
  }

  lfs_file_close(&lfs, &curr_file);
}

void erase_recordings() { /* remove everything from /flights */
}
