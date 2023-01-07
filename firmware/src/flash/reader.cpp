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

#include "reader.h"

#include <cstddef>
#include <cstdio>

#include "cli/settings.h"
#include "config/globals.h"
#include "flash/lfs_custom.h"
#include "recorder.h"
#include "util/enum_str_maps.h"
#include "util/log.h"

#define STRING_BUF_SZ 400
#define READ_BUF_SZ   256

void dump_recording(uint16_t number) {
  if (global_recorder_status == REC_WRITE_TO_FLASH) {
    log_raw("The recorder is currently active, stop it first!");
    return;
  }

  char *string_buffer1 = (char *)(pvPortMalloc(STRING_BUF_SZ * sizeof(char)));
  char *string_buffer2 = (char *)(pvPortMalloc(STRING_BUF_SZ * sizeof(char)));
  uint8_t *read_buf = (uint8_t *)(pvPortMalloc(READ_BUF_SZ * sizeof(uint8_t)));

  if (string_buffer1 == nullptr || string_buffer2 == nullptr || read_buf == nullptr) {
    log_raw("Cannot allocate enough memory for flight dumping!");
    return;
  }

  memset(string_buffer1, 0, STRING_BUF_SZ);
  memset(string_buffer2, 0, STRING_BUF_SZ);
  memset(read_buf, 0, READ_BUF_SZ);

  char filename[MAX_FILENAME_SIZE] = {};
  snprintf(filename, MAX_FILENAME_SIZE, "flights/flight_%05d", number);

  log_raw("Dumping file: %s", filename);

  lfs_file_t curr_file;
  if (lfs_file_open(&lfs, &curr_file, filename, LFS_O_RDONLY) == LFS_ERR_OK) {
    int file_size = lfs_file_size(&lfs, &curr_file);
    for (lfs_size_t i = 0; i < file_size; i += READ_BUF_SZ) {
      lfs_size_t chunk = lfs_min(READ_BUF_SZ, file_size - i);

      lfs_file_read(&lfs, &curr_file, read_buf, chunk);

      int write_idx = 0;
      for (uint32_t j = 0; j < READ_BUF_SZ / 2; ++j) {
        write_idx += sprintf(string_buffer1 + write_idx, "%02x ", read_buf[j]);
        //      for (uint32_t j = 0; j < 128; j += 16) {
        //        write_idx +=
        //            sprintf(string_buffer1 + write_idx,
        //                    "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x ",
        //                    read_buf[j], read_buf[j + 1], read_buf[j + 2], read_buf[j + 3], read_buf[j + 4],
        //                    read_buf[j + 5], read_buf[j + 6], read_buf[j + 7], read_buf[j + 8], read_buf[j + 9],
        //                    read_buf[j + 10], read_buf[j + 11], read_buf[j + 12], read_buf[j + 13], read_buf[j + 14],
        //                    read_buf[j + 15]);
      }
      log_rawr("%s", string_buffer1);
      write_idx = 0;
      for (uint32_t j = READ_BUF_SZ / 2; j < READ_BUF_SZ; ++j) {
        write_idx += sprintf(string_buffer2 + write_idx, "%02x ", read_buf[j]);
        //      for (uint32_t j = 128; j < 256; j += 16) {
        //        write_idx +=
        //            sprintf(string_buffer1 + write_idx,
        //                    "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x ",
        //                    read_buf[j], read_buf[j + 1], read_buf[j + 2], read_buf[j + 3], read_buf[j + 4],
        //                    read_buf[j + 5], read_buf[j + 6], read_buf[j + 7], read_buf[j + 8], read_buf[j + 9],
        //                    read_buf[j + 10], read_buf[j + 11], read_buf[j + 12], read_buf[j + 13], read_buf[j + 14],
        //                    read_buf[j + 15]);
      }
      log_rawr("%s\n", string_buffer2);

      memset(string_buffer1, 0, STRING_BUF_SZ);
      memset(string_buffer2, 0, STRING_BUF_SZ);
    }
  } else {
    log_error("Flight %d not found!", number);
  }

  lfs_file_close(&lfs, &curr_file);

  vPortFree(string_buffer1);
  vPortFree(string_buffer2);
  vPortFree(read_buf);
}

void parse_recording(uint16_t number, rec_entry_type_e filter_mask) {
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

    while (lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem, 8) > 0) {
      const rec_entry_type_e rec_type = rec_elem.rec_type;
      const rec_entry_type_e rec_type_without_id = get_record_type_without_id(rec_type);
      switch (rec_type_without_id) {
        case IMU: {
          size_t elem_sz = sizeof(rec_elem.u.imu);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          if ((rec_type_without_id & filter_mask) > 0) {
            log_raw("%lu|IMU%hu|%d|%d|%d|%d|%d|%d", rec_elem.ts, get_id_from_record_type(rec_type),
                    rec_elem.u.imu.acc.x, rec_elem.u.imu.acc.y, rec_elem.u.imu.acc.z, rec_elem.u.imu.gyro.x,
                    rec_elem.u.imu.gyro.y, rec_elem.u.imu.gyro.z);
          }
        } break;
        case BARO: {
          size_t elem_sz = sizeof(rec_elem.u.baro);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          if ((rec_type_without_id & filter_mask) > 0) {
            log_raw("%lu|BARO%hu|%lu|%lu", rec_elem.ts, get_id_from_record_type(rec_type), rec_elem.u.baro.pressure,
                    rec_elem.u.baro.temperature);
          }
        } break;
        case MAGNETO: {
          size_t elem_sz = sizeof(rec_elem.u.magneto_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          if ((rec_type_without_id & filter_mask) > 0) {
            log_raw("%lu|MAGNETO|%f|%f|%f", rec_elem.ts, (double)rec_elem.u.magneto_info.x,
                    (double)rec_elem.u.magneto_info.y, (double)rec_elem.u.magneto_info.z);
          }
        } break;
        case ACCELEROMETER: {
          size_t elem_sz = sizeof(rec_elem.u.accel_data);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          if ((rec_type_without_id & filter_mask) > 0) {
            log_raw("%lu|ACC|%d|%d|%d", rec_elem.ts, rec_elem.u.accel_data.x, rec_elem.u.accel_data.y,
                    rec_elem.u.accel_data.z);
          }
        } break;
        case FLIGHT_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.flight_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          if ((rec_type_without_id & filter_mask) > 0) {
            log_raw("%lu|FLIGHT_INFO|%f|%f|%f", rec_elem.ts, (double)rec_elem.u.flight_info.acceleration,
                    (double)rec_elem.u.flight_info.height, (double)rec_elem.u.flight_info.velocity);
          }
        } break;
        case ORIENTATION_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.orientation_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          if ((rec_type_without_id & filter_mask) > 0) {
            log_raw("%lu|ORIENTATION_INFO|%d|%d|%d|%d", rec_elem.ts,
                    rec_elem.u.orientation_info.estimated_orientation[0],
                    rec_elem.u.orientation_info.estimated_orientation[1],
                    rec_elem.u.orientation_info.estimated_orientation[2],
                    rec_elem.u.orientation_info.estimated_orientation[3]);
          }
        } break;
        case FILTERED_DATA_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.filtered_data_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          if ((rec_type_without_id & filter_mask) > 0) {
            log_raw("%lu|FILTERED_DATA_INFO|%f|%f", rec_elem.ts,
                    (double)rec_elem.u.filtered_data_info.filtered_altitude_AGL,
                    (double)rec_elem.u.filtered_data_info.filtered_acceleration);
          }
        } break;
        case FLIGHT_STATE: {
          size_t elem_sz = sizeof(rec_elem.u.flight_state);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          if ((rec_type_without_id & filter_mask) > 0) {
            log_raw("%lu|FLIGHT_STATE|%s", rec_elem.ts, fsm_map[rec_elem.u.flight_state]);
          }
        } break;
        case EVENT_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.event_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          if ((rec_type_without_id & filter_mask) > 0) {
            peripheral_act_t action = rec_elem.u.event_info.action;
            log_raw("%lu|EVENT_INFO|%s|%s|%u", rec_elem.ts, event_map[rec_elem.u.event_info.event],
                    action_map[rec_elem.u.event_info.action.action], action.action_arg);
          }
        } break;
        case ERROR_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.error_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          if ((rec_type_without_id & filter_mask) > 0) {
            log_raw("%lu|ERROR_INFO|%lu", rec_elem.ts, rec_elem.u.error_info.error);
          }
        } break;
        case GNSS_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.gnss_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          if ((rec_type_without_id & filter_mask) > 0) {
            log_raw("%lu|GNSS_INFO|%f|%f|%hu", rec_elem.ts, (double)rec_elem.u.gnss_info.lat,
                    (double)rec_elem.u.gnss_info.lon, rec_elem.u.gnss_info.sats);
          }
        } break;
        case VOLTAGE_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.voltage_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          if ((rec_type_without_id & filter_mask) > 0) {
            /* Convert mV to V by dividing with 1000. */
            log_raw("%lu|VOLTAGE_INFO|%.3f", rec_elem.ts, static_cast<double>(rec_elem.u.voltage_info) / 1000);
          }
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
    flight_stats_t *local_flight_stats = (flight_stats_t *)(pvPortMalloc(sizeof(flight_stats_t)));

    if (local_flight_stats == nullptr) {
      log_raw("Could not allocate enough memory for flight stats readout.");
      return;
    }
    if (lfs_file_read(&lfs, &curr_file, local_flight_stats, sizeof(flight_stats_t)) > 0) {
      log_raw("Flight Stats %d", number);
      log_raw("========================");
      log_raw("  Height");
      log_raw("    Time Since Bootup: %lu", local_flight_stats->max_height.ts);
      log_raw("    Max. Height [m]: %f", (double)local_flight_stats->max_height.val);
      log_raw("========================");
      log_raw("  Velocity");
      log_raw("    Time Since Bootup: %lu", local_flight_stats->max_velocity.ts);
      log_raw("    Max. Velocity [m/s]: %f", (double)local_flight_stats->max_velocity.val);
      log_raw("========================");
      log_raw("  Acceleration");
      log_raw("    Time Since Bootup: %lu", local_flight_stats->max_acceleration.ts);
      log_raw("    Max. Acceleration [m/s^2]: %f", (double)local_flight_stats->max_acceleration.val);
      log_raw("========================");
      log_raw("  Calibration Values");
      log_raw("    Height_0: %f", (double)local_flight_stats->height_0);
      log_raw("    IMU:");
      log_raw("      Angle: %f", (double)local_flight_stats->calibration_data.angle);
      log_raw("      Axis: %hu", local_flight_stats->calibration_data.axis);
      log_raw("    Gyro:");
      log_raw("      x: %f", (double)local_flight_stats->calibration_data.gyro_calib.x);
      log_raw("      y: %f", (double)local_flight_stats->calibration_data.gyro_calib.y);
      log_raw("      z: %f", (double)local_flight_stats->calibration_data.gyro_calib.z);
      log_raw("========================");
      log_raw("  Liftoff Time: %02hu:%02hu:%02hu UTC", local_flight_stats->liftoff_time.hour,
              local_flight_stats->liftoff_time.min, local_flight_stats->liftoff_time.sec);
      log_raw("========================");
      log_raw("  Config");
      if (local_flight_stats->config.config_version == CONFIG_VERSION) {
        print_cats_config("cli", &(local_flight_stats->config), false);
      } else {
        log_raw("    Config versions do not match, cannot print -- stats file: %lu, current: %u",
                local_flight_stats->config.config_version, CONFIG_VERSION);
      }

    } else {
      log_raw("Error while reading Stats %d", number);
    }

    vPortFree(local_flight_stats);
    lfs_file_close(&lfs, &curr_file);
  } else {
    log_raw("Stats %d not found!", number);
  }
}
