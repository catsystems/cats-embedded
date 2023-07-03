/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
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

#include "reader.hpp"

#include <cstddef>
#include <cstdio>

#include "cli/settings.hpp"
#include "config/globals.hpp"
#include "flash/lfs_custom.hpp"
#include "recorder.hpp"
#include "util/enum_str_maps.hpp"
#include "util/log.h"

#define STRING_BUF_SZ 400
#define READ_BUF_SZ   256

namespace {

/**
 * Prints the stats by reading them from the stats txt file.
 * The file is read READ_BUF_SZ bytes at a time.
 */
void print_stats(uint16_t flight_num) {
  if (global_recorder_status == REC_WRITE_TO_FLASH) {
    log_raw("The recorder is currently active, stop it first!");
    return;
  }

  char filename[MAX_FILENAME_SIZE] = {};
  snprintf(filename, MAX_FILENAME_SIZE, "stats/stats_%05d.txt", flight_num);

  log_raw("Reading file: %s", filename);

  lfs_file_t curr_file;

  if (lfs_file_open(&lfs, &curr_file, filename, LFS_O_RDONLY) == LFS_ERR_OK) {
    const auto file_size = lfs_file_size(&lfs, &curr_file);
    if (file_size > 0) {
      uint8_t *read_buf = (uint8_t *)(pvPortMalloc(READ_BUF_SZ * sizeof(uint8_t)));

      for (lfs_size_t i = 0; i < static_cast<lfs_size_t>(file_size); i += READ_BUF_SZ) {
        memset(read_buf, 0, READ_BUF_SZ);
        lfs_size_t bytes_read = lfs_min(READ_BUF_SZ, file_size - i);

        lfs_file_read(&lfs, &curr_file, read_buf, bytes_read);

        log_rawr("%.*s", static_cast<int>(bytes_read), reinterpret_cast<const char *>(read_buf));
      }

      vPortFree(read_buf);
    }
    lfs_file_close(&lfs, &curr_file);
  } else {
    log_raw("Stats %d not found!", flight_num);
  }
}

/**
 * Prints the config for a given flight by reading it from the configs directory.
 * The config is stored in binary format and is printed by calling print_cats_config CLI function.
 */
void print_cfg(uint16_t flight_num) {
  if (global_recorder_status == REC_WRITE_TO_FLASH) {
    log_raw("The recorder is currently active, stop it first!");
    return;
  }

  char filename[MAX_FILENAME_SIZE] = {};
  snprintf(filename, MAX_FILENAME_SIZE, "configs/flight_%05d.cfg", flight_num);

  log_raw("Reading file: %s", filename);

  lfs_file_t curr_file;

  if (lfs_file_open(&lfs, &curr_file, filename, LFS_O_RDONLY) == LFS_ERR_OK) {
    auto *local_config = static_cast<cats_config_t *>(pvPortMalloc(sizeof(cats_config_t)));

    if (local_config == nullptr) {
      log_raw("Could not allocate enough memory for flight config readout.");
      lfs_file_close(&lfs, &curr_file);
      return;
    }
    if (lfs_file_read(&lfs, &curr_file, local_config, sizeof(flight_stats_t)) > 0) {
      if (local_config->config_version == CONFIG_VERSION) {
        print_cats_config("cli", local_config, false);
      } else {
        log_raw("Config versions do not match, cannot print -- version in config file: %lu, current config version: %u",
                local_config->config_version, CONFIG_VERSION);
      }

    } else {
      log_raw("Error while reading flight config %d", flight_num);
    }

    vPortFree(local_config);
    lfs_file_close(&lfs, &curr_file);
  } else {
    log_raw("Flight config %d not found!", flight_num);
  }
}

}  // namespace

namespace reader {

void dump_recording(uint16_t flight_num) {
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
  snprintf(filename, MAX_FILENAME_SIZE, "flights/flight_%05d", flight_num);

  log_raw("Dumping file: %s", filename);

  lfs_file_t curr_file;
  if (lfs_file_open(&lfs, &curr_file, filename, LFS_O_RDONLY) == LFS_ERR_OK) {
    const auto file_size = lfs_file_size(&lfs, &curr_file);
    if (file_size > 0) {
      for (lfs_size_t i = 0; i < static_cast<lfs_size_t>(file_size); i += READ_BUF_SZ) {
        lfs_size_t chunk = lfs_min(READ_BUF_SZ, file_size - i);

        lfs_file_read(&lfs, &curr_file, read_buf, chunk);

        int write_idx = 0;
        for (uint32_t j = 0; j < READ_BUF_SZ / 2; ++j) {
          write_idx += sprintf(string_buffer1 + write_idx, "%02x ", read_buf[j]);
        }
        log_rawr("%s", string_buffer1);
        write_idx = 0;
        for (uint32_t j = READ_BUF_SZ / 2; j < READ_BUF_SZ; ++j) {
          write_idx += sprintf(string_buffer2 + write_idx, "%02x ", read_buf[j]);
        }
        log_rawr("%s\n", string_buffer2);

        memset(string_buffer1, 0, STRING_BUF_SZ);
        memset(string_buffer2, 0, STRING_BUF_SZ);
      }
    }
  } else {
    log_error("Flight %d not found!", flight_num);
  }

  lfs_file_close(&lfs, &curr_file);

  vPortFree(string_buffer1);
  vPortFree(string_buffer2);
  vPortFree(read_buf);
}

void parse_recording(uint16_t flight_num, rec_entry_type_e filter_mask) {
  if (global_recorder_status == REC_WRITE_TO_FLASH) {
    log_raw("The recorder is currently active, stop it first!");
    return;
  }

  char filename[MAX_FILENAME_SIZE] = {};
  snprintf(filename, MAX_FILENAME_SIZE, "flights/flight_%05d", flight_num);

  log_raw("Reading file: %s", filename);

  lfs_file_t curr_file;
  if (lfs_file_open(&lfs, &curr_file, filename, LFS_O_RDONLY) == LFS_ERR_OK) {
    rec_elem_t rec_elem;
    lfs_ssize_t file_size = lfs_file_size(&lfs, &curr_file);
    if (file_size < 0) {
      log_raw("Invalid file size %ld!", file_size);
      return;
    }

    // First bytes represent the code version
    char tmp_char = '\0';
    while (lfs_file_read(&lfs, &curr_file, &tmp_char, 1) > 0) {
      log_raw("read char: %hu", tmp_char);
      // Read until we encounter the NULL terminator
      if (tmp_char == 0) {
        break;
      }
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
            log_raw("%lu|BARO%hu|%ld|%ld", rec_elem.ts, get_id_from_record_type(rec_type), rec_elem.u.baro.pressure,
                    rec_elem.u.baro.temperature);
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
            log_raw("%lu|FLIGHT_STATE|%s", rec_elem.ts, GetStr(rec_elem.u.flight_state, fsm_map));
          }
        } break;
        case EVENT_INFO: {
          size_t elem_sz = sizeof(rec_elem.u.event_info);
          lfs_file_read(&lfs, &curr_file, (uint8_t *)&rec_elem.u.imu, elem_sz);
          if ((rec_type_without_id & filter_mask) > 0) {
            peripheral_act_t action = rec_elem.u.event_info.action;
            log_raw("%lu|EVENT_INFO|%s|%s|%d", rec_elem.ts, GetStr(rec_elem.u.event_info.event, event_map),
                    GetStr(rec_elem.u.event_info.action.action, action_map), action.action_arg);
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
          log_raw("Impossible recorder entry type: %lu!", rec_type_without_id);
          break;
      }
    }
    lfs_file_close(&lfs, &curr_file);
  } else {
    log_raw("Flight %d not found!", flight_num);
  }
}

void print_stats_and_cfg(uint16_t flight_num) {
  print_stats(flight_num);
  print_cfg(flight_num);
}

}  // namespace reader
