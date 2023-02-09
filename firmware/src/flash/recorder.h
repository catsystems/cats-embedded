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

#pragma once

#include "config/cats_config.h"
#include "util/error_handler.h"
#include "util/gnss.h"
#include "util/types.hpp"

#include "arm_math.h"
#include "cmsis_os.h"

/** Exported Defines **/

#define REC_QUEUE_SIZE 512

#define REC_CMD_QUEUE_SIZE 16

#define MAX_FILENAME_SIZE 32

#define REC_QUEUE_PRE_THRUSTING_FILL_RATIO 0.75f
#define REC_QUEUE_PRE_THRUSTING_LIMIT      (uint32_t)(REC_QUEUE_PRE_THRUSTING_FILL_RATIO * REC_QUEUE_SIZE)

/**
 * A bit mask that specifies where the IDs are located. The IDs occupy the first four bits of the rec_entry_type_e enum.
 */
#define REC_ID_MASK 0x0000000F

/** Exported Types **/

// clang-format off
 enum rec_entry_type_e: uint32_t {
  // Periodic recorder types, their recording speed is affected by global_cats_config.rec_speed_idx
  IMU                = 1 << 4,   // 0x20
  BARO               = 1 << 5,   // 0x40
  FLIGHT_INFO        = 1 << 6,   // 0x80
  ORIENTATION_INFO   = 1 << 7,   // 0x100
  FILTERED_DATA_INFO = 1 << 8,   // 0x200
// Sporadic recorder types, they will always be logged
  FLIGHT_STATE       = 1 << 9,   // 0x400
  EVENT_INFO         = 1 << 10,  // 0x800
  ERROR_INFO         = 1 << 11,  // 0x1000
  GNSS_INFO          = 1 << 12,  // 0x2000
  VOLTAGE_INFO       = 1 << 13,  // 0x4000
};
// clang-format on

enum rec_cmd_type_e { REC_CMD_INVALID = 0, REC_CMD_FILL_Q = 1, REC_CMD_FILL_Q_STOP, REC_CMD_WRITE, REC_CMD_WRITE_STOP };

struct flight_info_t {
  float32_t height;
  float32_t velocity;
  float32_t acceleration; /* Acceleration with removed offset from inside the KF */
};

struct orientation_info_t {
  int16_t estimated_orientation[4];
};

struct filtered_data_info_t {
  float32_t filtered_altitude_AGL; /* Averaged median-filtered values from Baro data. */
  float32_t filtered_acceleration; /* Averaged median-filtered values from acceleration converted into the correct
                                  coordinate frame. */
};

struct event_info_t {
  cats_event_e event;
  peripheral_act_t action;
};

struct error_info_t {
  cats_error_e error;
};

/* Voltage in mV */
using voltage_info_t = uint16_t;

union rec_elem_u {
  imu_data_t imu;
  baro_data_t baro;
  flight_info_t flight_info;
  orientation_info_t orientation_info;
  filtered_data_info_t filtered_data_info;
  flight_fsm_e flight_state;
  event_info_t event_info;
  error_info_t error_info;
  gnss_position_t gnss_info;
  voltage_info_t voltage_info;
};

struct rec_elem_t {
  timestamp_t ts;
  rec_entry_type_e rec_type;
  rec_elem_u u;
};

/* Flight Statistics */

struct flight_stats_t {
  cats_config_t config;
  struct {
    timestamp_t ts;
    float32_t val;
  } max_height;

  struct {
    timestamp_t ts;
    float32_t val;
  } max_velocity;

  struct {
    timestamp_t ts;
    float32_t val;
  } max_acceleration;

  calibration_data_t calibration_data;
  float32_t height_0;

  gnss_time_t liftoff_time;
};

/** Exported Variables **/
extern flight_stats_t global_flight_stats;

/** Exported Functions **/

void record(timestamp_t ts, rec_entry_type_e rec_type_with_id, const void *rec_value);

inline void init_global_flight_stats() {
  /* Save current flight config */
  memcpy(&global_flight_stats.config, &global_cats_config, sizeof(global_cats_config));
  /* Initialize nonzero members */
  global_flight_stats.max_height.val = -INFINITY;
  global_flight_stats.max_velocity.val = -INFINITY;
  global_flight_stats.max_acceleration.val = -INFINITY;
}

/**
 * Extract only the pure record type by clearing the ID mask bits.
 *
 * @param rec_type record type with or without ID
 * @return record type without ID
 */
inline rec_entry_type_e get_record_type_without_id(rec_entry_type_e rec_type) {
  return (rec_entry_type_e)(rec_type & ~REC_ID_MASK);
}

/**
 * Add the ID information to the given record type.
 *
 * @param rec_type record type without ID
 * @param id - identifier of the record element; should be between 0 & 15
 * @return record type with ID
 */
inline rec_entry_type_e add_id_to_record_type(rec_entry_type_e rec_type, uint8_t id) {
  return (rec_entry_type_e)(rec_type | (id & REC_ID_MASK));
}

/**
 * Strip REC_ID_MASK from the rec_entry_type_e enum and return the pure record type.
 *
 * @param rec_type record type information with ID
 * @return ID of the record element without record type information
 */
inline uint8_t get_id_from_record_type(rec_entry_type_e rec_type) { return rec_type & REC_ID_MASK; }
