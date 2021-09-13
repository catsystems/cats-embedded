//
// Created by stoja on 26.12.20.
//

#pragma once

#include "util/types.h"
#include "util/error_handler.h"

#include "cmsis_os.h"

/** Exported Defines **/

#ifdef FLASH_READ_TEST
#undef FLASH_READ_TEST
#endif
//#define FLASH_READ_TEST

#if (configUSE_TRACE_FACILITY == 1)
#define REC_QUEUE_SIZE 256
#else
#define REC_QUEUE_SIZE 512
#endif

#define REC_CMD_QUEUE_SIZE 16

#define REC_QUEUE_PRE_THRUSTING_FILL_RATIO 0.75f
#define REC_QUEUE_PRE_THRUSTING_LIMIT (uint32_t)(REC_QUEUE_PRE_THRUSTING_FILL_RATIO * REC_QUEUE_SIZE)

/** Exported Types **/

typedef enum {
  IMU0 = 0x01,
  IMU1 = 0x02,
  IMU2 = 0x04,
  BARO0 = 0x08,
  BARO1 = 0x10,
  BARO2 = 0x20,
  MAGNETO = 0x40,
  ACCELEROMETER = 0x80,
  FLIGHT_INFO = 0x100,
  ORIENTATION_INFO = 0x200,
  FILTERED_DATA_INFO = 0x400,
  FLIGHT_STATE = 0x800,
  COVARIANCE_INFO = 0x1000,
  SENSOR_INFO = 0x2000,
  EVENT_INFO = 0x4000,
  ERROR_INFO = 0x8000,
  HEHE = 0xFFFFFFFF,
} rec_entry_type_e;

typedef enum {
  REC_CMD_INVALID = 0,
  REC_CMD_FILL_Q = 1,
  REC_CMD_FILL_Q_STOP,
  REC_CMD_WRITE,
  REC_CMD_WRITE_STOP
} rec_cmd_type_e;

typedef struct {
  timestamp_t ts;
  float height_cov;
  float velocity_cov;
} covariance_info_t;

typedef struct {
  timestamp_t ts;
  float height;
  float velocity;
  float acceleration; /* Acceleration with removed offset from inside the KF */
} flight_info_t;

typedef struct {
  timestamp_t ts;
  int16_t estimated_orientation[4];
  int16_t raw_orientation[4];
} orientation_info_t;

typedef struct {
  timestamp_t ts;
  float measured_altitude_AGL; /* Raw Altitude computed from Baro Data averaged */
  float measured_acceleration; /* Raw Altitude computed from Acceleration Data averaged but turned into the right
                                  coordinate frame averaged */
  float filtered_altitude_AGL; /* Median Filtered Values from Baro Data averaged */
  float filtered_acceleration; /* Median Filtered Values from the acceleration but turned into the right coordinate
                                  frame averaged */
} filtered_data_info_t;

typedef union {
  flight_fsm_e flight_state;
  drop_test_fsm_e drop_state;
} flight_state_u;

typedef struct {
  timestamp_t ts;
  flight_state_u flight_or_drop_state;
} flight_state_t;

typedef struct {
  timestamp_t ts;
  uint8_t faulty_imu[3];
  uint8_t faulty_baro[3];
} sensor_info_t;

typedef struct {
  timestamp_t ts;
  cats_event_e event;
  uint8_t action_idx;
} event_info_t;

typedef struct {
  timestamp_t ts;
  cats_error_e error;
} error_info_t;

typedef union {
  imu_data_t imu;
  baro_data_t baro;
  magneto_data_t magneto_info;
  accel_data_t accel_data;
  flight_info_t flight_info;
  orientation_info_t orientation_info;
  filtered_data_info_t filtered_data_info;
  flight_state_t flight_state;
  covariance_info_t covariance_info;
  sensor_info_t sensor_info;
  event_info_t event_info;
  error_info_t error_info;
} rec_elem_u;

typedef struct {
  rec_entry_type_e rec_type;
  rec_elem_u u;
} rec_elem_t;

/** Exported Functions **/

void record(rec_entry_type_e rec_type, const void *rec_value);
