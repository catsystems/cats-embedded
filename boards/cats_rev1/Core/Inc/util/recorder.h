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

/** Exported Consts **/

extern const uint32_t REC_QUEUE_SIZE;
extern const uint32_t REC_QUEUE_PRE_THRUSTING_LIMIT;

/** Exported Types **/

typedef enum {
  IMU0 = 0x01,
  IMU1 = 0x02,
  IMU2 = 0x04,
  BARO0 = 0x08,
  BARO1 = 0x10,
  BARO2 = 0x20,
  FLIGHT_INFO = 0x40,
  FLIGHT_STATE = 0x80,
  COVARIANCE_INFO = 0x100,
  SENSOR_INFO = 0x200,
  EVENT_INFO = 0x400,
  ERROR_INFO = 0x800,
  HEHE = 0xFFFFFFFF /* TODO <- optimize these enums and remove this guy */
} rec_entry_type_e;

typedef struct {
  timestamp_t ts;
  float height_cov;
  float velocity_cov;
} covariance_info_t;

typedef struct {
  timestamp_t ts;
  float height;
  float velocity;
  float acceleration;
  float measured_altitude_AGL;
} flight_info_t;

typedef struct {
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
  flight_info_t flight_info;
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
