//
// Created by stoja on 26.12.20.
//

#ifndef CATS_REV1_RECORDER_H
#define CATS_REV1_RECORDER_H

#include "util/types.h"
#include "cmsis_os.h"

/** Exported Defines **/

/* TODO: remove this when flash logging is working */
#ifdef FLASH_TESTING
#undef FLASH_TESTING
#endif
//#define FLASH_TESTING

#ifdef FLASH_READ_TEST
#undef FLASH_READ_TEST
#endif
#define FLASH_READ_TEST

/** Exported Consts **/

extern const uint32_t REC_QUEUE_SIZE;

/** Exported Types **/

typedef enum {
  IMU0 = 1,
  IMU1,
  IMU2,
  BARO0,
  BARO1,
  BARO2,
  FLIGHT_INFO,
  FLIGHT_STATE,
  COVARIANCE_INFO,
  HEHE = 0x7FFFFFFF /* TODO <- optimize these enums and remove this guy */
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
} flight_info_t;

typedef struct {
  timestamp_t ts;
  flight_fsm_e flight_state;
} flight_state_t;

typedef union {
  imu_data_t imu;
  baro_data_t baro;
  flight_info_t flight_info;
  flight_state_t flight_state;
  covariance_info_t covariance_info;
} rec_elem_u;

typedef struct {
  rec_entry_type_e rec_type;
  rec_elem_u u;
} rec_elem_t;

/** Exported Functions **/

void record(rec_entry_type_e rec_type, const void *rec_value);

#endif  // CATS_REV1_RECORDER_H
